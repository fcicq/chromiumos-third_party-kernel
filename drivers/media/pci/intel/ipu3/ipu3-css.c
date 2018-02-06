/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/gcd.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/swab.h>

#include "ipu3-css.h"
#include "ipu3-css-fw.h"
#include "ipu3-css-params.h"
#include "ipu3-tables.h"

/* IRQ configuration */
#define IMGU_IRQCTRL_IRQ_MASK	(IMGU_IRQCTRL_IRQ_SP1 | \
				 IMGU_IRQCTRL_IRQ_SP2 | \
				 IMGU_IRQCTRL_IRQ_SW_PIN(0) | \
				 IMGU_IRQCTRL_IRQ_SW_PIN(1))

#define IPU3_CSS_FORMAT_BPP_DEN	50	/* Denominator */

/* Some sane limits for resolutions */
#define IPU3_CSS_MIN_RES	32
#define IPU3_CSS_MAX_RES	65535

/* filter size from graph settings is fixed as 4 */
#define FILTER_SIZE             4
#define MIN_ENVELOPE            8

/*
 * pre-allocated buffer size for CSS ABI, auxiliary frames
 * after BDS and before GDC. Those values should be tuned
 * to big enough to avoid buffer re-allocation when
 * streaming to lower streaming latency.
 */
#define CSS_ABI_SIZE    136
#define CSS_BDS_SIZE    (4480 * 3200 * 3)
#define CSS_GDC_SIZE    (4224 * 3200 * 12 / 8)

/* Formats supported by IPU3 Camera Sub System */
static const struct ipu3_css_format ipu3_css_formats[] = {
	{
		.pixelformat = V4L2_PIX_FMT_NV12,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.frame_format = IMGU_ABI_FRAME_FORMAT_NV12,
		.osys_format = IMGU_ABI_OSYS_FORMAT_NV12,
		.osys_tiling = IMGU_ABI_OSYS_TILING_NONE,
		.bytesperpixel_num = 1 * IPU3_CSS_FORMAT_BPP_DEN,
		.chroma_decim = 4,
		.width_align = IPU3_UAPI_ISP_VEC_ELEMS,
		.flags = IPU3_CSS_FORMAT_FL_OUT | IPU3_CSS_FORMAT_FL_VF,
	}, {
		/* Each 32 bytes contains 25 10-bit pixels */
		.pixelformat = V4L2_PIX_FMT_IPU3_SBGGR10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.frame_format = IMGU_ABI_FRAME_FORMAT_RAW_PACKED,
		.bayer_order = IMGU_ABI_BAYER_ORDER_BGGR,
		.bit_depth = 10,
		.bytesperpixel_num = 64,
		.width_align = 2 * IPU3_UAPI_ISP_VEC_ELEMS,
		.flags = IPU3_CSS_FORMAT_FL_IN,
	}, {
		.pixelformat = V4L2_PIX_FMT_IPU3_SGBRG10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.frame_format = IMGU_ABI_FRAME_FORMAT_RAW_PACKED,
		.bayer_order = IMGU_ABI_BAYER_ORDER_GBRG,
		.bit_depth = 10,
		.bytesperpixel_num = 64,
		.width_align = 2 * IPU3_UAPI_ISP_VEC_ELEMS,
		.flags = IPU3_CSS_FORMAT_FL_IN,
	}, {
		.pixelformat = V4L2_PIX_FMT_IPU3_SGRBG10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.frame_format = IMGU_ABI_FRAME_FORMAT_RAW_PACKED,
		.bayer_order = IMGU_ABI_BAYER_ORDER_GRBG,
		.bit_depth = 10,
		.bytesperpixel_num = 64,
		.width_align = 2 * IPU3_UAPI_ISP_VEC_ELEMS,
		.flags = IPU3_CSS_FORMAT_FL_IN,
	}, {
		.pixelformat = V4L2_PIX_FMT_IPU3_SRGGB10,
		.colorspace = V4L2_COLORSPACE_RAW,
		.frame_format = IMGU_ABI_FRAME_FORMAT_RAW_PACKED,
		.bayer_order = IMGU_ABI_BAYER_ORDER_RGGB,
		.bit_depth = 10,
		.bytesperpixel_num = 64,
		.width_align = 2 * IPU3_UAPI_ISP_VEC_ELEMS,
		.flags = IPU3_CSS_FORMAT_FL_IN,
	},
};

static const struct {
	enum imgu_abi_queue_id qid;
	size_t ptr_ofs;
} ipu3_css_queues[IPU3_CSS_QUEUES] = {
	[IPU3_CSS_QUEUE_IN] = {
		IMGU_ABI_QUEUE_C_ID,
		offsetof(struct imgu_abi_buffer, payload.frame.frame_data)
	},
	[IPU3_CSS_QUEUE_OUT] = {
		IMGU_ABI_QUEUE_D_ID,
		offsetof(struct imgu_abi_buffer, payload.frame.frame_data)
	},
	[IPU3_CSS_QUEUE_VF] = {
		IMGU_ABI_QUEUE_E_ID,
		offsetof(struct imgu_abi_buffer, payload.frame.frame_data)
	},
	[IPU3_CSS_QUEUE_STAT_3A] = {
		IMGU_ABI_QUEUE_F_ID,
		offsetof(struct imgu_abi_buffer, payload.s3a.data_ptr)
	},
	[IPU3_CSS_QUEUE_STAT_DVS] = {
		IMGU_ABI_QUEUE_G_ID,
		offsetof(struct imgu_abi_buffer, payload.skc_dvs_statistics)
	}
};

/* Initialize queue based on given format, adjust format as needed */
static int ipu3_css_queue_init(struct ipu3_css_queue *queue,
			       struct v4l2_pix_format_mplane *fmt, u32 flags)
{
	struct v4l2_pix_format_mplane *const f = &queue->fmt.mpix;
	unsigned int i;
	u32 sizeimage;

	INIT_LIST_HEAD(&queue->bufs);

	queue->css_fmt = NULL;	/* Disable */
	if (!fmt)
		return 0;

	for (i = 0; i < ARRAY_SIZE(ipu3_css_formats); i++) {
		if (!(ipu3_css_formats[i].flags & flags))
			continue;
		queue->css_fmt = &ipu3_css_formats[i];
		if (ipu3_css_formats[i].pixelformat == fmt->pixelformat)
			break;
	}
	if (!queue->css_fmt)
		return -EINVAL;	/* Could not find any suitable format */

	queue->fmt.mpix = *fmt;

	f->width = ALIGN(clamp_t(u32, f->width,
			IPU3_CSS_MIN_RES, IPU3_CSS_MAX_RES), 2);
	f->height = ALIGN(clamp_t(u32, f->height,
			IPU3_CSS_MIN_RES, IPU3_CSS_MAX_RES), 2);
	queue->width_pad = ALIGN(f->width, queue->css_fmt->width_align);
	if (queue->css_fmt->frame_format != IMGU_ABI_FRAME_FORMAT_RAW_PACKED)
		f->plane_fmt[0].bytesperline = DIV_ROUND_UP(queue->width_pad *
					queue->css_fmt->bytesperpixel_num,
					IPU3_CSS_FORMAT_BPP_DEN);
	else
		/* For packed raw, alignment for bpl is by 50 to the width */
		f->plane_fmt[0].bytesperline = DIV_ROUND_UP(f->width,
					IPU3_CSS_FORMAT_BPP_DEN) *
					queue->css_fmt->bytesperpixel_num;
	sizeimage = f->height * f->plane_fmt[0].bytesperline;

	if (queue->css_fmt->chroma_decim)
		sizeimage += 2 * sizeimage / queue->css_fmt->chroma_decim;

	f->plane_fmt[0].sizeimage = sizeimage;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = queue->css_fmt->colorspace;
	f->num_planes = 1;
	f->flags = 0;
	f->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->quantization = V4L2_QUANTIZATION_DEFAULT;
	f->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	memset(f->reserved, 0, sizeof(f->reserved));

	return 0;
}

static bool ipu3_css_queue_enabled(struct ipu3_css_queue *q)
{
	return q->css_fmt != NULL;
}

/******************* css hw *******************/

static void writes(void *mem, ssize_t len, void __iomem *reg)
{
	while (len >= 4) {
		writel(*(u32 *)mem, reg);
		mem += 4;
		reg += 4;
		len -= 4;
	}
}

/* Wait until register `reg', masked with `mask', becomes `cmp' */
static int ipu3_hw_wait(void __iomem *base, int reg, u32 mask, u32 cmp)
{
	u32 val;

	return readl_poll_timeout(base + reg, val, (val & mask) == cmp,
				  1000, 100 * 1000);
}

/* Initialize the IPU3 CSS hardware and associated h/w blocks */

int ipu3_css_set_powerup(struct device *dev, void __iomem *base)
{
	static const unsigned int freq = 450;
	u32 pm_ctrl, state, val;

	dev_dbg(dev, "power up.\n");
	/* Clear the CSS busy signal */
	readl(base + IMGU_REG_GP_BUSY);
	writel(0, base + IMGU_REG_GP_BUSY);

	/* Wait for idle signal */
	if (ipu3_hw_wait(base, IMGU_REG_STATE, IMGU_STATE_IDLE_STS,
			 IMGU_STATE_IDLE_STS)) {
		dev_err(dev, "failed to set CSS idle\n");
		goto fail;
	}

	/* Reset the css */
	writel(readl(base + IMGU_REG_PM_CTRL) | IMGU_PM_CTRL_FORCE_RESET,
		base + IMGU_REG_PM_CTRL);

	usleep_range(200, 300);

	/** Prepare CSS */

	pm_ctrl = readl(base + IMGU_REG_PM_CTRL);
	state = readl(base + IMGU_REG_STATE);

	dev_dbg(dev, "CSS pm_ctrl 0x%x state 0x%x (power %s)\n",
		pm_ctrl, state, state & IMGU_STATE_POWER_DOWN ? "down" : "up");

	/* Power up CSS using wrapper */
	if (state & IMGU_STATE_POWER_DOWN) {
		writel(IMGU_PM_CTRL_RACE_TO_HALT | IMGU_PM_CTRL_START,
			base + IMGU_REG_PM_CTRL);
		if (ipu3_hw_wait(base, IMGU_REG_PM_CTRL,
				 IMGU_PM_CTRL_START, 0)) {
			dev_err(dev, "failed to power up CSS\n");
			goto fail;
		}
		usleep_range(2000, 3000);
	} else {
		writel(IMGU_PM_CTRL_RACE_TO_HALT, base + IMGU_REG_PM_CTRL);
	}

	/* Set the busy bit */
	writel(readl(base + IMGU_REG_GP_BUSY) | 1, base + IMGU_REG_GP_BUSY);

	/* Set CSS clock frequency */
	pm_ctrl = readl(base + IMGU_REG_PM_CTRL);
	val = pm_ctrl & ~(IMGU_PM_CTRL_CSS_PWRDN | IMGU_PM_CTRL_RST_AT_EOF);
	writel(val, base + IMGU_REG_PM_CTRL);
	writel(0, base + IMGU_REG_GP_BUSY);
	if (ipu3_hw_wait(base, IMGU_REG_STATE,
			 IMGU_STATE_PWRDNM_FSM_MASK, 0)) {
		dev_err(dev, "failed to pwrdn CSS\n");
		goto fail;
	}
	val = (freq / IMGU_SYSTEM_REQ_FREQ_DIVIDER) & IMGU_SYSTEM_REQ_FREQ_MASK;
	writel(val, base + IMGU_REG_SYSTEM_REQ);
	writel(1, base + IMGU_REG_GP_BUSY);
	writel(readl(base + IMGU_REG_PM_CTRL) | IMGU_PM_CTRL_FORCE_HALT,
		base + IMGU_REG_PM_CTRL);
	if (ipu3_hw_wait(base, IMGU_REG_STATE, IMGU_STATE_HALT_STS,
			 IMGU_STATE_HALT_STS)) {
		dev_err(dev, "failed to halt CSS\n");
		goto fail;
	}

	writel(readl(base + IMGU_REG_PM_CTRL) | IMGU_PM_CTRL_START,
		base + IMGU_REG_PM_CTRL);
	if (ipu3_hw_wait(base, IMGU_REG_PM_CTRL, IMGU_PM_CTRL_START, 0)) {
		dev_err(dev, "failed to start CSS\n");
		goto fail;
	}
	writel(readl(base + IMGU_REG_PM_CTRL) | IMGU_PM_CTRL_FORCE_UNHALT,
		base + IMGU_REG_PM_CTRL);

	val = readl(base + IMGU_REG_PM_CTRL);	/* get pm_ctrl */
	val &= ~(IMGU_PM_CTRL_CSS_PWRDN | IMGU_PM_CTRL_RST_AT_EOF);
	val |= pm_ctrl & (IMGU_PM_CTRL_CSS_PWRDN | IMGU_PM_CTRL_RST_AT_EOF);
	writel(val, base + IMGU_REG_PM_CTRL);

	return 0;

fail:
	ipu3_css_set_powerdown(dev, base);
	return -EIO;
}

int ipu3_css_set_powerdown(struct device *dev, void __iomem *base)
{
	dev_dbg(dev, "power down.\n");

	/* wait for cio idle signal */
	if (ipu3_hw_wait(base, IMGU_REG_CIO_GATE_BURST_STATE,
			 IMGU_CIO_GATE_BURST_MASK, 0))
		dev_warn(dev, "wait cio gate idle timeout");

	/* wait for css idle signal */
	if (ipu3_hw_wait(base, IMGU_REG_STATE, IMGU_STATE_IDLE_STS,
			 IMGU_STATE_IDLE_STS))
		dev_warn(dev, "wait css idle timeout\n");

	/* do halt-halted handshake with css */
	writel(1, base + IMGU_REG_GP_HALT);
	if (ipu3_hw_wait(base, IMGU_REG_STATE, IMGU_STATE_HALT_STS,
			     IMGU_STATE_HALT_STS))
		dev_warn(dev, "failed to halt css");

	/* de-assert the busy bit */
	writel(0, base + IMGU_REG_GP_BUSY);

	return 0;
}

static void ipu3_css_hw_enable_irq(struct ipu3_css *css)
{
	void __iomem *const base = css->base;
	u32 val, i;

	/* Set up interrupts */

	/*
	 * Enable IRQ on the SP which signals that SP goes to idle
	 * (aka ready state) and set trigger to pulse
	 */
	val = readl(base + IMGU_REG_SP_CTRL(0)) | IMGU_CTRL_IRQ_READY;
	writel(val, base + IMGU_REG_SP_CTRL(0));
	writel(val | IMGU_CTRL_IRQ_CLEAR, base + IMGU_REG_SP_CTRL(0));

	/* Enable IRQs from the IMGU wrapper */
	writel(IMGU_REG_INT_CSS_IRQ, base + IMGU_REG_INT_ENABLE);
	/* Clear */
	writel(IMGU_REG_INT_CSS_IRQ, base + IMGU_REG_INT_STATUS);

	/* Enable IRQs from main IRQ controller */
	writel(~0, base + IMGU_REG_IRQCTRL_EDGE_NOT_PULSE(IMGU_IRQCTRL_MAIN));
	writel(0, base + IMGU_REG_IRQCTRL_MASK(IMGU_IRQCTRL_MAIN));
	writel(IMGU_IRQCTRL_IRQ_MASK,
		base + IMGU_REG_IRQCTRL_EDGE(IMGU_IRQCTRL_MAIN));
	writel(IMGU_IRQCTRL_IRQ_MASK,
		base + IMGU_REG_IRQCTRL_ENABLE(IMGU_IRQCTRL_MAIN));
	writel(IMGU_IRQCTRL_IRQ_MASK,
		base + IMGU_REG_IRQCTRL_CLEAR(IMGU_IRQCTRL_MAIN));
	writel(IMGU_IRQCTRL_IRQ_MASK,
		base + IMGU_REG_IRQCTRL_MASK(IMGU_IRQCTRL_MAIN));
	/* Wait for write complete */
	readl(base + IMGU_REG_IRQCTRL_ENABLE(IMGU_IRQCTRL_MAIN));

	/* Enable IRQs from SP0 and SP1 controllers */
	for (i = IMGU_IRQCTRL_SP0; i <= IMGU_IRQCTRL_SP1; i++) {
		writel(~0, base + IMGU_REG_IRQCTRL_EDGE_NOT_PULSE(i));
		writel(0, base + IMGU_REG_IRQCTRL_MASK(i));
		writel(IMGU_IRQCTRL_IRQ_MASK, base + IMGU_REG_IRQCTRL_EDGE(i));
		writel(IMGU_IRQCTRL_IRQ_MASK,
			base + IMGU_REG_IRQCTRL_ENABLE(i));
		writel(IMGU_IRQCTRL_IRQ_MASK, base + IMGU_REG_IRQCTRL_CLEAR(i));
		writel(IMGU_IRQCTRL_IRQ_MASK, base + IMGU_REG_IRQCTRL_MASK(i));
		/* Wait for write complete */
		readl(base + IMGU_REG_IRQCTRL_ENABLE(i));
	}
}

static int ipu3_css_hw_init(struct ipu3_css *css)
{
	/* For checking that streaming monitor statuses are valid */
	static const struct {
		u32 reg;
		u32 mask;
		const char *name;
	} stream_monitors[] = {
		{
			IMGU_REG_GP_SP1_STRMON_STAT,
			IMGU_GP_STRMON_STAT_ISP_PORT_SP12ISP,
			"ISP0 to SP0"
		}, {
			IMGU_REG_GP_ISP_STRMON_STAT,
			IMGU_GP_STRMON_STAT_SP1_PORT_ISP2SP1,
			"SP0 to ISP0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_ISP2DMA,
			"ISP0 to DMA0"
		}, {
			IMGU_REG_GP_ISP_STRMON_STAT,
			IMGU_GP_STRMON_STAT_ISP_PORT_DMA2ISP,
			"DMA0 to ISP0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_CELLS2GDC,
			"ISP0 to GDC0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_GDC2CELLS,
			"GDC0 to ISP0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_SP12DMA,
			"SP0 to DMA0"
		}, {
			IMGU_REG_GP_SP1_STRMON_STAT,
			IMGU_GP_STRMON_STAT_SP1_PORT_DMA2SP1,
			"DMA0 to SP0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_CELLS2GDC,
			"SP0 to GDC0"
		}, {
			IMGU_REG_GP_MOD_STRMON_STAT,
			IMGU_GP_STRMON_STAT_MOD_PORT_GDC2CELLS,
			"GDC0 to SP0"
		},
	};

	struct device *dev = css->dev;
	void __iomem *const base = css->base;
	u32 val, i;

	/* Set instruction cache address and inv bit for ISP, SP, and SP1 */
	for (i = 0; i < IMGU_NUM_SP; i++) {
		struct imgu_fw_info *bi =
			&css->fwp->binary_header[css->fw_sp[i]];

		writel(css->binary[css->fw_sp[i]].daddr,
			base + IMGU_REG_SP_ICACHE_ADDR(bi->type));
		writel(readl(base + IMGU_REG_SP_CTRL(bi->type)) |
			IMGU_CTRL_ICACHE_INV,
			base + IMGU_REG_SP_CTRL(bi->type));
	}
	writel(css->binary[css->fw_bl].daddr, base + IMGU_REG_ISP_ICACHE_ADDR);
	writel(readl(base + IMGU_REG_ISP_CTRL) | IMGU_CTRL_ICACHE_INV,
		base + IMGU_REG_ISP_CTRL);

	/* Check that IMGU hardware is ready */

	if (!(readl(base + IMGU_REG_SP_CTRL(0)) & IMGU_CTRL_IDLE)) {
		dev_err(dev, "SP is not idle\n");
		return -EIO;
	}
	if (!(readl(base + IMGU_REG_ISP_CTRL) & IMGU_CTRL_IDLE)) {
		dev_err(dev, "ISP is not idle\n");
		return -EIO;
	}

	for (i = 0; i < ARRAY_SIZE(stream_monitors); i++) {
		val = readl(base + stream_monitors[i].reg);
		if (val & stream_monitors[i].mask) {
			dev_err(dev, "error: Stream monitor %s is valid\n",
				stream_monitors[i].name);
			return -EIO;
		}
	}

	/* Initialize GDC with default values */

	for (i = 0; i < ARRAY_SIZE(ipu3_css_gdc_lut[0]); i++) {
		u32 val0 = ipu3_css_gdc_lut[0][i] & IMGU_GDC_LUT_MASK;
		u32 val1 = ipu3_css_gdc_lut[1][i] & IMGU_GDC_LUT_MASK;
		u32 val2 = ipu3_css_gdc_lut[2][i] & IMGU_GDC_LUT_MASK;
		u32 val3 = ipu3_css_gdc_lut[3][i] & IMGU_GDC_LUT_MASK;

		writel(val0 | (val1 << 16),
			base + IMGU_REG_GDC_LUT_BASE + i * 8);
		writel(val2 | (val3 << 16),
			base + IMGU_REG_GDC_LUT_BASE + i * 8 + 4);
	};

	return 0;
}

/* Boot the given IPU3 CSS SP */
static int ipu3_css_hw_start_sp(struct ipu3_css *css, int sp)
{
	void __iomem *const base = css->base;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[sp]];
	struct imgu_abi_sp_init_dmem_cfg dmem_cfg = {
		.ddr_data_addr = css->binary[css->fw_sp[sp]].daddr
			+ bi->blob.data_source,
		.dmem_data_addr = bi->blob.data_target,
		.dmem_bss_addr = bi->blob.bss_target,
		.data_size = bi->blob.data_size,
		.bss_size = bi->blob.bss_size,
		.sp_id = sp,
	};

	writes(&dmem_cfg, sizeof(dmem_cfg), base +
		IMGU_REG_SP_DMEM_BASE(sp) + bi->info.sp.init_dmem_data);

	writel(bi->info.sp.sp_entry, base + IMGU_REG_SP_START_ADDR(sp));

	writel(readl(base + IMGU_REG_SP_CTRL(sp))
		| IMGU_CTRL_START | IMGU_CTRL_RUN, base + IMGU_REG_SP_CTRL(sp));

	if (ipu3_hw_wait(css->base, IMGU_REG_SP_DMEM_BASE(sp)
			 + bi->info.sp.sw_state,
			 ~0, IMGU_ABI_SP_SWSTATE_INITIALIZED))
		return -EIO;

	return 0;
}

/* Start the IPU3 CSS ImgU (Imaging Unit) and all the SPs */
static int ipu3_css_hw_start(struct ipu3_css *css)
{
	static const u32 event_mask =
		((1 << IMGU_ABI_EVTTYPE_OUT_FRAME_DONE) |
		(1 << IMGU_ABI_EVTTYPE_2ND_OUT_FRAME_DONE) |
		(1 << IMGU_ABI_EVTTYPE_VF_OUT_FRAME_DONE) |
		(1 << IMGU_ABI_EVTTYPE_2ND_VF_OUT_FRAME_DONE) |
		(1 << IMGU_ABI_EVTTYPE_3A_STATS_DONE) |
		(1 << IMGU_ABI_EVTTYPE_DIS_STATS_DONE) |
		(1 << IMGU_ABI_EVTTYPE_PIPELINE_DONE) |
		(1 << IMGU_ABI_EVTTYPE_FRAME_TAGGED) |
		(1 << IMGU_ABI_EVTTYPE_INPUT_FRAME_DONE) |
		(1 << IMGU_ABI_EVTTYPE_METADATA_DONE) |
		(1 << IMGU_ABI_EVTTYPE_LACE_STATS_DONE) |
		(1 << IMGU_ABI_EVTTYPE_ACC_STAGE_COMPLETE))
		<< IMGU_ABI_SP_COMM_EVENT_IRQ_MASK_OR_SHIFT;

	void __iomem *const base = css->base;
	struct imgu_fw_info *bi, *bl = &css->fwp->binary_header[css->fw_bl];
	unsigned int i;

	writel(IMGU_TLB_INVALIDATE, base + IMGU_REG_TLB_INVALIDATE);

	/* Start bootloader */

	writel(IMGU_ABI_BL_SWSTATE_BUSY,
		base + IMGU_REG_ISP_DMEM_BASE + bl->info.bl.sw_state);
	writel(IMGU_NUM_SP,
		base + IMGU_REG_ISP_DMEM_BASE + bl->info.bl.num_dma_cmds);

	for (i = 0; i < IMGU_NUM_SP; i++) {
		int j = IMGU_NUM_SP - i - 1;	/* load sp1 first, then sp0 */
		struct imgu_fw_info *sp =
			&css->fwp->binary_header[css->fw_sp[j]];
		struct imgu_abi_bl_dma_cmd_entry dma_cmd = {
			.src_addr = css->binary[css->fw_sp[j]].daddr
				+ sp->blob.text_source,
			.size = sp->blob.text_size,
			.dst_type = IMGU_ABI_BL_DMACMD_TYPE_SP_PMEM,
			.dst_addr = IMGU_SP_PMEM_BASE(j),
		};

		writes(&dma_cmd, sizeof(dma_cmd),
			base + IMGU_REG_ISP_DMEM_BASE + i * sizeof(dma_cmd) +
			bl->info.bl.dma_cmd_list);
	}

	writel(bl->info.bl.bl_entry, base + IMGU_REG_ISP_START_ADDR);

	writel(readl(base + IMGU_REG_ISP_CTRL)
		| IMGU_CTRL_START | IMGU_CTRL_RUN, base + IMGU_REG_ISP_CTRL);
	if (ipu3_hw_wait(css->base, IMGU_REG_ISP_DMEM_BASE
			 + bl->info.bl.sw_state, ~0,
			 IMGU_ABI_BL_SWSTATE_OK)) {
		dev_err(css->dev, "failed to start bootloader\n");
		return -EIO;
	}

	/* Start ISP */

	memset(css->xmem_sp_group_ptrs.vaddr, 0,
		sizeof(struct imgu_abi_sp_group));

	bi = &css->fwp->binary_header[css->fw_sp[0]];

	writel(css->xmem_sp_group_ptrs.daddr,
		base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.per_frame_data);

	writel(IMGU_ABI_SP_SWSTATE_TERMINATED,
		base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.sw_state);
	writel(1, base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.invalidate_tlb);

	if (ipu3_css_hw_start_sp(css, 0))
		return -EIO;

	writel(0, base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.isp_started);
	writel(0, base + IMGU_REG_SP_DMEM_BASE(0) +
		bi->info.sp.host_sp_queues_initialized);
	writel(0, base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.sleep_mode);
	writel(0, base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.invalidate_tlb);
	writel(IMGU_ABI_SP_COMM_COMMAND_READY, base + IMGU_REG_SP_DMEM_BASE(0)
		+ bi->info.sp.host_sp_com + IMGU_ABI_SP_COMM_COMMAND);

	/* Enable all events for all queues */

	for (i = 0; i < IPU3_CSS_PIPE_ID_NUM; i++)
		writel(event_mask, base + IMGU_REG_SP_DMEM_BASE(0)
			+ bi->info.sp.host_sp_com
			+ IMGU_ABI_SP_COMM_EVENT_IRQ_MASK(i));
	writel(1, base + IMGU_REG_SP_DMEM_BASE(0) +
		bi->info.sp.host_sp_queues_initialized);

	/* Start SP1 */

	bi = &css->fwp->binary_header[css->fw_sp[1]];

	writel(IMGU_ABI_SP_SWSTATE_TERMINATED,
		base + IMGU_REG_SP_DMEM_BASE(1) + bi->info.sp.sw_state);

	if (ipu3_css_hw_start_sp(css, 1))
		return -EIO;

	writel(IMGU_ABI_SP_COMM_COMMAND_READY, base + IMGU_REG_SP_DMEM_BASE(1)
		+ bi->info.sp.host_sp_com + IMGU_ABI_SP_COMM_COMMAND);

	return 0;
}

static void ipu3_css_hw_stop(struct ipu3_css *css)
{
	void __iomem *const base = css->base;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[0]];

	/* Stop fw */
	writel(IMGU_ABI_SP_COMM_COMMAND_TERMINATE,
		base + IMGU_REG_SP_DMEM_BASE(0) +
		bi->info.sp.host_sp_com + IMGU_ABI_SP_COMM_COMMAND);
	if (ipu3_hw_wait(css->base, IMGU_REG_SP_CTRL(0),
		IMGU_CTRL_IDLE, IMGU_CTRL_IDLE))
		dev_err(css->dev, "wait sp0 idle timeout.\n");
	if (readl(base + IMGU_REG_SP_DMEM_BASE(0) + bi->info.sp.sw_state) !=
		IMGU_ABI_SP_SWSTATE_TERMINATED)
		dev_err(css->dev, "sp0 is not terminated.\n");
	if (ipu3_hw_wait(css->base, IMGU_REG_ISP_CTRL,
		IMGU_CTRL_IDLE, IMGU_CTRL_IDLE))
		dev_err(css->dev, "wait isp idle timeout\n");
}

static void ipu3_css_hw_cleanup(struct ipu3_css *css)
{
	void __iomem *const base = css->base;

	/** Reset CSS **/

	/* Clear the CSS busy signal */
	readl(base + IMGU_REG_GP_BUSY);
	writel(0, base + IMGU_REG_GP_BUSY);

	/* Wait for idle signal */
	if (ipu3_hw_wait(css->base, IMGU_REG_STATE, IMGU_STATE_IDLE_STS,
			 IMGU_STATE_IDLE_STS))
		dev_err(css->dev, "failed to shut down hw cleanly\n");

	/* Reset the css */
	writel(readl(base + IMGU_REG_PM_CTRL) | IMGU_PM_CTRL_FORCE_RESET,
		base + IMGU_REG_PM_CTRL);

	usleep_range(200, 300);
}

static void ipu3_css_pipeline_cleanup(struct ipu3_css *css)
{
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->current_binary];
	int pipe = 0;
	int i;

	ipu3_css_pool_cleanup(css->dma_dev, &css->pool.parameter_set_info);
	ipu3_css_pool_cleanup(css->dma_dev, &css->pool.acc);
	ipu3_css_pool_cleanup(css->dma_dev, &css->pool.gdc);
	ipu3_css_pool_cleanup(css->dma_dev, &css->pool.obgrid);
	for (i = 0; i < IMGU_ABI_NUM_MEMORIES; i++)
		ipu3_css_pool_cleanup(css->dma_dev,
				      &css->pool.binary_params_p[i]);

	for (i = 0; i < bi->info.isp.sp.iterator.num_stripes; i++)
		ipu3_css_dma_free(css->dma_dev, &css->dvs_meta_data[pipe][i]);
}

/*
 * This function initializes various stages of the
 * IPU3 CSS ISP pipeline
 */
static int ipu3_css_pipeline_init(struct ipu3_css *css)
{
	static const unsigned int PIPE_ID = IPU3_CSS_PIPE_ID_VIDEO;
	static const int BYPC = 2;	/* Bytes per component */
	static const struct imgu_abi_buffer_sp buffer_sp_init = {
		.buf_src = {.queue_id = IMGU_ABI_QUEUE_EVENT_ID},
		.buf_type = IMGU_ABI_BUFFER_TYPE_INVALID,
	};

	struct imgu_abi_isp_iterator_config *cfg_iter;
	struct imgu_abi_isp_ref_config *cfg_ref;
	struct imgu_abi_isp_dvs_config *cfg_dvs;
	struct imgu_abi_isp_tnr3_config *cfg_tnr;
	struct imgu_abi_isp_ref_dmem_state *cfg_ref_state;
	struct imgu_abi_isp_tnr3_dmem_state *cfg_tnr_state;

	const int pipe = 0, stage = 0, thread = 0;
	int i;

	const struct imgu_fw_info *bi =
	    &css->fwp->binary_header[css->current_binary];
	const unsigned int stripes = bi->info.isp.sp.iterator.num_stripes;

	struct imgu_fw_config_memory_offsets *cofs = (void *)css->fwp +
	    bi->blob.memory_offsets.offsets[IMGU_ABI_PARAM_CLASS_CONFIG];
	struct imgu_fw_state_memory_offsets *sofs = (void *)css->fwp +
	    bi->blob.memory_offsets.offsets[IMGU_ABI_PARAM_CLASS_STATE];

	struct imgu_abi_isp_stage *isp_stage;
	struct imgu_abi_sp_stage *sp_stage;
	struct imgu_abi_sp_group *sp_group;

	const unsigned int bds_width_pad =
		ALIGN(css->rect[IPU3_CSS_RECT_BDS].width,
		2 * IPU3_UAPI_ISP_VEC_ELEMS);

	enum imgu_abi_param_class c = IMGU_ABI_PARAM_CLASS_CONFIG;
	enum imgu_abi_memories m = IMGU_ABI_MEM_ISP_DMEM0;

	/* Configure iterator */

	cfg_iter = ipu3_css_fw_pipeline_params(
				css, c, m, &cofs->dmem.iterator,
				sizeof(*cfg_iter),
				css->binary_params_cs[c - 1][m].vaddr);
	if (!cfg_iter)
		goto bad_firmware;

	cfg_iter->input_info.res.width =
	    css->queue[IPU3_CSS_QUEUE_IN].fmt.mpix.width;
	cfg_iter->input_info.res.height =
	    css->queue[IPU3_CSS_QUEUE_IN].fmt.mpix.height;
	cfg_iter->input_info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_IN].width_pad;
	cfg_iter->input_info.format =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->frame_format;
	cfg_iter->input_info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->bit_depth;
	cfg_iter->input_info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->bayer_order;
	cfg_iter->input_info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;

	cfg_iter->internal_info.res.width =
			css->rect[IPU3_CSS_RECT_BDS].width;
	cfg_iter->internal_info.res.height =
			css->rect[IPU3_CSS_RECT_BDS].height;
	cfg_iter->internal_info.padded_width = bds_width_pad;
	cfg_iter->internal_info.format =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->frame_format;
	cfg_iter->internal_info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bit_depth;
	cfg_iter->internal_info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bayer_order;
	cfg_iter->internal_info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;

	cfg_iter->output_info.res.width =
	    css->queue[IPU3_CSS_QUEUE_OUT].fmt.mpix.width;
	cfg_iter->output_info.res.height =
	    css->queue[IPU3_CSS_QUEUE_OUT].fmt.mpix.height;
	cfg_iter->output_info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_OUT].width_pad;
	cfg_iter->output_info.format =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->frame_format;
	cfg_iter->output_info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bit_depth;
	cfg_iter->output_info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bayer_order;
	cfg_iter->output_info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;

	cfg_iter->vf_info.res.width =
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.width;
	cfg_iter->vf_info.res.height =
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.height;
	cfg_iter->vf_info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_VF].width_pad;
	cfg_iter->vf_info.format =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->frame_format;
	cfg_iter->vf_info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->bit_depth;
	cfg_iter->vf_info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->bayer_order;
	cfg_iter->vf_info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;

	cfg_iter->dvs_envelope.width = css->rect[IPU3_CSS_RECT_ENVELOPE].width;
	cfg_iter->dvs_envelope.height =
		css->rect[IPU3_CSS_RECT_ENVELOPE].height;

	/* Configure reference (delay) frames */

	cfg_ref = ipu3_css_fw_pipeline_params(css, c, m, &cofs->dmem.ref,
					   sizeof(*cfg_ref),
					   css->binary_params_cs[c -
								 1][m].vaddr);
	if (!cfg_ref)
		goto bad_firmware;

	cfg_ref->port_b.crop = 0;
	cfg_ref->port_b.elems = IMGU_ABI_ISP_DDR_WORD_BYTES / BYPC;
	cfg_ref->port_b.width = css->aux_frames[IPU3_CSS_AUX_FRAME_REF].width;
	cfg_ref->port_b.stride =
	    css->aux_frames[IPU3_CSS_AUX_FRAME_REF].bytesperline;
	cfg_ref->width_a_over_b =
	    IPU3_UAPI_ISP_VEC_ELEMS / cfg_ref->port_b.elems;
	cfg_ref->dvs_frame_delay = IPU3_CSS_AUX_FRAMES - 1;
	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++) {
		cfg_ref->ref_frame_addr_y[i] =
		    css->aux_frames[IPU3_CSS_AUX_FRAME_REF].mem[i].daddr;
		cfg_ref->ref_frame_addr_c[i] =
		    css->aux_frames[IPU3_CSS_AUX_FRAME_REF].mem[i].daddr +
		    css->aux_frames[IPU3_CSS_AUX_FRAME_REF].bytesperline *
		    css->aux_frames[IPU3_CSS_AUX_FRAME_REF].height;
	}
	for (; i < IMGU_ABI_FRAMES_REF; i++) {
		cfg_ref->ref_frame_addr_y[i] = 0;
		cfg_ref->ref_frame_addr_c[i] = 0;
	}

	/* Configure DVS (digital video stabilization) */

	cfg_dvs = ipu3_css_fw_pipeline_params(css, c, m,
				&cofs->dmem.dvs, sizeof(*cfg_dvs),
				css->binary_params_cs[c - 1][m].vaddr);
	if (!cfg_dvs)
		goto bad_firmware;

	cfg_dvs->num_horizontal_blocks =
	    ALIGN(DIV_ROUND_UP(css->rect[IPU3_CSS_RECT_GDC].width,
			       IMGU_DVS_BLOCK_W), 2);
	cfg_dvs->num_vertical_blocks =
	    DIV_ROUND_UP(css->rect[IPU3_CSS_RECT_GDC].height,
			     IMGU_DVS_BLOCK_H);

	if (cfg_dvs->num_horizontal_blocks * cfg_dvs->num_vertical_blocks < 0)
		return -EPROTO;

	/* Configure TNR (temporal noise reduction) */

	if (css->pipe_id == IPU3_CSS_PIPE_ID_VIDEO) {
		cfg_tnr = ipu3_css_fw_pipeline_params(css, c, m,
			&cofs->dmem.tnr3, sizeof(*cfg_tnr),
			css->binary_params_cs[c - 1][m].vaddr);
		if (!cfg_tnr)
			goto bad_firmware;

		cfg_tnr->port_b.crop = 0;
		cfg_tnr->port_b.elems = IMGU_ABI_ISP_DDR_WORD_BYTES;
		cfg_tnr->port_b.width =
		    css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].width;
		cfg_tnr->port_b.stride =
		    css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].bytesperline;
		cfg_tnr->width_a_over_b =
		    IPU3_UAPI_ISP_VEC_ELEMS / cfg_tnr->port_b.elems;
		cfg_tnr->frame_height =
		    css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].height;
		cfg_tnr->delay_frame = IPU3_CSS_AUX_FRAMES - 1;
		for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
			cfg_tnr->frame_addr[i] =
			    css->aux_frames[IPU3_CSS_AUX_FRAME_TNR]
			    .mem[i].daddr;
		for (; i < IMGU_ABI_FRAMES_TNR; i++)
			cfg_tnr->frame_addr[i] = 0;
	}

	/* Configure ref dmem state parameters */

	c = IMGU_ABI_PARAM_CLASS_STATE;

	cfg_ref_state = ipu3_css_fw_pipeline_params(css, c, m,
				&sofs->dmem.ref, sizeof(*cfg_ref_state),
				css->binary_params_cs[c - 1][m].vaddr);
	if (!cfg_ref_state)
		goto bad_firmware;

	cfg_ref_state->ref_in_buf_idx = 0;
	cfg_ref_state->ref_out_buf_idx = 1;

	/* Configure tnr dmem state parameters */
	if (css->pipe_id == IPU3_CSS_PIPE_ID_VIDEO) {
		cfg_tnr_state = ipu3_css_fw_pipeline_params(css, c, m,
			&sofs->dmem.tnr3, sizeof(*cfg_tnr_state),
			css->binary_params_cs[c - 1][m].vaddr);
		if (!cfg_tnr_state)
			goto bad_firmware;

		cfg_tnr_state->in_bufidx = 0;
		cfg_tnr_state->out_bufidx = 1;
		cfg_tnr_state->bypass_filter = 0;
		cfg_tnr_state->total_frame_counter = 0;
		for (i = 0; i < IMGU_ABI_BUF_SETS_TNR; i++)
			cfg_tnr_state->buffer_frame_counter[i] = 0;
	}

	/* Configure ISP stage */

	isp_stage = css->xmem_isp_stage_ptrs[pipe][stage].vaddr;
	memset(isp_stage, 0, sizeof(*isp_stage));
	isp_stage->blob_info = bi->blob;
	isp_stage->binary_info = bi->info.isp.sp;
	strcpy(isp_stage->binary_name,
	       (char *)css->fwp + bi->blob.prog_name_offset);
	isp_stage->mem_initializers = bi->info.isp.sp.mem_initializers;
	for (c = IMGU_ABI_PARAM_CLASS_CONFIG; c < IMGU_ABI_PARAM_CLASS_NUM; c++)
		for (m = 0; m < IMGU_ABI_NUM_MEMORIES; m++)
			isp_stage->mem_initializers.params[c][m].address =
			    css->binary_params_cs[c - 1][m].daddr;

	/* Configure SP stage */

	sp_stage = css->xmem_sp_stage_ptrs[pipe][stage].vaddr;
	memset(sp_stage, 0, sizeof(*sp_stage));

	sp_stage->frames.in.buf_attr = buffer_sp_init;
	for (i = 0; i < IMGU_ABI_BINARY_MAX_OUTPUT_PORTS; i++)
		sp_stage->frames.out[i].buf_attr = buffer_sp_init;
	sp_stage->frames.out_vf.buf_attr = buffer_sp_init;
	sp_stage->frames.s3a_buf = buffer_sp_init;
	sp_stage->frames.dvs_buf = buffer_sp_init;
	sp_stage->frames.lace_buf = buffer_sp_init;

	sp_stage->stage_type = IMGU_ABI_STAGE_TYPE_ISP;
	sp_stage->num = stage;
	sp_stage->isp_online = 0;
	sp_stage->isp_copy_vf = 0;
	sp_stage->isp_copy_output = 0;

	/* Enable VF output only when VF or PV queue requested by user */

	sp_stage->enable.vf_output =
		(css->vf_output_en != IPU3_NODE_VF_DISABLED);

	sp_stage->frames.effective_in_res.width =
		css->rect[IPU3_CSS_RECT_EFFECTIVE].width;
	sp_stage->frames.effective_in_res.height =
		css->rect[IPU3_CSS_RECT_EFFECTIVE].height;
	sp_stage->frames.in.info.res.width =
	    css->queue[IPU3_CSS_QUEUE_IN].fmt.mpix.width;
	sp_stage->frames.in.info.res.height =
	    css->queue[IPU3_CSS_QUEUE_IN].fmt.mpix.height;
	sp_stage->frames.in.info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_IN].width_pad;
	sp_stage->frames.in.info.format =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->frame_format;
	sp_stage->frames.in.info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->bit_depth;
	sp_stage->frames.in.info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_IN].css_fmt->bayer_order;
	sp_stage->frames.in.info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;
	sp_stage->frames.in.buf_attr.buf_src.queue_id = IMGU_ABI_QUEUE_C_ID;
	sp_stage->frames.in.buf_attr.buf_type =
	    IMGU_ABI_BUFFER_TYPE_INPUT_FRAME;

	sp_stage->frames.out[0].info.res.width =
	    css->queue[IPU3_CSS_QUEUE_OUT].fmt.mpix.width;
	sp_stage->frames.out[0].info.res.height =
	    css->queue[IPU3_CSS_QUEUE_OUT].fmt.mpix.height;
	sp_stage->frames.out[0].info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_OUT].width_pad;
	sp_stage->frames.out[0].info.format =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->frame_format;
	sp_stage->frames.out[0].info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bit_depth;
	sp_stage->frames.out[0].info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bayer_order;
	sp_stage->frames.out[0].info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;
	sp_stage->frames.out[0].planes.nv.uv.offset =
	    css->queue[IPU3_CSS_QUEUE_OUT].width_pad *
	    css->queue[IPU3_CSS_QUEUE_OUT].fmt.mpix.height;
	sp_stage->frames.out[0].buf_attr.buf_src.queue_id = IMGU_ABI_QUEUE_D_ID;
	sp_stage->frames.out[0].buf_attr.buf_type =
	    IMGU_ABI_BUFFER_TYPE_OUTPUT_FRAME;

	sp_stage->frames.out[1].buf_attr.buf_src.queue_id =
	    IMGU_ABI_QUEUE_EVENT_ID;

	sp_stage->frames.internal_frame_info.res.width =
					css->rect[IPU3_CSS_RECT_BDS].width;
	sp_stage->frames.internal_frame_info.res.height =
					css->rect[IPU3_CSS_RECT_BDS].height;
	sp_stage->frames.internal_frame_info.padded_width = bds_width_pad;

	sp_stage->frames.internal_frame_info.format =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->frame_format;
	sp_stage->frames.internal_frame_info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bit_depth;
	sp_stage->frames.internal_frame_info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_OUT].css_fmt->bayer_order;
	sp_stage->frames.internal_frame_info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;

	sp_stage->frames.out_vf.info.res.width =
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.width;
	sp_stage->frames.out_vf.info.res.height =
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.height;
	sp_stage->frames.out_vf.info.padded_width =
	    css->queue[IPU3_CSS_QUEUE_VF].width_pad;
	sp_stage->frames.out_vf.info.format =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->frame_format;
	sp_stage->frames.out_vf.info.raw_bit_depth =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->bit_depth;
	sp_stage->frames.out_vf.info.raw_bayer_order =
	    css->queue[IPU3_CSS_QUEUE_VF].css_fmt->bayer_order;
	sp_stage->frames.out_vf.info.raw_type = IMGU_ABI_RAW_TYPE_BAYER;
	sp_stage->frames.out_vf.planes.yuv.u.offset =
	    css->queue[IPU3_CSS_QUEUE_VF].width_pad *
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.height;
	sp_stage->frames.out_vf.planes.yuv.v.offset =
	    css->queue[IPU3_CSS_QUEUE_VF].width_pad *
	    css->queue[IPU3_CSS_QUEUE_VF].fmt.mpix.height * 5 / 4;
	sp_stage->frames.out_vf.buf_attr.buf_src.queue_id = IMGU_ABI_QUEUE_E_ID;
	sp_stage->frames.out_vf.buf_attr.buf_type =
	    IMGU_ABI_BUFFER_TYPE_VF_OUTPUT_FRAME;

	sp_stage->frames.s3a_buf.buf_src.queue_id = IMGU_ABI_QUEUE_F_ID;
	sp_stage->frames.s3a_buf.buf_type = IMGU_ABI_BUFFER_TYPE_3A_STATISTICS;

	sp_stage->frames.dvs_buf.buf_src.queue_id = IMGU_ABI_QUEUE_G_ID;
	sp_stage->frames.dvs_buf.buf_type = IMGU_ABI_BUFFER_TYPE_DIS_STATISTICS;

	sp_stage->dvs_envelope.width = css->rect[IPU3_CSS_RECT_ENVELOPE].width;
	sp_stage->dvs_envelope.height =
		css->rect[IPU3_CSS_RECT_ENVELOPE].height;

	sp_stage->isp_pipe_version =
				bi->info.isp.sp.pipeline.isp_pipe_version;
	sp_stage->isp_deci_log_factor = clamp(
		max(fls(css->rect[IPU3_CSS_RECT_BDS].width /
					IMGU_MAX_BQ_GRID_WIDTH),
		    fls(css->rect[IPU3_CSS_RECT_BDS].height /
					IMGU_MAX_BQ_GRID_HEIGHT)) - 1, 3, 5);
	sp_stage->isp_vf_downscale_bits = 0;
	sp_stage->if_config_index = 255;
	sp_stage->sp_enable_xnr = 0;
	sp_stage->num_stripes = stripes;
	sp_stage->enable.s3a = 1;
	sp_stage->enable.dvs_stats = 1;

	sp_stage->xmem_bin_addr = css->binary[css->current_binary].daddr;
	sp_stage->xmem_map_addr = css->sp_ddr_ptrs.daddr;
	sp_stage->isp_stage_addr = css->xmem_isp_stage_ptrs[pipe][stage].daddr;

	/* Configure SP group */

	sp_group = css->xmem_sp_group_ptrs.vaddr;
	memset(sp_group, 0, sizeof(*sp_group));

	sp_group->pipe[thread].num_stages = 1;
	sp_group->pipe[thread].pipe_id = PIPE_ID;
	sp_group->pipe[thread].thread_id = thread;
	sp_group->pipe[thread].pipe_num = pipe;
	sp_group->pipe[thread].num_execs = -1;
	sp_group->pipe[thread].pipe_qos_config = -1;
	sp_group->pipe[thread].required_bds_factor = 0;
	sp_group->pipe[thread].dvs_frame_delay = IPU3_CSS_AUX_FRAMES - 1;
	sp_group->pipe[thread].inout_port_config =
	    IMGU_ABI_PORT_CONFIG_TYPE_INPUT_HOST |
	    IMGU_ABI_PORT_CONFIG_TYPE_OUTPUT_HOST;
	sp_group->pipe[thread].scaler_pp_lut = 0;
	sp_group->pipe[thread].shading.internal_frame_origin_x_bqs_on_sctbl = 0;
	sp_group->pipe[thread].shading.internal_frame_origin_y_bqs_on_sctbl = 0;
	sp_group->pipe[thread].sp_stage_addr[stage] =
	    css->xmem_sp_stage_ptrs[pipe][stage].daddr;
	sp_group->pipe[thread].pipe_config =
	    bi->info.isp.sp.enable.params ? (1 << thread) : 0;
	sp_group->pipe[thread].pipe_config |= IMGU_ABI_PIPE_CONFIG_ACQUIRE_ISP;

	/* Allocate dvs statistics metadata */

	for (i = 0; i < stripes; i++)
		if (ipu3_css_dma_alloc(css->dma_dev, &css->dvs_meta_data[pipe][i],
				       sizeof(struct imgu_abi_dvs_meta_data)))
			goto out_of_memory;

	/* Initialize parameter pools */

	if (ipu3_css_pool_init(css->dma_dev, &css->pool.parameter_set_info,
			       sizeof(struct imgu_abi_parameter_set_info)) ||
	    ipu3_css_pool_init(css->dma_dev, &css->pool.acc,
			       sizeof(struct ipu3_uapi_acc_param)) ||
	    ipu3_css_pool_init(css->dma_dev, &css->pool.gdc,
				sizeof(struct ipu3_uapi_gdc_warp_param) *
				3 * cfg_dvs->num_horizontal_blocks / 2 *
				cfg_dvs->num_vertical_blocks) ||
	    ipu3_css_pool_init(css->dma_dev, &css->pool.obgrid,
				ipu3_css_fw_obgrid_size(
				&css->fwp->binary_header[css->current_binary])))
		goto out_of_memory;

	for (m = 0; m < IMGU_ABI_NUM_MEMORIES; m++)
		if (ipu3_css_pool_init(css->dma_dev, &css->pool.binary_params_p[m],
				       bi->info.isp.sp.mem_initializers.params
				       [IMGU_ABI_PARAM_CLASS_PARAM][m].size))
			goto out_of_memory;

	return 0;

bad_firmware:
	ipu3_css_pipeline_cleanup(css);
	return -EPROTO;

out_of_memory:
	ipu3_css_pipeline_cleanup(css);
	return -ENOMEM;
}

static u8 ipu3_css_queue_pos(struct ipu3_css *css, int queue, int thread)
{
	static const unsigned int sp;
	void __iomem *const base = css->base;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[sp]];
	struct imgu_abi_queues __iomem *q = base + IMGU_REG_SP_DMEM_BASE(sp) +
	    bi->info.sp.host_sp_queue;

	return queue >= 0 ? readb(&q->host2sp_bufq_info[thread][queue].end) :
	    readb(&q->host2sp_evtq_info.end);
}

/* Sent data to sp using given buffer queue, or if queue < 0, event queue. */
static int ipu3_css_queue_data(struct ipu3_css *css,
			       int queue, int thread, u32 data)
{
	static const unsigned int sp;
	void __iomem *const base = css->base;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[sp]];
	struct imgu_abi_queues __iomem *q = base + IMGU_REG_SP_DMEM_BASE(sp) +
	    bi->info.sp.host_sp_queue;
	u8 size, start, end, end2;

	if (queue >= 0) {
		size = readb(&q->host2sp_bufq_info[thread][queue].size);
		start = readb(&q->host2sp_bufq_info[thread][queue].start);
		end = readb(&q->host2sp_bufq_info[thread][queue].end);
	} else {
		size = readb(&q->host2sp_evtq_info.size);
		start = readb(&q->host2sp_evtq_info.start);
		end = readb(&q->host2sp_evtq_info.end);
	}

	if (size == 0)
		return -EIO;

	end2 = (end + 1) % size;
	if (end2 == start)
		return -EBUSY;	/* Queue full */

	if (queue >= 0) {
		writel(data, &q->host2sp_bufq[thread][queue][end]);
		writeb(end2, &q->host2sp_bufq_info[thread][queue].end);
	} else {
		writel(data, &q->host2sp_evtq[end]);
		writeb(end2, &q->host2sp_evtq_info.end);
	}

	return 0;
}

/* Receive data using given buffer queue, or if queue < 0, event queue. */
static int ipu3_css_dequeue_data(struct ipu3_css *css, int queue, u32 *data)
{
	static const unsigned int sp;
	void __iomem *const base = css->base;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[sp]];
	struct imgu_abi_queues __iomem *q = base + IMGU_REG_SP_DMEM_BASE(sp) +
	    bi->info.sp.host_sp_queue;
	u8 size, start, end, start2;

	if (queue >= 0) {
		size = readb(&q->sp2host_bufq_info[queue].size);
		start = readb(&q->sp2host_bufq_info[queue].start);
		end = readb(&q->sp2host_bufq_info[queue].end);
	} else {
		size = readb(&q->sp2host_evtq_info.size);
		start = readb(&q->sp2host_evtq_info.start);
		end = readb(&q->sp2host_evtq_info.end);
	}

	if (size == 0)
		return -EIO;

	if (end == start)
		return -EBUSY;	/* Queue empty */

	start2 = (start + 1) % size;

	if (queue >= 0) {
		*data = readl(&q->sp2host_bufq[queue][start]);
		writeb(start2, &q->sp2host_bufq_info[queue].start);
	} else {
		int r;

		*data = readl(&q->sp2host_evtq[start]);
		writeb(start2, &q->sp2host_evtq_info.start);

		/* Acknowledge events dequeued from event queue */
		r = ipu3_css_queue_data(css, queue, 0,
					IMGU_ABI_EVENT_EVENT_DEQUEUED);
		if (r < 0)
			return r;
	}

	return 0;
}

/* Free binary-specific resources */
static void ipu3_css_binary_cleanup(struct ipu3_css *css)
{
	int i, j;

	for (j = 0; j < IMGU_ABI_PARAM_CLASS_NUM - 1; j++)
		for (i = 0; i < IMGU_ABI_NUM_MEMORIES; i++)
			ipu3_css_dma_free(css->dma_dev,
					  &css->binary_params_cs[j][i]);

	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		ipu3_css_dma_free(
			css->dma_dev,
			&css->aux_frames[IPU3_CSS_AUX_FRAME_REF].mem[i]);

	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		ipu3_css_dma_free(
			css->dma_dev,
			&css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].mem[i]);
}

static int ipu3_css_binary_preallocate(struct ipu3_css *css)
{
	int i, j;

	for (j = IMGU_ABI_PARAM_CLASS_CONFIG;
	     j < IMGU_ABI_PARAM_CLASS_NUM; j++)
		for (i = 0; i < IMGU_ABI_NUM_MEMORIES; i++) {
			if (ipu3_css_dma_alloc(
				css->dma_dev,
				&css->binary_params_cs[j - 1][i],
				CSS_ABI_SIZE))
				goto out_of_memory;
	}

	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		if (ipu3_css_dma_alloc(
			css->dma_dev,
			&css->aux_frames[IPU3_CSS_AUX_FRAME_REF].mem[i],
			CSS_BDS_SIZE))
			goto out_of_memory;

	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		if (ipu3_css_dma_alloc(
			css->dma_dev,
			&css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].mem[i],
			CSS_GDC_SIZE))
			goto out_of_memory;

	return 0;

out_of_memory:
	ipu3_css_binary_cleanup(css);
	return -ENOMEM;
}

/* allocate binary-specific resources */
static int ipu3_css_binary_setup(struct ipu3_css *css)
{
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->current_binary];
	int i, j, size;
	static const int BYPC = 2;	/* Bytes per component */
	unsigned int w, h;

	/* Allocate parameter memory blocks for this binary */
	for (j = IMGU_ABI_PARAM_CLASS_CONFIG; j < IMGU_ABI_PARAM_CLASS_NUM; j++)
		for (i = 0; i < IMGU_ABI_NUM_MEMORIES; i++)
			if (ipu3_css_dma_buffer_resize(
			    css->dma_dev,
			    &css->binary_params_cs[j - 1][i],
			    bi->info.isp.sp.mem_initializers.params[j][i].size))
				goto out_of_memory;

	/* Allocate internal frame buffers */

	/* Reference frames for DVS, FRAME_FORMAT_YUV420_16 */
	css->aux_frames[IPU3_CSS_AUX_FRAME_REF].bytesperpixel = BYPC;
	css->aux_frames[IPU3_CSS_AUX_FRAME_REF].width =
					css->rect[IPU3_CSS_RECT_BDS].width;
	h = ALIGN(css->rect[IPU3_CSS_RECT_BDS].height,
		  IMGU_DVS_BLOCK_H) + 2 * IMGU_GDC_BUF_Y;
	css->aux_frames[IPU3_CSS_AUX_FRAME_REF].height = h;
	w = ALIGN(css->rect[IPU3_CSS_RECT_BDS].width,
		  2 * IPU3_UAPI_ISP_VEC_ELEMS) + 2 * IMGU_GDC_BUF_X;
	css->aux_frames[IPU3_CSS_AUX_FRAME_REF].bytesperline =
		css->aux_frames[IPU3_CSS_AUX_FRAME_REF].bytesperpixel * w;
	size = w * h * BYPC + (w / 2) * (h / 2) * BYPC * 2;
	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		if (ipu3_css_dma_buffer_resize(
			    css->dma_dev,
			    &css->aux_frames[IPU3_CSS_AUX_FRAME_REF].mem[i],
			    size))
			goto out_of_memory;

	/* TNR frames for temporal noise reduction, FRAME_FORMAT_YUV_LINE */
	css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].bytesperpixel = 1;
	w = roundup(css->rect[IPU3_CSS_RECT_GDC].width,
		    bi->info.isp.sp.block.block_width *
		    IPU3_UAPI_ISP_VEC_ELEMS);
	css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].width = w;
	h = roundup(css->rect[IPU3_CSS_RECT_GDC].height,
		    bi->info.isp.sp.block.output_block_height);
	css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].height = h;
	css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].bytesperline = w;
	size = w * ALIGN(h * 3 / 2 + 3, 2);	/* +3 for vf_pp prefetch */
	for (i = 0; i < IPU3_CSS_AUX_FRAMES; i++)
		if (ipu3_css_dma_buffer_resize(
			    css->dma_dev,
			    &css->aux_frames[IPU3_CSS_AUX_FRAME_TNR].mem[i],
			    size))
			goto out_of_memory;

	return 0;

out_of_memory:
	ipu3_css_binary_cleanup(css);
	return -ENOMEM;
}

int ipu3_css_start_streaming(struct ipu3_css *css)
{
	u32 data;
	int r;

	if (css->streaming)
		return -EPROTO;

	r = ipu3_css_binary_setup(css);
	if (r < 0)
		return r;

	r = ipu3_css_hw_init(css);
	if (r < 0)
		return r;

	r = ipu3_css_hw_start(css);
	if (r < 0)
		goto fail;

	r = ipu3_css_pipeline_init(css);
	if (r < 0)
		goto fail;

	css->streaming = true;
	css->frame = 0;

	ipu3_css_hw_enable_irq(css);

	/* Initialize parameters to default */
	r = ipu3_css_set_parameters(css, NULL, NULL, 0, NULL, 0);
	if (r < 0)
		goto fail;

	while (!(r = ipu3_css_dequeue_data(css, IMGU_ABI_QUEUE_A_ID, &data)))
		;
	if (r != -EBUSY)
		goto fail;

	while (!(r = ipu3_css_dequeue_data(css, IMGU_ABI_QUEUE_B_ID, &data)))
		;
	if (r != -EBUSY)
		goto fail;

	r = ipu3_css_queue_data(css, IMGU_ABI_QUEUE_EVENT_ID, 0,
				IMGU_ABI_EVENT_START_STREAM);
	if (r < 0)
		goto fail;

	return 0;

fail:
	css->streaming = false;
	ipu3_css_hw_cleanup(css);
	ipu3_css_pipeline_cleanup(css);
	ipu3_css_binary_cleanup(css);

	return r;
}

void ipu3_css_stop_streaming(struct ipu3_css *css)
{
	struct ipu3_css_buffer *b, *b0;
	int q;

	if (!css->streaming)
		return;

	ipu3_css_hw_stop(css);

	ipu3_css_hw_cleanup(css);

	ipu3_css_pipeline_cleanup(css);

	spin_lock(&css->qlock);
	for (q = 0; q < IPU3_CSS_QUEUES; q++)
		list_for_each_entry_safe(b, b0, &css->queue[q].bufs, list) {
			b->state = IPU3_CSS_BUFFER_FAILED;
			list_del(&b->list);
		}
	spin_unlock(&css->qlock);

	css->streaming = false;
}

bool ipu3_css_queue_empty(struct ipu3_css *css)
{
	int q;

	spin_lock(&css->qlock);
	for (q = 0; q < IPU3_CSS_QUEUES; q++)
		if (!list_empty(&css->queue[q].bufs))
			break;
	spin_unlock(&css->qlock);
	return (q == IPU3_CSS_QUEUES);
}

bool ipu3_css_is_streaming(struct ipu3_css *css)
{
	return css->streaming;
}

void ipu3_css_cleanup(struct ipu3_css *css)
{
	int p, q, i;

	ipu3_css_stop_streaming(css);
	ipu3_css_binary_cleanup(css);

	v4l2_ctrl_handler_free(&css->ctrls.handler);

	for (q = 0; q < IPU3_CSS_QUEUES; q++)
		for (i = 0; i < ARRAY_SIZE(css->abi_buffers[q]); i++)
			ipu3_css_dma_free(css->dma_dev, &css->abi_buffers[q][i]);

	for (p = 0; p < IPU3_CSS_PIPE_ID_NUM; p++)
		for (i = 0; i < IMGU_ABI_MAX_STAGES; i++) {
			ipu3_css_dma_free(css->dma_dev,
					  &css->xmem_sp_stage_ptrs[p][i]);
			ipu3_css_dma_free(css->dma_dev,
					  &css->xmem_isp_stage_ptrs[p][i]);
		}

	ipu3_css_dma_free(css->dma_dev, &css->sp_ddr_ptrs);
	ipu3_css_dma_free(css->dma_dev, &css->xmem_sp_group_ptrs);

	ipu3_css_fw_cleanup(css);
}

int ipu3_css_init(struct device *dev, struct ipu3_css *css,
		  void __iomem *base, int length, struct device *dma_dev)
{
	int r, p, q, i;

	/* Initialize main data structure */

	css->dev = dev;
	css->dma_dev = dma_dev;
	css->base = base;
	css->iomem_length = length;
	css->current_binary = IPU3_CSS_DEFAULT_BINARY;
	css->pipe_id = IPU3_CSS_PIPE_ID_NUM;
	spin_lock_init(&css->qlock);

	for (q = 0; q < IPU3_CSS_QUEUES; q++) {
		r = ipu3_css_queue_init(&css->queue[q], NULL, 0);
		if (r)
			return r;
	}

	r = ipu3_css_fw_init(css);
	if (r)
		return r;

	/* Allocate and map common structures with imgu hardware */

	for (p = 0; p < IPU3_CSS_PIPE_ID_NUM; p++)
		for (i = 0; i < IMGU_ABI_MAX_STAGES; i++) {
			if (ipu3_css_dma_alloc(dma_dev,
					&css->xmem_sp_stage_ptrs[p][i],
					sizeof(struct imgu_abi_sp_stage)))
				goto error_no_memory;
			if (ipu3_css_dma_alloc(dma_dev,
					&css->xmem_isp_stage_ptrs[p][i],
					sizeof(struct imgu_abi_isp_stage)))
				goto error_no_memory;
		}

	if (ipu3_css_dma_alloc(dma_dev, &css->sp_ddr_ptrs,
				ALIGN(sizeof(struct imgu_abi_ddr_address_map),
				IMGU_ABI_ISP_DDR_WORD_BYTES)))
		goto error_no_memory;

	if (ipu3_css_dma_alloc(dma_dev, &css->xmem_sp_group_ptrs,
				sizeof(struct imgu_abi_sp_group)))
		goto error_no_memory;

	for (q = 0; q < IPU3_CSS_QUEUES; q++)
		for (i = 0; i < ARRAY_SIZE(css->abi_buffers[q]); i++)
			if (ipu3_css_dma_alloc(dma_dev, &css->abi_buffers[q][i],
						sizeof(struct imgu_abi_buffer)))
				goto error_no_memory;

	if (ipu3_css_binary_preallocate(css))
		goto error_binary_setup;

	return 0;

error_binary_setup:
	ipu3_css_binary_cleanup(css);
error_no_memory:
	ipu3_css_cleanup(css);

	return -ENOMEM;
}

static u32 ipu3_css_adjust(u32 res, u32 align)
{
	if (res < IPU3_CSS_MIN_RES)
		res = IPU3_CSS_MIN_RES;
	res = roundclosest(res, align);

	return res;
}

/* Select a binary matching the required resolutions and formats */
static int ipu3_css_find_binary(struct ipu3_css *css,
				struct ipu3_css_queue queue[IPU3_CSS_QUEUES],
				struct v4l2_rect rects[IPU3_CSS_RECTS])
{
	const int binary_nr = css->fwp->file_header.binary_nr;
	const char *name;

	const struct v4l2_pix_format_mplane *in =
			&queue[IPU3_CSS_QUEUE_IN].fmt.mpix;
	const struct v4l2_pix_format_mplane *out =
			&queue[IPU3_CSS_QUEUE_OUT].fmt.mpix;
	const struct v4l2_pix_format_mplane *vf =
			&queue[IPU3_CSS_QUEUE_VF].fmt.mpix;
	unsigned int binary_mode = (css->pipe_id == IPU3_CSS_PIPE_ID_CAPTURE) ?
		IA_CSS_BINARY_MODE_PRIMARY : IA_CSS_BINARY_MODE_VIDEO;

	int i, j;
	u32 stripe_w = 0;
	u32 stripe_h = 0;

	if (!ipu3_css_queue_enabled(&queue[IPU3_CSS_QUEUE_IN]))
		return -EINVAL;

	/* Find out the strip size boundary */
	for (i = 0; i < binary_nr; i++) {
		struct imgu_fw_info *bi = &css->fwp->binary_header[i];

		u32 max_width = bi->info.isp.sp.output.max_width;
		u32 max_height = bi->info.isp.sp.output.max_height;

		if (bi->info.isp.sp.iterator.num_stripes <= 1) {
			stripe_w = stripe_w ?
				min(stripe_w, max_width) : max_width;
			stripe_h = stripe_h ?
				min(stripe_h, max_height) : max_height;
		}
	}

	for (i = 0; i < binary_nr; i++) {
		struct imgu_fw_info *bi = &css->fwp->binary_header[i];
		enum imgu_abi_frame_format q_fmt;

		name = (void *)css->fwp + bi->blob.prog_name_offset;

		/* Check that binary supports memory-to-memory processing */
		if (bi->info.isp.sp.input.source !=
		    IMGU_ABI_BINARY_INPUT_SOURCE_MEMORY)
			continue;

		/* Check that binary supports raw10 input */
		if (!bi->info.isp.sp.enable.input_feeder &&
		    !bi->info.isp.sp.enable.input_raw)
			continue;

		/* Check binary mode */
		if (bi->info.isp.sp.pipeline.mode != binary_mode)
			continue;

		/* Since input is RGGB bayer, need to process colors */
		if (bi->info.isp.sp.enable.luma_only)
			continue;

		if (in->width < bi->info.isp.sp.input.min_width ||
		    in->width > bi->info.isp.sp.input.max_width ||
		    in->height < bi->info.isp.sp.input.min_height ||
		    in->height > bi->info.isp.sp.input.max_height)
			continue;

		if (ipu3_css_queue_enabled(&queue[IPU3_CSS_QUEUE_OUT])) {
			if (bi->info.isp.num_output_pins <= 0)
				continue;

			q_fmt = queue[IPU3_CSS_QUEUE_OUT].css_fmt->frame_format;

			for (j = 0; j < bi->info.isp.num_output_formats; j++)
				if (bi->info.isp.output_formats[j] == q_fmt)
					break;
			if (j >= bi->info.isp.num_output_formats)
				continue;

			if (out->width < bi->info.isp.sp.output.min_width ||
			    out->width > bi->info.isp.sp.output.max_width ||
			    out->height < bi->info.isp.sp.output.min_height ||
			    out->height > bi->info.isp.sp.output.max_height)
				continue;

			if (out->width > bi->info.isp.sp.internal.max_width ||
			    out->height > bi->info.isp.sp.internal.max_height)
				continue;
		}

		if (ipu3_css_queue_enabled(&queue[IPU3_CSS_QUEUE_VF])) {
			if (bi->info.isp.num_output_pins <= 1)
				continue;
			q_fmt = queue[IPU3_CSS_QUEUE_VF].css_fmt->frame_format;
			for (j = 0; j < bi->info.isp.num_output_formats; j++)
				if (bi->info.isp.output_formats[j] == q_fmt)
					break;
			if (j >= bi->info.isp.num_output_formats)
				continue;

			if (vf->width < bi->info.isp.sp.output.min_width ||
			    vf->width > bi->info.isp.sp.output.max_width ||
			    vf->height < bi->info.isp.sp.output.min_height ||
			    vf->height > bi->info.isp.sp.output.max_height)
				continue;
		}

		/* All checks passed, select the binary */
		dev_dbg(css->dev, "using binary %s\n", name);
		return i;
	}

	/* Can not find suitable binary for these parameters */
	return -EINVAL;
}

/*
 * Check that there is a binary matching requirements. Parameters may be
 * NULL indicating disabled input/output. Return negative if given
 * parameters can not be supported or on error, zero or positive indicating
 * found binary number. May modify the given parameters if not exact match
 * is found.
 */
int ipu3_css_fmt_try(struct ipu3_css *css,
		     struct v4l2_pix_format_mplane *fmts[IPU3_CSS_QUEUES],
		     struct v4l2_rect *rects[IPU3_CSS_RECTS])
{
	static const u32 EFF_ALIGN_W = 2;
	static const u32 BDS_ALIGN_W = 4;
	static const u32 OUT_ALIGN_W = 8;
	static const u32 OUT_ALIGN_H = 4;
	static const u32 VF_ALIGN_W  = 2;
	static const char *qnames[IPU3_CSS_QUEUES] = {
		[IPU3_CSS_QUEUE_IN] = "in",
		[IPU3_CSS_QUEUE_PARAMS]    = "params",
		[IPU3_CSS_QUEUE_OUT] = "out",
		[IPU3_CSS_QUEUE_VF] = "vf",
		[IPU3_CSS_QUEUE_STAT_3A]   = "3a",
		[IPU3_CSS_QUEUE_STAT_DVS]  = "dvs",
		[IPU3_CSS_QUEUE_STAT_LACE] = "lace",
	};
	static const char *rnames[IPU3_CSS_RECTS] = {
		[IPU3_CSS_RECT_EFFECTIVE] = "effective resolution",
		[IPU3_CSS_RECT_BDS]       = "bayer-domain scaled resolution",
		[IPU3_CSS_RECT_ENVELOPE]  = "DVS envelope size",
		[IPU3_CSS_RECT_GDC]  = "GDC output res",
	};
	struct ipu3_css_queue q[IPU3_CSS_QUEUES];
	struct v4l2_rect r[IPU3_CSS_RECTS] = { };
	struct v4l2_pix_format_mplane *const in =
			&q[IPU3_CSS_QUEUE_IN].fmt.mpix;
	struct v4l2_pix_format_mplane *const out =
			&q[IPU3_CSS_QUEUE_OUT].fmt.mpix;
	struct v4l2_pix_format_mplane *const vf =
			&q[IPU3_CSS_QUEUE_VF].fmt.mpix;
	struct v4l2_rect       *const eff = &r[IPU3_CSS_RECT_EFFECTIVE];
	struct v4l2_rect       *const bds = &r[IPU3_CSS_RECT_BDS];
	struct v4l2_rect       *const env = &r[IPU3_CSS_RECT_ENVELOPE];
	struct v4l2_rect       *const gdc = &r[IPU3_CSS_RECT_GDC];
	int binary, i, s;

	/* Decide which pipe to use */
	if (css->vf_output_en == IPU3_NODE_PV_ENABLED)
		css->pipe_id = IPU3_CSS_PIPE_ID_CAPTURE;
	else if (css->vf_output_en == IPU3_NODE_VF_ENABLED)
		css->pipe_id = IPU3_CSS_PIPE_ID_VIDEO;

	/* Adjust all formats, get statistics buffer sizes and formats */
	for (i = 0; i < IPU3_CSS_QUEUES; i++) {
		if (fmts[i])
			dev_dbg(css->dev, "%s %s: (%i,%i) fmt 0x%x\n", __func__,
				qnames[i], fmts[i]->width, fmts[i]->height,
				fmts[i]->pixelformat);
		else
			dev_dbg(css->dev, "%s %s: (not set)\n", __func__,
				qnames[i]);
		if (ipu3_css_queue_init(&q[i], fmts[i],
				IPU3_CSS_QUEUE_TO_FLAGS(i))) {
			dev_notice(css->dev, "can not initialize queue %s\n",
					qnames[i]);
			return -EINVAL;
		}
	}
	for (i = 0; i < IPU3_CSS_RECTS; i++) {
		if (rects[i]) {
			dev_dbg(css->dev, "%s %s: (%i,%i)\n", __func__,
				rnames[i], rects[i]->width, rects[i]->height);
			r[i].width  = rects[i]->width;
			r[i].height = rects[i]->height;
		} else {
			dev_dbg(css->dev, "%s %s: (not set)\n", __func__,
				rnames[i]);
		}
		/* For now, force known good resolutions */
		r[i].left = 0;
		r[i].top  = 0;
	}

	/* Always require one input and vf only if out is also enabled */
	if (!ipu3_css_queue_enabled(&q[IPU3_CSS_QUEUE_IN]) ||
		(ipu3_css_queue_enabled(&q[IPU3_CSS_QUEUE_VF]) &&
		!ipu3_css_queue_enabled(&q[IPU3_CSS_QUEUE_OUT]))) {
		dev_dbg(css->dev, "required queues are disabled\n");
		return -EINVAL;
	}

	if (!ipu3_css_queue_enabled(&q[IPU3_CSS_QUEUE_OUT])) {
		out->width = in->width;
		out->height = in->height;
	}
	if (eff->width <= 0 || eff->height <= 0) {
		eff->width = in->width;
		eff->height = in->height;
	}
	if (bds->width <= 0 || bds->height <= 0) {
		bds->width = out->width;
		bds->height = out->height;
	}
	if (gdc->width <= 0 || gdc->height <= 0) {
		gdc->width = out->width;
		gdc->height = out->height;
	}

	in->width   = ipu3_css_adjust(in->width, 1);
	in->height  = ipu3_css_adjust(in->height, 1);
	eff->width  = ipu3_css_adjust(eff->width, EFF_ALIGN_W);
	eff->height = ipu3_css_adjust(eff->height, 1);
	bds->width  = ipu3_css_adjust(bds->width, BDS_ALIGN_W);
	bds->height = ipu3_css_adjust(bds->height, 1);
	gdc->width  = ipu3_css_adjust(gdc->width, OUT_ALIGN_W);
	gdc->height = ipu3_css_adjust(gdc->height, OUT_ALIGN_H);
	out->width  = ipu3_css_adjust(out->width, OUT_ALIGN_W);
	out->height = ipu3_css_adjust(out->height, OUT_ALIGN_H);
	vf->width   = ipu3_css_adjust(vf->width, VF_ALIGN_W);
	vf->height  = ipu3_css_adjust(vf->height, 1);

	s = (bds->width - gdc->width) / 2 - FILTER_SIZE;
	env->width = s < MIN_ENVELOPE ? MIN_ENVELOPE : s;
	s = (bds->height - gdc->height) / 2 - FILTER_SIZE;
	env->height = s < MIN_ENVELOPE ? MIN_ENVELOPE : s;

	binary = ipu3_css_find_binary(css, q, r);
	if (binary < 0) {
		dev_err(css->dev, "failed to find suitable binary\n");
		return -EINVAL;
	}

	/* Final adjustment and set back the queried formats */
	for (i = 0; i < IPU3_CSS_QUEUES; i++) {
		if (fmts[i]) {
			if (ipu3_css_queue_init(&q[i], &q[i].fmt.mpix,
						IPU3_CSS_QUEUE_TO_FLAGS(i))) {
				dev_err(css->dev,
					"final resolution adjustment failed\n");
				return -EINVAL;
			}
			*fmts[i] = q[i].fmt.mpix;
		}
	}

	for (i = 0; i < IPU3_CSS_RECTS; i++)
		if (rects[i])
			*rects[i] = r[i];

	dev_dbg(css->dev,
		"in(%u,%u) if(%u,%u) ds(%u,%u) gdc(%u,%u) out(%u,%u) vf(%u,%u)",
		 in->width, in->height, eff->width, eff->height,
		 bds->width, bds->height, gdc->width, gdc->height,
		 out->width, out->height, vf->width, vf->height);

	return binary;
}

int ipu3_css_fmt_set(struct ipu3_css *css,
		     struct v4l2_pix_format_mplane *fmts[IPU3_CSS_QUEUES],
		     struct v4l2_rect *rects[IPU3_CSS_RECTS])
{
	struct v4l2_rect rect_data[IPU3_CSS_RECTS];
	struct v4l2_rect *all_rects[IPU3_CSS_RECTS];
	int i, r;

	for (i = 0; i < IPU3_CSS_RECTS; i++) {
		if (rects[i])
			rect_data[i] = *rects[i];
		else
			memset(&rect_data[i], 0, sizeof(rect_data[i]));
		all_rects[i] = &rect_data[i];
	}
	r = ipu3_css_fmt_try(css, fmts, all_rects);
	if (r < 0)
		return r;
	css->current_binary = (unsigned int)r;

	for (i = 0; i < IPU3_CSS_QUEUES; i++)
		if (ipu3_css_queue_init(&css->queue[i], fmts[i],
					IPU3_CSS_QUEUE_TO_FLAGS(i)))
			return -EINVAL;
	for (i = 0; i < IPU3_CSS_RECTS; i++) {
		css->rect[i] = rect_data[i];
		if (rects[i])
			*rects[i] = rect_data[i];
	}

	return 0;
}

int ipu3_css_meta_fmt_set(struct v4l2_meta_format *fmt)
{
	switch (fmt->dataformat) {
	case V4L2_META_FMT_IPU3_PARAMS:
		fmt->buffersize = sizeof(struct ipu3_uapi_params);
		break;
	case V4L2_META_FMT_IPU3_STAT_3A:
		fmt->buffersize = sizeof(struct ipu3_uapi_stats_3a);
		break;
	case V4L2_META_FMT_IPU3_STAT_DVS:
		fmt->buffersize = sizeof(struct ipu3_uapi_stats_dvs);
		break;
	case V4L2_META_FMT_IPU3_STAT_LACE:
		fmt->buffersize = sizeof(struct ipu3_uapi_stats_lace);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Queue given buffer to CSS. ipu3_css_buf_prepare() must have been first
 * called for the buffer. May be called from interrupt context.
 * Returns 0 on success, -EBUSY if the buffer queue is full, or some other
 * code on error conditions.
 */
int ipu3_css_buf_queue(struct ipu3_css *css, struct ipu3_css_buffer *b)
{
	static const int thread;
	struct imgu_abi_buffer *abi_buf;
	struct imgu_addr_t *buf_addr;
	u32 data;
	int r;

	if (!css->streaming)
		return -EPROTO;	/* CSS or buffer in wrong state */

	if (b->queue >= IPU3_CSS_QUEUES || !ipu3_css_queues[b->queue].qid)
		return -EINVAL;

	b->queue_pos = ipu3_css_queue_pos(
				css, ipu3_css_queues[b->queue].qid, thread);

	if (b->queue_pos >= ARRAY_SIZE(css->abi_buffers[b->queue]))
		return -EIO;
	abi_buf = css->abi_buffers[b->queue][b->queue_pos].vaddr;

	/* Fill struct abi_buffer for firmware */
	memset(abi_buf, 0, sizeof(*abi_buf));

	buf_addr = (void *)abi_buf + ipu3_css_queues[b->queue].ptr_ofs;
	*(imgu_addr_t *)buf_addr = b->daddr;

	if (b->queue == IPU3_CSS_QUEUE_STAT_3A)
		abi_buf->payload.s3a.data.dmem.s3a_tbl = b->daddr;

	if (b->queue == IPU3_CSS_QUEUE_OUT)
		abi_buf->payload.frame.padded_width =
			css->queue[IPU3_CSS_QUEUE_OUT].width_pad;

	if (b->queue == IPU3_CSS_QUEUE_VF)
		abi_buf->payload.frame.padded_width =
			css->queue[IPU3_CSS_QUEUE_VF].width_pad;

	spin_lock(&css->qlock);
	list_add_tail(&b->list, &css->queue[b->queue].bufs);
	spin_unlock(&css->qlock);
	b->state = IPU3_CSS_BUFFER_QUEUED;

	data = css->abi_buffers[b->queue][b->queue_pos].daddr;
	r = ipu3_css_queue_data(css, ipu3_css_queues[b->queue].qid,
				thread, data);
	if (r < 0)
		goto queueing_failed;

	data = IMGU_ABI_EVENT_BUFFER_ENQUEUED(thread,
				ipu3_css_queues[b->queue].qid);
	r = ipu3_css_queue_data(css, IMGU_ABI_QUEUE_EVENT_ID,
				0, data);
	if (r < 0)
		goto queueing_failed;

	dev_dbg(css->dev, "queued buffer %p to css queue %i\n", b, b->queue);

	return 0;

queueing_failed:
	b->state = (r == -EBUSY || r == -EAGAIN) ?
		IPU3_CSS_BUFFER_NEW : IPU3_CSS_BUFFER_FAILED;
	list_del(&b->list);

	return r;
}

/*
 * Get next ready CSS buffer. Returns -EAGAIN in which case the function
 * should be called again, or -EBUSY which means that there are no more
 * buffers available. May be called from interrupt context.
 */
struct ipu3_css_buffer *ipu3_css_buf_dequeue(struct ipu3_css *css)
{
	static const int thread;
	static const unsigned char evtype_to_queue[] = {
		[IMGU_ABI_EVTTYPE_INPUT_FRAME_DONE] = IPU3_CSS_QUEUE_IN,
		[IMGU_ABI_EVTTYPE_OUT_FRAME_DONE] = IPU3_CSS_QUEUE_OUT,
		[IMGU_ABI_EVTTYPE_VF_OUT_FRAME_DONE] = IPU3_CSS_QUEUE_VF,
		[IMGU_ABI_EVTTYPE_3A_STATS_DONE] = IPU3_CSS_QUEUE_STAT_3A,
		[IMGU_ABI_EVTTYPE_DIS_STATS_DONE] = IPU3_CSS_QUEUE_STAT_DVS,
		[IMGU_ABI_EVTTYPE_LACE_STATS_DONE] = IPU3_CSS_QUEUE_STAT_LACE,
	};
	struct ipu3_css_buffer *b = ERR_PTR(-EAGAIN);
	u32 event, daddr;
	int evtype, pipe, pipeid, queue, qid, r;

	if (!css->streaming)
		return ERR_PTR(-EPROTO);

	r = ipu3_css_dequeue_data(css, IMGU_ABI_QUEUE_EVENT_ID, &event);
	if (r < 0)
		return ERR_PTR(r);

	evtype = (event & IMGU_ABI_EVTTYPE_EVENT_MASK) >>
	    IMGU_ABI_EVTTYPE_EVENT_SHIFT;

	switch (evtype) {
	case IMGU_ABI_EVTTYPE_OUT_FRAME_DONE:
	case IMGU_ABI_EVTTYPE_VF_OUT_FRAME_DONE:
	case IMGU_ABI_EVTTYPE_3A_STATS_DONE:
	case IMGU_ABI_EVTTYPE_DIS_STATS_DONE:
	case IMGU_ABI_EVTTYPE_INPUT_FRAME_DONE:
	case IMGU_ABI_EVTTYPE_LACE_STATS_DONE:
		pipe = (event & IMGU_ABI_EVTTYPE_PIPE_MASK) >>
		    IMGU_ABI_EVTTYPE_PIPE_SHIFT;
		pipeid = (event & IMGU_ABI_EVTTYPE_PIPEID_MASK) >>
		    IMGU_ABI_EVTTYPE_PIPEID_SHIFT;
		queue = evtype_to_queue[evtype];
		qid = ipu3_css_queues[queue].qid;

		if (qid >= IMGU_ABI_QUEUE_NUM) {
			dev_err(css->dev, "Invalid qid: %i\n", qid);
			return ERR_PTR(-EIO);
		}

		dev_dbg(css->dev,
			 "event: buffer done 0x%x queue %i pipe %i pipeid %i\n",
			 event, queue, pipe, pipeid);

		r = ipu3_css_dequeue_data(css, qid, &daddr);
		if (r < 0) {
			dev_err(css->dev, "failed to dequeue buffer\n");
			/* Force real error, not -EBUSY */
			return ERR_PTR(-EIO);
		}

		r = ipu3_css_queue_data(css, IMGU_ABI_QUEUE_EVENT_ID, thread,
					IMGU_ABI_EVENT_BUFFER_DEQUEUED(qid));
		if (r < 0) {
			dev_err(css->dev, "failed to queue event\n");
			return ERR_PTR(-EIO);
		}

		spin_lock(&css->qlock);
		if (list_empty(&css->queue[queue].bufs)) {
			spin_unlock(&css->qlock);
			dev_err(css->dev, "event on empty queue\n");
			return ERR_PTR(-EIO);
		}
		b = list_first_entry(&css->queue[queue].bufs,
				     struct ipu3_css_buffer, list);
		if (queue != b->queue ||
		    daddr != css->abi_buffers[b->queue][b->queue_pos].daddr) {
			spin_unlock(&css->qlock);
			dev_err(css->dev, "dequeued bad buffer 0x%x\n", daddr);
			return ERR_PTR(-EIO);
		}
		b->state = IPU3_CSS_BUFFER_DONE;
		list_del(&b->list);
		spin_unlock(&css->qlock);
		break;
	case IMGU_ABI_EVTTYPE_PIPELINE_DONE:
		dev_dbg(css->dev, "event: pipeline done 0x%x for frame %ld\n",
			 event, css->frame);
		if (css->frame == LONG_MAX)
			css->frame = 0;
		else
			css->frame++;
		break;
	case IMGU_ABI_EVTTYPE_TIMER:
		r = ipu3_css_dequeue_data(css, IMGU_ABI_QUEUE_EVENT_ID, &event);
		if (r < 0)
			return ERR_PTR(r);

		if ((event & IMGU_ABI_EVTTYPE_EVENT_MASK) >>
		    IMGU_ABI_EVTTYPE_EVENT_SHIFT == IMGU_ABI_EVTTYPE_TIMER)
			dev_dbg(css->dev, "event: timer\n");
		else
			dev_warn(css->dev, "half of timer event missing\n");
		break;
	case IMGU_ABI_EVTTYPE_FW_WARNING:
		dev_warn(css->dev, "event: firmware warning 0x%x\n", event);
		break;
	case IMGU_ABI_EVTTYPE_FW_ASSERT:
		dev_err(css->dev,
			"event: firmware assert 0x%x module_id %i line_no %i\n",
			event,
			(event & IMGU_ABI_EVTTYPE_MODULEID_MASK) >>
			IMGU_ABI_EVTTYPE_MODULEID_SHIFT,
			swab16((event & IMGU_ABI_EVTTYPE_LINENO_MASK) >>
			       IMGU_ABI_EVTTYPE_LINENO_SHIFT));
		break;
	default:
		dev_warn(css->dev, "received unknown event 0x%x\n", event);
	}

	return b;
}

/*
 * Get a new set of parameters from pool and initialize them based on
 * the parameters params, gdc, and obgrid. Any of these may be NULL,
 * in which case the previously set parameters are used.
 * If parameters haven't been set previously, initialize from scratch.
 *
 * Return index to css->parameter_set_info which has the newly created
 * parameters or negative value on error.
 */
int ipu3_css_set_parameters(struct ipu3_css *css,
			    struct ipu3_uapi_params *set_params,
			    struct ipu3_uapi_gdc_warp_param *set_gdc,
			    unsigned int gdc_bytes,
			    struct ipu3_uapi_obgrid_param *set_obgrid,
			    unsigned int obgrid_bytes)
{
	static const unsigned int queue_id = IMGU_ABI_QUEUE_A_ID;
	const int stage = 0, thread = 0;
	const struct imgu_fw_info *bi =
	    &css->fwp->binary_header[css->current_binary];
	const int obgrid_size = ipu3_css_fw_obgrid_size(bi);
	const unsigned int stripes = bi->info.isp.sp.iterator.num_stripes ? : 1;
	struct ipu3_uapi_flags *use = set_params ? &set_params->use : NULL;

	/* Destination buffers which are filled here */
	struct imgu_abi_parameter_set_info *param_set;
	struct ipu3_uapi_acc_param *acc = NULL;
	struct ipu3_uapi_gdc_warp_param *gdc = NULL;
	struct ipu3_uapi_obgrid_param *obgrid = NULL;
	const struct ipu3_css_map *map;
	void *vmem0 = NULL;
	void *dmem0 = NULL;

	enum imgu_abi_memories m;
	int r = -EBUSY;
	int s;

	if (!css->streaming)
		return -EPROTO;

	/*
	 * Check that we can get a new parameter_set_info from the pool.
	 * If this succeeds, then all of the other pool_get() calls below
	 * should also succeed.
	 */
	if (ipu3_css_pool_get(&css->pool.parameter_set_info, css->frame) < 0)
		goto fail_no_put;
	param_set = ipu3_css_pool_last(&css->pool.parameter_set_info, 0)->vaddr;

	/* Get a new acc only if new parameters given, or none yet */
	if (set_params || !ipu3_css_pool_last(&css->pool.acc, 0)->vaddr) {
		if (ipu3_css_pool_get(&css->pool.acc, css->frame) < 0)
			goto fail;
		acc = ipu3_css_pool_last(&css->pool.acc, 0)->vaddr;
	}

	/* Get new VMEM0 only if needed, or none yet */
	m = IMGU_ABI_MEM_ISP_VMEM0;
	if (!ipu3_css_pool_last(&css->pool.binary_params_p[m], 0)->vaddr ||
	    (set_params && (set_params->use.lin_vmem_params ||
			    set_params->use.tnr3_vmem_params ||
			    set_params->use.xnr3_vmem_params))) {
		if (ipu3_css_pool_get(&css->pool.binary_params_p[m],
				      css->frame) < 0)
			goto fail;
		vmem0 = ipu3_css_pool_last(&css->pool.binary_params_p[m], 0)
		    ->vaddr;
	}

	/* Get new DMEM0 only if needed, or none yet */
	m = IMGU_ABI_MEM_ISP_DMEM0;
	if (!ipu3_css_pool_last(&css->pool.binary_params_p[m], 0)->vaddr ||
	    (set_params && (set_params->use.tnr3_dmem_params ||
			    set_params->use.xnr3_dmem_params))) {
		if (ipu3_css_pool_get(&css->pool.binary_params_p[m],
				      css->frame) < 0)
			goto fail;
		dmem0 = ipu3_css_pool_last(&css->pool.binary_params_p[m], 0)
		    ->vaddr;
	}

	/* Configure acc parameter cluster */
	if (acc) {
		map = ipu3_css_pool_last(&css->pool.acc, 1);
		r = ipu3_css_cfg_acc(css, use, acc, map->vaddr,
				set_params ? &set_params->acc_param : NULL);
		if (r < 0)
			goto fail;
	}

	/* Configure late binding parameters */
	if (vmem0) {
		m = IMGU_ABI_MEM_ISP_VMEM0;
		map = ipu3_css_pool_last(&css->pool.binary_params_p[m], 1);
		r = ipu3_css_cfg_vmem0(css, use, vmem0, map->vaddr, set_params);
		if (r < 0)
			goto fail;
	}

	if (dmem0) {
		m = IMGU_ABI_MEM_ISP_DMEM0;
		map = ipu3_css_pool_last(&css->pool.binary_params_p[m], 1);
		r = ipu3_css_cfg_dmem0(css, use, dmem0, map->vaddr, set_params);
		if (r < 0)
			goto fail;
	}

	/* Get a new gdc only if a new gdc is given, or none yet */
	if (bi->info.isp.sp.enable.dvs_6axis) {
		unsigned int a = IPU3_CSS_AUX_FRAME_REF;
		unsigned int g = IPU3_CSS_RECT_GDC;
		unsigned int e = IPU3_CSS_RECT_ENVELOPE;
		if (set_params && !set_params->use.gdc)
			set_gdc = NULL;
		if (set_gdc || !ipu3_css_pool_last(&css->pool.gdc, 0)->vaddr) {
			if (ipu3_css_pool_get(&css->pool.gdc, css->frame) < 0)
				goto fail;

			map = ipu3_css_pool_last(&css->pool.gdc, 0);
			gdc =  map->vaddr;
			/* Config geometric distortion correction table (gdc) */
			ipu3_css_cfg_gdc_table(gdc,
					       css->aux_frames[a].bytesperline /
					       css->aux_frames[a].bytesperpixel,
					       css->aux_frames[a].height,
					       css->rect[g].width,
					       css->rect[g].height,
					       css->rect[e].width + FILTER_SIZE,
					       css->rect[e].height +
					       FILTER_SIZE);
		}
	}

	/* Get a new obgrid only if a new obgrid is given, or none yet */
	if (set_params && !set_params->use.obgrid)
		set_obgrid = NULL;
	if (set_obgrid && obgrid_bytes < obgrid_size / stripes)
		goto fail;
	if (set_obgrid || (set_params && set_params->use.obgrid_param) ||
		!ipu3_css_pool_last(&css->pool.obgrid, 0)->vaddr) {
		if (ipu3_css_pool_get(&css->pool.obgrid, css->frame) < 0)
			goto fail;
		map = ipu3_css_pool_last(&css->pool.obgrid, 0);
		obgrid = map->vaddr;

		/* Configure optical black level grid (obgrid) */
		if (set_obgrid) {
			for (s = 0; s < stripes; s++)
				memcpy((void *)obgrid +
					(obgrid_size / stripes) * s, set_obgrid,
					obgrid_size / stripes);

		} else if (set_params && set_params->use.obgrid_param) {
			for (s = 0; s < obgrid_size / sizeof(*obgrid); s++)
				obgrid[s] = set_params->obgrid_param;
		} else {
			memset(obgrid, 0, obgrid_size);
		}
	}

	/* Configure parameter set info, queued to `queue_id' */

	memset(param_set, 0, sizeof(*param_set));

	param_set->mem_map.acc_cluster_params_for_sp =
	    ipu3_css_pool_last(&css->pool.acc, 0)->daddr;

	param_set->mem_map.dvs_6axis_params_y =
	    ipu3_css_pool_last(&css->pool.gdc, 0)->daddr;

	for (s = 0; s < stripes; s++)
		param_set->mem_map.obgrid_tbl[s] =
		    ipu3_css_pool_last(&css->pool.obgrid, 0)->daddr +
		    (obgrid_size / stripes) * s;

	for (m = 0; m < IMGU_ABI_NUM_MEMORIES; m++)
		param_set->mem_map.isp_mem_param[stage][m] =
		    ipu3_css_pool_last(&css->pool.binary_params_p[m], 0)
		    ->daddr;

	/* Then queue the new parameter buffer */

	r = ipu3_css_queue_data(css, queue_id, thread,
		ipu3_css_pool_last(&css->pool.parameter_set_info, 0)->daddr);
	if (r < 0)
		goto fail;

	r = ipu3_css_queue_data(css, IMGU_ABI_QUEUE_EVENT_ID, 0,
			IMGU_ABI_EVENT_BUFFER_ENQUEUED(thread, queue_id));
	if (r < 0)
		goto fail_no_put;

	/* Finally dequeue all old parameter buffers */

	do {
		u32 daddr;

		r = ipu3_css_dequeue_data(css, queue_id, &daddr);
		if (r == -EBUSY)
			break;
		if (r)
			goto fail_no_put;
		r = ipu3_css_queue_data(css, IMGU_ABI_QUEUE_EVENT_ID, thread,
					IMGU_ABI_EVENT_BUFFER_DEQUEUED
					(queue_id));
		if (r < 0) {
			dev_err(css->dev, "failed to queue parameter event\n");
			goto fail_no_put;
		}
	} while (1);

	return 0;

fail:
	/*
	 * A failure, most likely the parameter queue was full.
	 * Return error but continue streaming. User can try submitting new
	 * parameters again later.
	 */

	ipu3_css_pool_put(&css->pool.parameter_set_info);
	if (acc)
		ipu3_css_pool_put(&css->pool.acc);
	if (gdc)
		ipu3_css_pool_put(&css->pool.gdc);
	if (obgrid)
		ipu3_css_pool_put(&css->pool.obgrid);
	if (vmem0)
		ipu3_css_pool_put(
			&css->pool.binary_params_p[IMGU_ABI_MEM_ISP_VMEM0]);
	if (dmem0)
		ipu3_css_pool_put(
			&css->pool.binary_params_p[IMGU_ABI_MEM_ISP_DMEM0]);

fail_no_put:
	return r;
}

int ipu3_css_irq_ack(struct ipu3_css *css)
{
	static const int NUM_SWIRQS = 3;
	struct imgu_fw_info *bi = &css->fwp->binary_header[css->fw_sp[0]];
	void __iomem *const base = css->base;
	u32 irq_status[IMGU_IRQCTRL_NUM];
	int i;

	u32 imgu_status = readl(base + IMGU_REG_INT_STATUS);

	writel(imgu_status, base + IMGU_REG_INT_STATUS);
	for (i = 0; i < IMGU_IRQCTRL_NUM; i++)
		irq_status[i] = readl(base + IMGU_REG_IRQCTRL_STATUS(i));

	for (i = 0; i < NUM_SWIRQS; i++) {
		if (irq_status[IMGU_IRQCTRL_SP0] & IMGU_IRQCTRL_IRQ_SW_PIN(i)) {
			/* SP SW interrupt */
			u32 cnt = readl(base + IMGU_REG_SP_DMEM_BASE(0) +
					bi->info.sp.output);
			u32 val = readl(base + IMGU_REG_SP_DMEM_BASE(0) +
					bi->info.sp.output + 4 + 4 * i);

			dev_dbg(css->dev, "%s: swirq %i cnt %i val 0x%x\n",
				 __func__, i, cnt, val);
		}
	}

	for (i = IMGU_IRQCTRL_NUM - 1; i >= 0; i--)
		if (irq_status[i]) {
			writel(irq_status[i], base + IMGU_REG_IRQCTRL_CLEAR(i));
			/* Wait for write to complete */
			readl(base + IMGU_REG_IRQCTRL_ENABLE(i));
		}

	dev_dbg(css->dev, "%s: imgu 0x%x main 0x%x sp0 0x%x sp1 0x%x\n",
		__func__,
		imgu_status, irq_status[IMGU_IRQCTRL_MAIN],
		irq_status[IMGU_IRQCTRL_SP0], irq_status[IMGU_IRQCTRL_SP1]);

	if (!imgu_status && !irq_status[IMGU_IRQCTRL_MAIN])
		return -ENOMSG;

	return 0;
}
