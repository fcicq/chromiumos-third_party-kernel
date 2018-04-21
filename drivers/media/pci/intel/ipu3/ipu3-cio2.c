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
 *
 * Based partially on Intel IPU4 driver written by
 *  Sakari Ailus <sakari.ailus@linux.intel.com>
 *  Samu Onkalo <samu.onkalo@intel.com>
 *  Jouni Högander <jouni.hogander@intel.com>
 *  Jouni Ukkonen <jouni.ukkonen@intel.com>
 *  Antti Laakso <antti.laakso@intel.com>
 * et al.
 *
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/vmalloc.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-sg.h>

#include "ipu3-cio2.h"

struct ipu3_cio2_fmt {
	u32 mbus_code;
	u32 fourcc;
	u8 mipicode;
};

/*
 * These are raw formats used in Intel's third generation of
 * Image Processing Unit known as IPU3.
 * 10bit raw bayer packed, 32 bytes for every 25 pixels,
 * last LSB 6 bits unused.
 */
static const struct ipu3_cio2_fmt formats[] = {
	{	/* put default entry at beginning */
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.fourcc		= V4L2_PIX_FMT_IPU3_SGRBG10,
		.mipicode	= 0x2b,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.fourcc		= V4L2_PIX_FMT_IPU3_SGBRG10,
		.mipicode	= 0x2b,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.fourcc		= V4L2_PIX_FMT_IPU3_SBGGR10,
		.mipicode	= 0x2b,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.fourcc		= V4L2_PIX_FMT_IPU3_SRGGB10,
		.mipicode	= 0x2b,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/*
 * cio2_find_format - lookup color format by fourcc or/and media bus code
 * @pixelformat: fourcc to match, ignored if null
 * @mbus_code: media bus code to match, ignored if null
 */
static const struct ipu3_cio2_fmt *cio2_find_format(const u32 *pixelformat,
						    const u32 *mbus_code)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (pixelformat && *pixelformat != formats[i].fourcc)
			continue;
		if (mbus_code && *mbus_code != formats[i].mbus_code)
			continue;

		return &formats[i];
	}

	return NULL;
}

static inline u32 cio2_bytesperline(const unsigned int width)
{
	/*
	 * 64 bytes for every 50 pixels, the line length
	 * in bytes is multiple of 64 (line end alignment).
	 */
	return DIV_ROUND_UP(width, 50) * 64;
}

/**************** FBPT operations ****************/

static void cio2_fbpt_exit_dummy(struct cio2_device *cio2)
{
	if (cio2->dummy_lop) {
		dma_free_coherent(&cio2->pci_dev->dev, CIO2_PAGE_SIZE,
				cio2->dummy_lop, cio2->dummy_lop_bus_addr);
		cio2->dummy_lop = NULL;
	}
	if (cio2->dummy_page) {
		dma_free_coherent(&cio2->pci_dev->dev, CIO2_PAGE_SIZE,
				cio2->dummy_page, cio2->dummy_page_bus_addr);
		cio2->dummy_page = NULL;
	}
}

static int cio2_fbpt_init_dummy(struct cio2_device *cio2)
{
	unsigned int i;

	cio2->dummy_page = dma_alloc_coherent(&cio2->pci_dev->dev,
				CIO2_PAGE_SIZE, &cio2->dummy_page_bus_addr,
				GFP_KERNEL);
	cio2->dummy_lop = dma_alloc_coherent(&cio2->pci_dev->dev,
				CIO2_PAGE_SIZE, &cio2->dummy_lop_bus_addr,
				GFP_KERNEL);
	if (!cio2->dummy_page || !cio2->dummy_lop) {
		cio2_fbpt_exit_dummy(cio2);
		return -ENOMEM;
	}
	/*
	 * List of Pointers(LOP) contains 1024x32b pointers to 4KB page each
	 * Initialize each entry to dummy_page bus base address.
	 */
	for (i = 0; i < CIO2_PAGE_SIZE / sizeof(*cio2->dummy_lop); i++)
		cio2->dummy_lop[i] = cio2->dummy_page_bus_addr >> PAGE_SHIFT;

	return 0;
}

static void cio2_fbpt_entry_enable(struct cio2_device *cio2,
				   struct cio2_fbpt_entry entry[CIO2_MAX_LOPS])
{
	/*
	 * The CPU first initializes some fields in fbpt, then sets
	 * the VALID bit, this barrier is to ensure that the DMA(device)
	 * does not see the VALID bit enabled before other fields are
	 * initialized; otherwise it could lead to havoc.
	 */
	dma_wmb();

	/*
	 * Request interrupts for start and completion
	 * Valid bit is applicable only to 1st entry
	 */
	entry[0].first_entry.ctrl = CIO2_FBPT_CTRL_VALID |
		CIO2_FBPT_CTRL_IOC | CIO2_FBPT_CTRL_IOS;
}

/* Initialize fpbt entries to point to dummy frame */
static void cio2_fbpt_entry_init_dummy(struct cio2_device *cio2,
				       struct cio2_fbpt_entry
				       entry[CIO2_MAX_LOPS])
{
	unsigned int i;

	entry[0].first_entry.first_page_offset = 0;
	entry[1].second_entry.num_of_pages =
		CIO2_PAGE_SIZE / sizeof(u32) * CIO2_MAX_LOPS;
	entry[1].second_entry.last_page_available_bytes = CIO2_PAGE_SIZE - 1;

	for (i = 0; i < CIO2_MAX_LOPS; i++)
		entry[i].lop_page_addr = cio2->dummy_lop_bus_addr >> PAGE_SHIFT;

	cio2_fbpt_entry_enable(cio2, entry);
}

/* Initialize fpbt entries to point to a given buffer */
static void cio2_fbpt_entry_init_buf(struct cio2_device *cio2,
				     struct cio2_buffer *b,
				     struct cio2_fbpt_entry
				     entry[CIO2_MAX_LOPS])
{
	struct vb2_buffer *vb = &b->vbb.vb2_buf;
	unsigned int length = vb->planes[0].length;
	dma_addr_t lop_bus_addr = b->lop_bus_addr;
	int remaining;

	entry[0].first_entry.first_page_offset = b->offset;
	remaining = length + entry[0].first_entry.first_page_offset;
	entry[1].second_entry.num_of_pages =
		DIV_ROUND_UP(remaining, CIO2_PAGE_SIZE);
	/*
	 * last_page_available_bytes has the offset of the last byte in the
	 * last page which is still accessible by DMA. DMA cannot access
	 * beyond this point. Valid range for this is from 0 to 4095.
	 * 0 indicates 1st byte in the page is DMA accessible.
	 * 4095 (CIO2_PAGE_SIZE - 1) means every single byte in the last page
	 * is available for DMA transfer.
	 */
	entry[1].second_entry.last_page_available_bytes =
			(remaining & ~PAGE_MASK) ?
				(remaining & ~PAGE_MASK) - 1 :
				CIO2_PAGE_SIZE - 1;
	/* Fill FBPT */
	remaining = length;
	while (remaining > 0) {
		entry->lop_page_addr = lop_bus_addr >> PAGE_SHIFT;
		lop_bus_addr += CIO2_PAGE_SIZE;
		remaining -= CIO2_PAGE_SIZE / sizeof(u32) * CIO2_PAGE_SIZE;
		entry++;
	}

	/*
	 * The first not meaningful FBPT entry should point to a valid LOP
	 */
	entry->lop_page_addr = cio2->dummy_lop_bus_addr >> PAGE_SHIFT;

	cio2_fbpt_entry_enable(cio2, entry);
}

static int cio2_fbpt_init(struct cio2_device *cio2, struct cio2_queue *q)
{
	struct device *dev = &cio2->pci_dev->dev;

	q->fbpt = dma_alloc_coherent(dev, CIO2_FBPT_SIZE,
			&q->fbpt_bus_addr, GFP_KERNEL);
	if (!q->fbpt)
		return -ENOMEM;

	memset(q->fbpt, 0, CIO2_FBPT_SIZE);

	return 0;
}

static int cio2_fbpt_rearrange(struct cio2_device *cio2, struct cio2_queue *q)
{
	struct cio2_fbpt_entry *fbpt_temp;
	int i, j;
	int size, entries;
	struct cio2_buffer *bufs[CIO2_MAX_BUFFERS];

	for (i = 0, j = q->bufs_first; i < CIO2_MAX_BUFFERS;
		i++, j = (j + 1) % CIO2_MAX_BUFFERS)
		if (q->bufs[j])
			break;

	if (i == CIO2_MAX_BUFFERS)
		return 0;

	fbpt_temp = vmalloc(CIO2_FBPT_SIZE);
	if (!fbpt_temp) {
		dev_err(&cio2->pci_dev->dev, "Out of memory.\n");
		return -ENOMEM;
	}

	entries = (CIO2_MAX_BUFFERS - j) * CIO2_MAX_LOPS;
	size = entries * sizeof(struct cio2_fbpt_entry);

	memcpy(fbpt_temp, q->fbpt + j * CIO2_MAX_LOPS, size);
	memcpy(fbpt_temp + entries, q->fbpt, CIO2_FBPT_SIZE - size);
	memcpy(q->fbpt, fbpt_temp, CIO2_FBPT_SIZE);
	memcpy(bufs, q->bufs + j, (CIO2_MAX_BUFFERS - j) * sizeof(*bufs));
	memcpy(bufs + CIO2_MAX_BUFFERS - j, q->bufs, j * sizeof(*bufs));
	memcpy(q->bufs, bufs, CIO2_MAX_BUFFERS * sizeof(*bufs));

	/*
	 * DMA clears the valid bit when accessing the buffer.
	 * When stopping stream in suspend callback, some of the buffers
	 * may be in invalid state. After resume, when DMA meets the inavlid
	 * buffer, it will halt and stop receiving new data.
	 * To avoid DMA halting, set the valid bit for all buffers in FBPT.
	 */
	for (i = 0; i < CIO2_MAX_BUFFERS; i++)
		cio2_fbpt_entry_enable(cio2, q->fbpt + i * CIO2_MAX_LOPS);

	vfree(fbpt_temp);

	return 0;
}

static void cio2_fbpt_exit(struct cio2_queue *q, struct device *dev)
{
	dma_free_coherent(dev, CIO2_FBPT_SIZE, q->fbpt, q->fbpt_bus_addr);
}

/**************** CSI2 hardware setup ****************/

/*
 * The CSI2 receiver has several parameters affecting
 * the receiver timings. These depend on the MIPI bus frequency
 * F in Hz (sensor transmitter rate) as follows:
 *     register value = (A/1e9 + B * UI) / COUNT_ACC
 * where
 *      UI = 1 / (2 * F) in seconds
 *      COUNT_ACC = counter accuracy in seconds
 *      For IPU3 COUNT_ACC = 0.0625
 *
 * A and B are coefficients from the table below,
 * depending whether the register minimum or maximum value is
 * calculated.
 *                                     Minimum     Maximum
 * Clock lane                          A     B     A     B
 * reg_rx_csi_dly_cnt_termen_clane     0     0    38     0
 * reg_rx_csi_dly_cnt_settle_clane    95    -8   300   -16
 * Data lanes
 * reg_rx_csi_dly_cnt_termen_dlane0    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane0   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane1    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane1   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane2    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane2   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane3    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane3   85    -2   145    -6
 *
 * We use the minimum values of both A and B.
 */
#define DIV_SHIFT	8
static int cio2_rx_timing(s32 a, s32 b, s64 freq)
{
	int r;
	const u32 accinv = 16;

	/*
	 * b could be 0, -2 or -8, so |accinv * b| is always
	 * less than (1 << ds) and thus |r| < 500000000.
	 */
	r = accinv * b * (500000000 >> DIV_SHIFT);
	r /= freq;
	/* max value of a is 95 */
	r += accinv * a;

	return r;
};

/* Computation for the Delay value for Termination enable of Clock lane HS Rx */
static int cio2_csi2_calc_timing(struct cio2_device *cio2, struct cio2_queue *q,
			    struct cio2_csi2_timing *timing)
{
	struct device *dev = &cio2->pci_dev->dev;
	struct v4l2_querymenu qm = {.id = V4L2_CID_LINK_FREQ, };
	struct v4l2_ctrl *link_freq;
	s64 freq;
	int r;

	if (!q->sensor)
		return -ENODEV;

	link_freq = v4l2_ctrl_find(q->sensor->ctrl_handler,
						V4L2_CID_LINK_FREQ);
	if (!link_freq) {
		dev_err(dev, "failed to find LINK_FREQ\n");
		return -EPIPE;
	};

	qm.index = v4l2_ctrl_g_ctrl(link_freq);
	r = v4l2_querymenu(q->sensor->ctrl_handler, &qm);
	if (r) {
		dev_err(dev, "failed to get menu item\n");
		return r;
	}

	if (!qm.value) {
		dev_err(dev, "error invalid link_freq\n");
		return -EINVAL;
	}
	freq = qm.value;

	dev_dbg(dev, "link freq is %lld\n", qm.value);

	/* check before calling cio2_rx_timing to avoid divide by 0 */
	freq = qm.value >> DIV_SHIFT;
	if (WARN_ON(freq == 0))
		return -EINVAL;

	timing->clk_termen = cio2_rx_timing(CIO2_CSIRX_DLY_CNT_TERMEN_CLANE_A,
				CIO2_CSIRX_DLY_CNT_TERMEN_CLANE_B, freq);
	timing->clk_settle = cio2_rx_timing(CIO2_CSIRX_DLY_CNT_SETTLE_CLANE_A,
				CIO2_CSIRX_DLY_CNT_SETTLE_CLANE_B, freq);
	timing->dat_termen = cio2_rx_timing(CIO2_CSIRX_DLY_CNT_TERMEN_DLANE_A,
				CIO2_CSIRX_DLY_CNT_TERMEN_DLANE_B, freq);
	timing->dat_settle = cio2_rx_timing(CIO2_CSIRX_DLY_CNT_SETTLE_DLANE_A,
				CIO2_CSIRX_DLY_CNT_SETTLE_DLANE_B, freq);

	dev_dbg(dev, "freq ct value is %d\n", timing->clk_termen);
	dev_dbg(dev, "freq cs value is %d\n", timing->clk_settle);
	dev_dbg(dev, "freq dt value is %d\n", timing->dat_termen);
	dev_dbg(dev, "freq ds value is %d\n", timing->dat_settle);

	return 0;
};

static int cio2_hw_init(struct cio2_device *cio2, struct cio2_queue *q)
{
	static const int NUM_VCS = 4;
	static const int SID;	/* Stream id */
	static const int ENTRY;
	static const int FBPT_WIDTH = DIV_ROUND_UP(CIO2_MAX_LOPS,
					CIO2_FBPT_SUBENTRY_UNIT);
	const u32 num_buffers1 = CIO2_MAX_BUFFERS - 1;
	const struct ipu3_cio2_fmt *fmt;
	void __iomem *const base = cio2->base;
	u8 lanes, csi2bus = q->csi2.port;
	u8 sensor_vc = SENSOR_VIR_CH_DFLT;
	struct cio2_csi2_timing timing;
	int i, r;

	fmt = cio2_find_format(NULL, &q->subdev_fmt.code);
	if (!fmt)
		return -EINVAL;

	lanes = r = q->csi2.lanes;
	if (r < 0)
		return r;

	writel(CIO2_PBM_WMCTRL1_MIN_2CK |
	       CIO2_PBM_WMCTRL1_MID1_2CK |
	       CIO2_PBM_WMCTRL1_MID2_2CK, base + CIO2_REG_PBM_WMCTRL1);
	writel(CIO2_PBM_WMCTRL2_HWM_2CK << CIO2_PBM_WMCTRL2_HWM_2CK_SHIFT |
	       CIO2_PBM_WMCTRL2_LWM_2CK << CIO2_PBM_WMCTRL2_LWM_2CK_SHIFT |
	       CIO2_PBM_WMCTRL2_OBFFWM_2CK <<
	       CIO2_PBM_WMCTRL2_OBFFWM_2CK_SHIFT |
	       CIO2_PBM_WMCTRL2_TRANSDYN << CIO2_PBM_WMCTRL2_TRANSDYN_SHIFT |
	       CIO2_PBM_WMCTRL2_OBFF_MEM_EN, base + CIO2_REG_PBM_WMCTRL2);
	writel(CIO2_PBM_ARB_CTRL_LANES_DIV << CIO2_PBM_ARB_CTRL_LANES_DIV |
	       CIO2_PBM_ARB_CTRL_LE_EN |
	       CIO2_PBM_ARB_CTRL_PLL_POST_SHTDN <<
	       CIO2_PBM_ARB_CTRL_PLL_POST_SHTDN_SHIFT |
	       CIO2_PBM_ARB_CTRL_PLL_AHD_WK_UP <<
	       CIO2_PBM_ARB_CTRL_PLL_AHD_WK_UP_SHIFT,
	       base + CIO2_REG_PBM_ARB_CTRL);
	writel(CIO2_CSIRX_STATUS_DLANE_HS_MASK,
	       q->csi_rx_base + CIO2_REG_CSIRX_STATUS_DLANE_HS);
	writel(CIO2_CSIRX_STATUS_DLANE_LP_MASK,
	       q->csi_rx_base + CIO2_REG_CSIRX_STATUS_DLANE_LP);

	writel(CIO2_FB_HPLL_FREQ, base + CIO2_REG_FB_HPLL_FREQ);
	writel(CIO2_ISCLK_RATIO, base + CIO2_REG_ISCLK_RATIO);

	/* Configure MIPI backend */
	for (i = 0; i < NUM_VCS; i++)
		writel(1, q->csi_rx_base + CIO2_REG_MIPIBE_SP_LUT_ENTRY(i));

	/* There are 16 short packet LUT entry */
	for (i = 0; i < 16; i++)
		writel(CIO2_MIPIBE_LP_LUT_ENTRY_DISREGARD,
		       q->csi_rx_base + CIO2_REG_MIPIBE_LP_LUT_ENTRY(i));
	writel(CIO2_MIPIBE_GLOBAL_LUT_DISREGARD,
	       q->csi_rx_base + CIO2_REG_MIPIBE_GLOBAL_LUT_DISREGARD);

	writel(CIO2_INT_EN_EXT_IE_MASK, base + CIO2_REG_INT_EN_EXT_IE);
	writel(CIO2_IRQCTRL_MASK, q->csi_rx_base + CIO2_REG_IRQCTRL_MASK);
	writel(CIO2_IRQCTRL_MASK, q->csi_rx_base + CIO2_REG_IRQCTRL_ENABLE);
	writel(0, q->csi_rx_base + CIO2_REG_IRQCTRL_EDGE);
	writel(0, q->csi_rx_base + CIO2_REG_IRQCTRL_LEVEL_NOT_PULSE);
	writel(CIO2_INT_EN_EXT_OE_MASK, base + CIO2_REG_INT_EN_EXT_OE);

	writel(CIO2_REG_INT_EN_IRQ | CIO2_INT_IOC(CIO2_DMA_CHAN) |
	       CIO2_REG_INT_EN_IOS(CIO2_DMA_CHAN),
	       base + CIO2_REG_INT_EN);

	writel((CIO2_PXM_PXF_FMT_CFG_BPP_10 | CIO2_PXM_PXF_FMT_CFG_PCK_64B)
	       << CIO2_PXM_PXF_FMT_CFG_SID0_SHIFT,
	       base + CIO2_REG_PXM_PXF_FMT_CFG0(csi2bus));
	writel(SID << CIO2_MIPIBE_LP_LUT_ENTRY_SID_SHIFT |
	       sensor_vc << CIO2_MIPIBE_LP_LUT_ENTRY_VC_SHIFT |
	       fmt->mipicode << CIO2_MIPIBE_LP_LUT_ENTRY_FORMAT_TYPE_SHIFT,
	       q->csi_rx_base + CIO2_REG_MIPIBE_LP_LUT_ENTRY(ENTRY));
	writel(0, q->csi_rx_base + CIO2_REG_MIPIBE_COMP_FORMAT(sensor_vc));
	writel(0, q->csi_rx_base + CIO2_REG_MIPIBE_FORCE_RAW8);
	writel(0, base + CIO2_REG_PXM_SID2BID0(csi2bus));

	r = cio2_csi2_calc_timing(cio2, q, &timing);
	if (r) {
		/* Use default values */
		for (i = -1; i < lanes; i++) {
			writel(0x4, q->csi_rx_base +
				CIO2_REG_CSIRX_DLY_CNT_TERMEN(i));
			writel(0x570, q->csi_rx_base +
				CIO2_REG_CSIRX_DLY_CNT_SETTLE(i));
		}
	} else {
		i = CIO2_CSIRX_DLY_CNT_CLANE_IDX;
		writel(timing.clk_termen, q->csi_rx_base +
			CIO2_REG_CSIRX_DLY_CNT_TERMEN(i));
		writel(timing.clk_settle, q->csi_rx_base +
			CIO2_REG_CSIRX_DLY_CNT_SETTLE(i));

		for (i = 0; i < lanes; i++) {
			writel(timing.dat_termen, q->csi_rx_base +
				CIO2_REG_CSIRX_DLY_CNT_TERMEN(i));
			writel(timing.dat_settle, q->csi_rx_base +
				CIO2_REG_CSIRX_DLY_CNT_SETTLE(i));
		}
	}

	writel(lanes, q->csi_rx_base + CIO2_REG_CSIRX_NOF_ENABLED_LANES);
	writel(CIO2_CGC_PRIM_TGE |
	       CIO2_CGC_SIDE_TGE |
	       CIO2_CGC_XOSC_TGE |
	       CIO2_CGC_D3I3_TGE |
	       CIO2_CGC_CSI2_INTERFRAME_TGE |
	       CIO2_CGC_CSI2_PORT_DCGE |
	       CIO2_CGC_SIDE_DCGE |
	       CIO2_CGC_PRIM_DCGE |
	       CIO2_CGC_ROSC_DCGE |
	       CIO2_CGC_XOSC_DCGE |
	       CIO2_CGC_CLKGATE_HOLDOFF << CIO2_CGC_CLKGATE_HOLDOFF_SHIFT |
	       CIO2_CGC_CSI_CLKGATE_HOLDOFF
	       << CIO2_CGC_CSI_CLKGATE_HOLDOFF_SHIFT, base + CIO2_REG_CGC);
	writel(CIO2_LTRCTRL_LTRDYNEN, base + CIO2_REG_LTRCTRL);
	writel(CIO2_LTRVAL0_VAL << CIO2_LTRVAL02_VAL_SHIFT |
	       CIO2_LTRVAL0_SCALE << CIO2_LTRVAL02_SCALE_SHIFT |
	       CIO2_LTRVAL1_VAL << CIO2_LTRVAL13_VAL_SHIFT |
	       CIO2_LTRVAL1_SCALE << CIO2_LTRVAL13_SCALE_SHIFT,
	       base + CIO2_REG_LTRVAL01);
	writel(CIO2_LTRVAL2_VAL << CIO2_LTRVAL02_VAL_SHIFT |
	       CIO2_LTRVAL2_SCALE << CIO2_LTRVAL02_SCALE_SHIFT |
	       CIO2_LTRVAL3_VAL << CIO2_LTRVAL13_VAL_SHIFT |
	       CIO2_LTRVAL3_SCALE << CIO2_LTRVAL13_SCALE_SHIFT,
	       base + CIO2_REG_LTRVAL23);

	for (i = 0; i < CIO2_NUM_DMA_CHAN; i++) {
		writel(0, base + CIO2_REG_CDMABA(i));
		writel(0, base + CIO2_REG_CDMAC0(i));
		writel(0, base + CIO2_REG_CDMAC1(i));
	}

	/* Enable DMA */
	writel(q->fbpt_bus_addr >> PAGE_SHIFT,
	       base + CIO2_REG_CDMABA(CIO2_DMA_CHAN));

	writel(num_buffers1 << CIO2_CDMAC0_FBPT_LEN_SHIFT |
	       FBPT_WIDTH << CIO2_CDMAC0_FBPT_WIDTH_SHIFT |
	       CIO2_CDMAC0_DMA_INTR_ON_FE |
	       CIO2_CDMAC0_FBPT_UPDATE_FIFO_FULL |
	       CIO2_CDMAC0_DMA_EN |
	       CIO2_CDMAC0_DMA_INTR_ON_FS |
	       CIO2_CDMAC0_DMA_HALTED, base + CIO2_REG_CDMAC0(CIO2_DMA_CHAN));

	writel(1 << CIO2_CDMAC1_LINENUMUPDATE_SHIFT,
	       base + CIO2_REG_CDMAC1(CIO2_DMA_CHAN));

	writel(0, base + CIO2_REG_PBM_FOPN_ABORT);

	writel(CIO2_PXM_FRF_CFG_CRC_TH << CIO2_PXM_FRF_CFG_CRC_TH_SHIFT |
	       CIO2_PXM_FRF_CFG_MSK_ECC_DPHY_NR |
	       CIO2_PXM_FRF_CFG_MSK_ECC_RE |
	       CIO2_PXM_FRF_CFG_MSK_ECC_DPHY_NE,
	       base + CIO2_REG_PXM_FRF_CFG(q->csi2.port));

	/* Clear interrupts */
	writel(CIO2_IRQCTRL_MASK, q->csi_rx_base + CIO2_REG_IRQCTRL_CLEAR);
	writel(~0, base + CIO2_REG_INT_STS_EXT_OE);
	writel(~0, base + CIO2_REG_INT_STS_EXT_IE);
	writel(~0, base + CIO2_REG_INT_STS);

	/* Enable devices, starting from the last device in the pipe */
	writel(1, q->csi_rx_base + CIO2_REG_MIPIBE_ENABLE);
	writel(1, q->csi_rx_base + CIO2_REG_CSIRX_ENABLE);

	return 0;
}

static void cio2_hw_exit(struct cio2_device *cio2, struct cio2_queue *q)
{
	void __iomem *base = cio2->base;
	unsigned int i, maxloops = 1000;

	/* Disable CSI receiver and MIPI backend devices */
	writel(0, q->csi_rx_base + CIO2_REG_CSIRX_ENABLE);
	writel(0, q->csi_rx_base + CIO2_REG_MIPIBE_ENABLE);

	/* Halt DMA */
	writel(0, base + CIO2_REG_CDMAC0(CIO2_DMA_CHAN));
	do {
		if (readl(base + CIO2_REG_CDMAC0(CIO2_DMA_CHAN)) &
		    CIO2_CDMAC0_DMA_HALTED)
			break;
		usleep_range(1000, 2000);
	} while (--maxloops);
	if (!maxloops)
		dev_err(&cio2->pci_dev->dev,
			"DMA %i can not be halted\n", CIO2_DMA_CHAN);

	for (i = 0; i < CIO2_NUM_PORTS; i++) {
		writel(readl(base + CIO2_REG_PXM_FRF_CFG(i)) |
		       CIO2_PXM_FRF_CFG_ABORT, base + CIO2_REG_PXM_FRF_CFG(i));
		writel(readl(base + CIO2_REG_PBM_FOPN_ABORT) |
		       CIO2_PBM_FOPN_ABORT(i), base + CIO2_REG_PBM_FOPN_ABORT);
	}
}

static void cio2_buffer_done(struct cio2_device *cio2, unsigned int dma_chan)
{
	struct device *dev = &cio2->pci_dev->dev;
	struct cio2_queue *q = cio2->cur_queue;
	int buffers_found = 0;

	if (dma_chan >= CIO2_QUEUES) {
		dev_err(dev, "bad DMA channel %i\n", dma_chan);
		return;
	}

	/* Find out which buffer(s) are ready */
	do {
		struct cio2_fbpt_entry *const entry =
			&q->fbpt[q->bufs_first * CIO2_MAX_LOPS];
		struct cio2_buffer *b;

		if (entry->first_entry.ctrl & CIO2_FBPT_CTRL_VALID)
			break;

		b = q->bufs[q->bufs_first];
		if (b) {
			u64 ns = ktime_get_ns();

			q->bufs[q->bufs_first] = NULL;
			atomic_dec(&q->bufs_queued);
			dev_dbg(&cio2->pci_dev->dev,
				"buffer %i done\n", b->vbb.vb2_buf.index);

			/* Fill vb2 buffer entries and tell it's ready */
			b->vbb.timestamp = ns_to_timeval(ns);
			b->vbb.flags = V4L2_BUF_FLAG_DONE;
			b->vbb.field = V4L2_FIELD_NONE;
			memset(&b->vbb.timecode, 0, sizeof(b->vbb.timecode));
			b->vbb.sequence = atomic_read(&q->frame_sequence);
			vb2_buffer_done(&b->vbb.vb2_buf, VB2_BUF_STATE_DONE);
		}
		atomic_inc(&q->frame_sequence);
		cio2_fbpt_entry_init_dummy(cio2, entry);
		q->bufs_first = (q->bufs_first + 1) % CIO2_MAX_BUFFERS;
		buffers_found++;
	} while (1);

	if (buffers_found == 0)
		dev_warn(&cio2->pci_dev->dev,
			 "no ready buffers found on DMA channel %i\n",
			 dma_chan);
}

static void cio2_queue_event_sof(struct cio2_device *cio2, struct cio2_queue *q)
{
	struct v4l2_event event = {
		.type = V4L2_EVENT_FRAME_SYNC,
		.u.frame_sync.frame_sequence =
			atomic_read(&q->frame_sequence),
	};
	v4l2_event_queue(q->subdev.devnode, &event);
}

static const char *const cio2_irq_errs[] = {
	"single packet header error corrected",
	"multiple packet header errors detected",
	"payload checksum (CRC) error",
	"fifo overflow",
	"reserved short packet data type detected",
	"reserved long packet data type detected",
	"incomplete long packet detected",
	"frame sync error",
	"line sync error",
	"DPHY start of transmission error",
	"DPHY synchronization error",
	"escape mode error",
	"escape mode trigger event",
	"escape mode ultra-low power state for data lane(s)",
	"escape mode ultra-low power state exit for clock lane",
	"inter-frame short packet discarded",
	"inter-frame long packet discarded",
	"non-matching Long Packet stalled",
};

static const char *const cio2_port_errs[] = {
	"ECC recoverable",
	"DPHY not recoverable",
	"ECC not recoverable",
	"CRC error",
	"INTERFRAMEDATA",
	"PKT2SHORT",
	"PKT2LONG",
};

static void cio2_irq_handle_once(struct cio2_device *cio2, u32 int_status)
{
	void __iomem *const base = cio2->base;
	struct device *dev = &cio2->pci_dev->dev;

	if (int_status & CIO2_INT_IOOE) {
		/*
		 * Interrupt on Output Error:
		 * 1) SRAM is full and FS received, or
		 * 2) An invalid bit detected by DMA.
		 */
		u32 oe_status, oe_clear;

		oe_clear = readl(base + CIO2_REG_INT_STS_EXT_OE);
		oe_status = oe_clear;

		if (oe_status & CIO2_INT_EXT_OE_DMAOE_MASK) {
			dev_err(dev, "DMA output error: 0x%x\n",
				(oe_status & CIO2_INT_EXT_OE_DMAOE_MASK)
				>> CIO2_INT_EXT_OE_DMAOE_SHIFT);
			oe_status &= ~CIO2_INT_EXT_OE_DMAOE_MASK;
		}
		if (oe_status & CIO2_INT_EXT_OE_OES_MASK) {
			dev_err(dev, "DMA output error on CSI2 buses: 0x%x\n",
				(oe_status & CIO2_INT_EXT_OE_OES_MASK)
				>> CIO2_INT_EXT_OE_OES_SHIFT);
			oe_status &= ~CIO2_INT_EXT_OE_OES_MASK;
		}
		writel(oe_clear, base + CIO2_REG_INT_STS_EXT_OE);
		if (oe_status)
			dev_warn(dev, "unknown interrupt 0x%x on OE\n",
				 oe_status);
		int_status &= ~CIO2_INT_IOOE;
	}

	if (int_status & CIO2_INT_IOC_MASK) {
		/* DMA IO done -- frame ready */
		u32 clr = 0;
		unsigned int d;

		for (d = 0; d < CIO2_NUM_DMA_CHAN; d++)
			if (int_status & CIO2_INT_IOC(d)) {
				clr |= CIO2_INT_IOC(d);
				dev_dbg(dev, "DMA %i done\n", d);
				cio2_buffer_done(cio2, d);
			}
		int_status &= ~clr;
	}

	if (int_status & CIO2_INT_IOS_IOLN_MASK) {
		/* DMA IO starts or reached specified line */
		u32 clr = 0;
		unsigned int d;

		for (d = 0; d < CIO2_NUM_DMA_CHAN; d++)
			if (int_status & CIO2_INT_IOS_IOLN(d)) {
				clr |= CIO2_INT_IOS_IOLN(d);
				if (d == CIO2_DMA_CHAN)
					cio2_queue_event_sof(cio2,
							     cio2->cur_queue);
				dev_dbg(dev,
					"DMA %i started or reached line\n", d);
			}
		int_status &= ~clr;
	}

	if (int_status & (CIO2_INT_IOIE | CIO2_INT_IOIRQ)) {
		/* CSI2 receiver (error) interrupt */
		u32 ie_status, ie_clear;
		unsigned int port;

		ie_clear = readl(base + CIO2_REG_INT_STS_EXT_IE);
		ie_status = ie_clear;

		for (port = 0; port < CIO2_NUM_PORTS; port++) {
			u32 port_status = (ie_status >> (port * 8)) & 0xff;
			u32 err_mask = BIT_MASK(ARRAY_SIZE(cio2_port_errs)) - 1;
			void __iomem *const csi_rx_base =
						base + CIO2_REG_PIPE_BASE(port);
			unsigned int i;

			while (port_status & err_mask) {
				i = ffs(port_status) - 1;
				dev_err(dev, "port %i error %s\n",
					port, cio2_port_errs[i]);
				ie_status &= ~BIT(port * 8 + i);
				port_status &= ~BIT(i);
			}

			if (ie_status & CIO2_INT_EXT_IE_IRQ(port)) {
				u32 csi2_status, csi2_clear;

				csi2_status = readl(csi_rx_base +
						CIO2_REG_IRQCTRL_STATUS);
				csi2_clear = csi2_status;
				err_mask =
					BIT_MASK(ARRAY_SIZE(cio2_irq_errs)) - 1;

				while (csi2_status & err_mask) {
					i = ffs(csi2_status) - 1;
					dev_err(dev,
						"CSI-2 receiver port %i: %s\n",
							port, cio2_irq_errs[i]);
					csi2_status &= ~BIT(i);
				}

				writel(csi2_clear,
				       csi_rx_base + CIO2_REG_IRQCTRL_CLEAR);
				if (csi2_status)
					dev_warn(dev,
						 "unknown CSI2 error 0x%x on port %i\n",
						 csi2_status, port);

				ie_status &= ~CIO2_INT_EXT_IE_IRQ(port);
			}
		}

		writel(ie_clear, base + CIO2_REG_INT_STS_EXT_IE);
		if (ie_status)
			dev_warn(dev, "unknown interrupt 0x%x on IE\n",
				 ie_status);

		int_status &= ~(CIO2_INT_IOIE | CIO2_INT_IOIRQ);
	}

	if (int_status)
		dev_warn(dev, "unknown interrupt 0x%x on INT\n", int_status);
}

static irqreturn_t cio2_irq(int irq, void *cio2_ptr)
{
	struct cio2_device *cio2 = cio2_ptr;
	void __iomem *const base = cio2->base;
	struct device *dev = &cio2->pci_dev->dev;
	u32 int_status;

	int_status = readl(base + CIO2_REG_INT_STS);
	dev_dbg(dev, "isr enter - interrupt status 0x%x\n", int_status);
	if (!int_status)
		return IRQ_NONE;

	do {
		writel(int_status, base + CIO2_REG_INT_STS);
		cio2_irq_handle_once(cio2, int_status);
		int_status = readl(base + CIO2_REG_INT_STS);
		if (int_status)
			dev_dbg(dev, "pending status 0x%x\n", int_status);
	} while (int_status);

	return IRQ_HANDLED;
}

/**************** Videobuf2 interface ****************/

static void cio2_vb2_return_all_buffers(struct cio2_queue *q,
					enum vb2_buffer_state state)
{
	unsigned int i;

	for (i = 0; i < CIO2_MAX_BUFFERS; i++) {
		if (q->bufs[i]) {
			atomic_dec(&q->bufs_queued);
			vb2_buffer_done(&q->bufs[i]->vbb.vb2_buf, state);
		}
	}
}

static int cio2_vb2_queue_setup(struct vb2_queue *vq, const void *parg,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct cio2_device *cio2 = vb2_get_drv_priv(vq);
	struct cio2_queue *q = vb2q_to_cio2_queue(vq);
	unsigned int i;

	*num_planes = q->format.num_planes;

	for (i = 0; i < *num_planes; ++i) {
		sizes[i] = q->format.plane_fmt[i].sizeimage;
		alloc_ctxs[i] = cio2->vb2_alloc_ctx;
	}

	*num_buffers = clamp_val(*num_buffers, 1, CIO2_MAX_BUFFERS);

	/* Initialize buffer queue */
	for (i = 0; i < CIO2_MAX_BUFFERS; i++) {
		q->bufs[i] = NULL;
		cio2_fbpt_entry_init_dummy(cio2, &q->fbpt[i * CIO2_MAX_LOPS]);
	}
	atomic_set(&q->bufs_queued, 0);
	q->bufs_first = 0;
	q->bufs_next = 0;

	return 0;
}

/* Called after each buffer is allocated */
static int cio2_vb2_buf_init(struct vb2_buffer *vb)
{
	struct cio2_device *cio2 = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = &cio2->pci_dev->dev;
	struct cio2_buffer *b =
		container_of(vb, struct cio2_buffer, vbb.vb2_buf);
	unsigned int length = vb->planes[0].length;
	unsigned int pages = DIV_ROUND_UP(length, CIO2_PAGE_SIZE), nr = 0;
	int lops  = DIV_ROUND_UP(pages + 1, CIO2_PAGE_SIZE / sizeof(u32));
	u32 *lop;
	struct sg_table *sg;
	struct sg_page_iter sg_iter;

	if (lops <= 0 || lops > CIO2_MAX_LOPS) {
		dev_err(dev, "%s: bad buffer size (%i)\n", __func__, length);
		return -ENOSPC;		/* Should never happen */
	}

	/* Allocate LOP table */
	b->lop = lop = dma_alloc_coherent(dev, lops * CIO2_PAGE_SIZE,
					&b->lop_bus_addr, GFP_KERNEL);
	if (!lop)
		return -ENOMEM;

	/* Fill LOP */
	sg = vb2_dma_sg_plane_desc(vb, 0);
	if (!sg)
		return -ENOMEM;

	if (sg->nents && sg->sgl)
		b->offset = sg->sgl->offset;

	for_each_sg_page(sg->sgl, &sg_iter, sg->nents, 0) {
		if (nr++ > pages)
			break;
		*lop++ = sg_page_iter_dma_address(&sg_iter) >> PAGE_SHIFT;
	}

	*lop++ = cio2->dummy_page_bus_addr >> PAGE_SHIFT;

	return 0;
}

/* Transfer buffer ownership to cio2 */
static void cio2_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct cio2_device *cio2 = vb2_get_drv_priv(vb->vb2_queue);
	struct cio2_queue *q =
		container_of(vb->vb2_queue, struct cio2_queue, vbq);
	struct cio2_buffer *b =
		container_of(vb, struct cio2_buffer, vbb.vb2_buf);
	struct cio2_fbpt_entry *entry;
	unsigned long flags;
	unsigned int i, j, next = q->bufs_next;
	int bufs_queued = atomic_inc_return(&q->bufs_queued);
	u32 fbpt_rp;

	dev_dbg(&cio2->pci_dev->dev, "queue buffer %d\n", vb->index);

	/*
	 * This code queues the buffer to the CIO2 DMA engine, which starts
	 * running once streaming has started. It is possible that this code
	 * gets pre-empted due to increased CPU load. Upon this, the driver
	 * does not get an opportunity to queue new buffers to the CIO2 DMA
	 * engine. When the DMA engine encounters an FBPT entry without the
	 * VALID bit set, the DMA engine halts, which requires a restart of
	 * the DMA engine and sensor, to continue streaming.
	 * This is not desired and is highly unlikely given that there are
	 * 32 FBPT entries that the DMA engine needs to process, to run into
	 * an FBPT entry, without the VALID bit set. We try to mitigate this
	 * by disabling interrupts for the duration of this queueing.
	 */
	local_irq_save(flags);

	fbpt_rp = (readl(cio2->base + CIO2_REG_CDMARI(CIO2_DMA_CHAN))
		   >> CIO2_CDMARI_FBPT_RP_SHIFT) & CIO2_CDMARI_FBPT_RP_MASK;

	/*
	 * fbpt_rp is the fbpt entry that the dma is currently working
	 * on, but since it could jump to next entry at any time,
	 * assume that we might already be there.
	 */
	fbpt_rp = (fbpt_rp + 1) % CIO2_MAX_BUFFERS;

	if (bufs_queued <= 1 || fbpt_rp == next)
		/* Buffers were drained */
		next = (fbpt_rp + 1) % CIO2_MAX_BUFFERS;

	for (i = 0; i < CIO2_MAX_BUFFERS; i++) {
		/*
		 * We have allocated CIO2_MAX_BUFFERS circularly for the
		 * hw, the user has requested N buffer queue. The driver
		 * ensures N <= CIO2_MAX_BUFFERS and guarantees that whenever
		 * user queues a buffer, there necessarily is a free buffer.
		 */
		if (!q->bufs[next]) {
			q->bufs[next] = b;
			entry = &q->fbpt[next * CIO2_MAX_LOPS];
			cio2_fbpt_entry_init_buf(cio2, b, entry);
			local_irq_restore(flags);
			q->bufs_next = (next + 1) % CIO2_MAX_BUFFERS;
			for (j = 0; j < vb->num_planes; j++)
				vb2_set_plane_payload(vb, i,
					q->format.plane_fmt[j].sizeimage);
			return;
		}

		dev_dbg(&cio2->pci_dev->dev, "entry %i was full!\n", next);
		next = (next + 1) % CIO2_MAX_BUFFERS;
	}

	local_irq_restore(flags);
	dev_err(&cio2->pci_dev->dev, "error: all cio2 entries were full!\n");
	atomic_dec(&q->bufs_queued);
	vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

/* Called when each buffer is freed */
static void cio2_vb2_buf_cleanup(struct vb2_buffer *vb)
{
	struct cio2_device *cio2 = vb2_get_drv_priv(vb->vb2_queue);
	struct cio2_buffer *b =
		container_of(vb, struct cio2_buffer, vbb.vb2_buf);
	unsigned int length = vb->planes[0].length;
	int lops = DIV_ROUND_UP(DIV_ROUND_UP(length, CIO2_PAGE_SIZE),
				CIO2_PAGE_SIZE / sizeof(u32));

	/* Free LOP table */
	dma_free_coherent(&cio2->pci_dev->dev, lops * CIO2_PAGE_SIZE,
				b->lop, b->lop_bus_addr);
}

static int cio2_vb2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cio2_queue *q = container_of(vq, struct cio2_queue, vbq);
	struct cio2_device *cio2 = vb2_get_drv_priv(vq);
	int r;

	cio2->cur_queue = q;
	atomic_set(&q->frame_sequence, 0);

	r = pm_runtime_get_sync(&cio2->pci_dev->dev);
	if (r < 0) {
		dev_info(&cio2->pci_dev->dev,
			"failed to set power %d\n", r);
		pm_runtime_put(&cio2->pci_dev->dev);
		return r;
	}

	r = media_entity_pipeline_start(&q->vdev.entity, &q->pipe);
	if (r)
		goto fail_pipeline;

	r = cio2_hw_init(cio2, q);
	if (r)
		goto fail_hw;

	/* Start streaming on CSI2 receiver */
	r = v4l2_subdev_call(&q->subdev, video, s_stream, 1);
	if (r && r != -ENOIOCTLCMD)
		goto fail_csi2_subdev;

	/* Start streaming on sensor */
	r = v4l2_subdev_call(q->sensor, video, s_stream, 1);
	if (r)
		goto fail_sensor_subdev;

	cio2->streaming = true;
	return 0;

fail_sensor_subdev:
	v4l2_subdev_call(&q->subdev, video, s_stream, 0);
fail_csi2_subdev:
	cio2_hw_exit(cio2, q);
fail_hw:
	media_entity_pipeline_stop(&q->vdev.entity);
fail_pipeline:
	dev_dbg(&cio2->pci_dev->dev, "failed to start streaming (%d)\n", r);
	cio2_vb2_return_all_buffers(q, VB2_BUF_STATE_QUEUED);

	return r;
}

static void cio2_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct cio2_queue *q = container_of(vq, struct cio2_queue, vbq);
	struct cio2_device *cio2 = vb2_get_drv_priv(vq);
	int r;

	if (v4l2_subdev_call(q->sensor, video, s_stream, 0))
		dev_err(&cio2->pci_dev->dev,
			"failed to stop sensor streaming\n");

	r = v4l2_subdev_call(&q->subdev, video, s_stream, 0);
	if (r && r != -ENOIOCTLCMD)
		dev_err(&cio2->pci_dev->dev, "failed to stop CSI2 streaming\n");

	cio2_hw_exit(cio2, q);
	cio2_vb2_return_all_buffers(q, VB2_BUF_STATE_ERROR);
	media_entity_pipeline_stop(&q->vdev.entity);
	pm_runtime_put(&cio2->pci_dev->dev);
	cio2->streaming = false;
}

static const struct vb2_ops cio2_vb2_ops = {
	.buf_init = cio2_vb2_buf_init,
	.buf_queue = cio2_vb2_buf_queue,
	.buf_cleanup = cio2_vb2_buf_cleanup,
	.queue_setup = cio2_vb2_queue_setup,
	.start_streaming = cio2_vb2_start_streaming,
	.stop_streaming = cio2_vb2_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/**************** V4L2 interface ****************/

static int cio2_v4l2_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct cio2_device *cio2 = video_drvdata(file);

	strlcpy(cap->driver, CIO2_NAME, sizeof(cap->driver));
	strlcpy(cap->card, CIO2_DEVICE_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "PCI:%s", pci_name(cio2->pci_dev));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int cio2_v4l2_enum_fmt(struct file *file, void *fh,
			      struct v4l2_fmtdesc *f)
{
	if (f->index >= NUM_FORMATS)
		return -EINVAL;

	f->pixelformat = formats[f->index].fourcc;

	return 0;
}

/* Propagate forward always the format from the CIO2 subdev */
static int cio2_v4l2_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct cio2_queue *q = file_to_cio2_queue(file);

	f->fmt.pix_mp = q->format;

	return 0;
}

static int cio2_v4l2_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	const struct ipu3_cio2_fmt *fmt;
	struct v4l2_pix_format_mplane *mpix = &f->fmt.pix_mp;

	fmt = cio2_find_format(&mpix->pixelformat, NULL);
	if (!fmt)
		fmt = &formats[0];

	/* Only supports up to 4224x3136 */
	if (mpix->width > CIO2_IMAGE_MAX_WIDTH)
		mpix->width = CIO2_IMAGE_MAX_WIDTH;
	if (mpix->height > CIO2_IMAGE_MAX_LENGTH)
		mpix->height = CIO2_IMAGE_MAX_LENGTH;

	mpix->num_planes = 1;
	mpix->pixelformat = fmt->fourcc;
	mpix->colorspace = V4L2_COLORSPACE_RAW;
	mpix->field = V4L2_FIELD_NONE;
	memset(mpix->reserved, 0, sizeof(mpix->reserved));
	mpix->plane_fmt[0].bytesperline = cio2_bytesperline(mpix->width);
	mpix->plane_fmt[0].sizeimage = mpix->plane_fmt[0].bytesperline *
							mpix->height;
	memset(mpix->plane_fmt[0].reserved, 0,
	       sizeof(mpix->plane_fmt[0].reserved));

	/* use default */
	mpix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mpix->quantization = V4L2_QUANTIZATION_DEFAULT;
	mpix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int cio2_v4l2_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct cio2_queue *q = file_to_cio2_queue(file);

	cio2_v4l2_try_fmt(file, fh, f);
	q->format = f->fmt.pix_mp;

	return 0;
}

static const struct v4l2_file_operations cio2_v4l2_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static const struct v4l2_ioctl_ops cio2_v4l2_ioctl_ops = {
	.vidioc_querycap = cio2_v4l2_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = cio2_v4l2_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = cio2_v4l2_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = cio2_v4l2_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane = cio2_v4l2_try_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

static int cio2_subdev_subscribe_event(struct v4l2_subdev *sd,
				       struct v4l2_fh *fh,
				       struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	/* Line number. For now only zero accepted. */
	if (sub->id != 0)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

/*
 * cio2_subdev_get_fmt - Handle get format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @cfg: V4L2 subdev pad config
 * @fmt: pointer to v4l2 subdev format structure
 * return -EINVAL or zero on success
 */
static int cio2_subdev_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct cio2_queue *q = container_of(sd, struct cio2_queue, subdev);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	else	/* Retrieve the current format */
		fmt->format = q->subdev_fmt;

	return 0;
}

/*
 * cio2_subdev_set_fmt - Handle set format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @cfg: V4L2 subdev pad config
 * @fmt: pointer to v4l2 subdev format structure
 * return -EINVAL or zero on success
 */
static int cio2_subdev_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct cio2_queue *q = container_of(sd, struct cio2_queue, subdev);

	/*
	 * Only allow setting sink pad format;
	 * source always propagates from sink
	 */
	if (fmt->pad == CIO2_PAD_SOURCE)
		return cio2_subdev_get_fmt(sd, cfg, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
	} else {
		/* It's the sink, allow changing frame size */
		q->subdev_fmt.width = fmt->format.width;
		q->subdev_fmt.height = fmt->format.height;
		q->subdev_fmt.code = fmt->format.code;
		fmt->format = q->subdev_fmt;
	}

	return 0;
}

static int cio2_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= NUM_FORMATS)
		return -EINVAL;

	code->code = formats[code->index].mbus_code;

	return 0;
}

static const struct v4l2_subdev_core_ops cio2_subdev_core_ops = {
	.subscribe_event = cio2_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops cio2_subdev_video_ops = {};

static const struct v4l2_subdev_pad_ops cio2_subdev_pad_ops = {
	.link_validate = v4l2_subdev_link_validate_default,
	.get_fmt = cio2_subdev_get_fmt,
	.set_fmt = cio2_subdev_set_fmt,
	.enum_mbus_code = cio2_subdev_enum_mbus_code,
};

static const struct v4l2_subdev_ops cio2_subdev_ops = {
	.core = &cio2_subdev_core_ops,
	.video = &cio2_subdev_video_ops,
	.pad = &cio2_subdev_pad_ops,
};

/******* V4L2 sub-device asynchronous registration callbacks***********/

struct sensor_async_subdev {
	struct v4l2_async_subdev asd;
	struct csi2_bus_info csi2;
};

/* The .bound() notifier callback when a match is found */
static int cio2_notifier_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *sd,
			       struct v4l2_async_subdev *asd)
{
	struct cio2_device *cio2 = container_of(notifier,
					struct cio2_device, notifier);
	struct sensor_async_subdev *s_asd = container_of(asd,
					struct sensor_async_subdev, asd);
	struct cio2_queue *q;

	if (cio2->queue[s_asd->csi2.port].sensor)
		return -EBUSY;

	q = &cio2->queue[s_asd->csi2.port];

	q->csi2 = s_asd->csi2;
	q->sensor = sd;
	q->csi_rx_base = cio2->base + CIO2_REG_PIPE_BASE(q->csi2.port);

	return 0;
}

/* The .unbind callback */
static void cio2_notifier_unbind(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_subdev *asd)
{
	struct cio2_device *cio2 = container_of(notifier,
						struct cio2_device, notifier);
	struct sensor_async_subdev *s_asd = container_of(asd,
					struct sensor_async_subdev, asd);

	cio2->queue[s_asd->csi2.port].sensor = NULL;
}

/* .complete() is called after all subdevices have been located */
static int cio2_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct cio2_device *cio2 = container_of(notifier, struct cio2_device,
						notifier);
	struct sensor_async_subdev *s_asd;
	struct cio2_queue *q;
	unsigned int i, pad;
	int ret;

	for (i = 0; i < notifier->num_subdevs; i++) {
		s_asd = container_of(cio2->notifier.subdevs[i],
				     struct sensor_async_subdev,
				     asd);
		q = &cio2->queue[s_asd->csi2.port];

		for (pad = 0; pad < q->sensor->entity.num_pads; pad++)
			if (q->sensor->entity.pads[pad].flags &
					MEDIA_PAD_FL_SOURCE)
			break;

		if (pad == q->sensor->entity.num_pads) {
			dev_err(&cio2->pci_dev->dev,
				"failed to find src pad for %s\n",
				q->sensor->name);

			return -ENXIO;
		}

		ret = media_entity_create_link(
				&q->sensor->entity, pad,
				&q->subdev.entity, CIO2_PAD_SINK,
				MEDIA_LNK_FL_ENABLED);
		if (ret) {
			dev_err(&cio2->pci_dev->dev,
				"failed to create link for %s\n",
				cio2->queue[i].sensor->name);

			return ret;
		}
	}

	return v4l2_device_register_subdev_nodes(&cio2->v4l2_dev);
}

static int cio2_fwnode_parse(struct device *dev,
			     struct v4l2_fwnode_endpoint *vep,
			     struct v4l2_async_subdev *asd)
{
	struct sensor_async_subdev *s_asd =
			container_of(asd, struct sensor_async_subdev, asd);

	if (vep->bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "Only CSI2 bus type is currently supported\n");
		return -EINVAL;
	}

	s_asd->csi2.port = vep->base.port;
	s_asd->csi2.lanes = vep->bus.mipi_csi2.num_data_lanes;

	return 0;
}

static const struct v4l2_async_notifier_operations cio2_async_ops = {
	.bound = cio2_notifier_bound,
	.unbind = cio2_notifier_unbind,
	.complete = cio2_notifier_complete,
};

static int cio2_notifier_init(struct cio2_device *cio2)
{
	int ret;

	ret = v4l2_async_notifier_parse_fwnode_endpoints(
		&cio2->pci_dev->dev, &cio2->notifier,
		sizeof(struct sensor_async_subdev),
		cio2_fwnode_parse);
	if (ret < 0)
		return ret;

	if (!cio2->notifier.num_subdevs)
		return -ENODEV;		/* No subdevices */

	cio2->notifier.ops = &cio2_async_ops;
	ret = v4l2_async_notifier_register(&cio2->v4l2_dev, &cio2->notifier);
	if (ret) {
		dev_err(&cio2->pci_dev->dev,
			"failed to register async notifier : %d\n", ret);
		v4l2_async_notifier_cleanup(&cio2->notifier);
	}

	return ret;
}

static void cio2_notifier_exit(struct cio2_device *cio2)
{
	v4l2_async_notifier_unregister(&cio2->notifier);
	v4l2_async_notifier_cleanup(&cio2->notifier);
}

static int cio2_link_validate(struct media_link *link)
{
	return v4l2_subdev_link_validate(link);
}

/**************** Queue initialization ****************/
static const struct media_entity_operations cio2_media_ops = {
	.link_validate = cio2_link_validate,
};

static int cio2_queue_init(struct cio2_device *cio2, struct cio2_queue *q)
{
	static const u32 default_width = 1936;
	static const u32 default_height = 1096;
	const struct ipu3_cio2_fmt dflt_fmt = formats[0];
	struct video_device *vdev = &q->vdev;
	struct vb2_queue *vbq = &q->vbq;
	struct v4l2_subdev *subdev = &q->subdev;
	struct v4l2_mbus_framefmt *fmt;
	int r;

	/* Initialize miscellaneous variables */
	mutex_init(&q->lock);

	/* Initialize formats to default values */
	fmt = &q->subdev_fmt;
	fmt->width = default_width;
	fmt->height = default_height;
	fmt->code = dflt_fmt.mbus_code;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	q->format.width = default_width;
	q->format.height = default_height;
	q->format.pixelformat = dflt_fmt.fourcc;
	q->format.colorspace = V4L2_COLORSPACE_RAW;
	q->format.field = V4L2_FIELD_NONE;
	q->format.num_planes = 1;
	q->format.plane_fmt[0].bytesperline =
				cio2_bytesperline(q->format.width);
	q->format.plane_fmt[0].sizeimage = q->format.plane_fmt[0].bytesperline *
						q->format.height;
	/* Initialize fbpt */
	r = cio2_fbpt_init(cio2, q);
	if (r)
		goto fail_fbpt;

	/* Initialize media entities */
	r = media_entity_init(&subdev->entity, CIO2_PADS, q->subdev_pads, 0);
	if (r) {
		dev_err(&cio2->pci_dev->dev,
			"failed initialize subdev media entity (%d)\n", r);
		goto fail_subdev_media_entity;
	}
	q->subdev_pads[CIO2_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
		MEDIA_PAD_FL_MUST_CONNECT;
	q->subdev_pads[CIO2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	subdev->entity.ops = &cio2_media_ops;
	r = media_entity_init(&vdev->entity, 1, &q->vdev_pad, 0);
	if (r) {
		dev_err(&cio2->pci_dev->dev,
			"failed initialize videodev media entity (%d)\n", r);
		goto fail_vdev_media_entity;
	}
	q->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev->entity.ops = &cio2_media_ops;

	/* Initialize subdev */
	v4l2_subdev_init(subdev, &cio2_subdev_ops);
	subdev->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	subdev->owner = THIS_MODULE;
	snprintf(subdev->name, sizeof(subdev->name),
		 CIO2_ENTITY_NAME " %td", q - cio2->queue);
	v4l2_set_subdevdata(subdev, cio2);
	r = v4l2_device_register_subdev(&cio2->v4l2_dev, subdev);
	if (r) {
		dev_err(&cio2->pci_dev->dev,
			"failed initialize subdev (%d)\n", r);
		goto fail_subdev;
	}

	/* Initialize vbq */
	vbq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	vbq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
	vbq->ops = &cio2_vb2_ops;
	vbq->mem_ops = &vb2_dma_sg_memops;
	vbq->buf_struct_size = sizeof(struct cio2_buffer);
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vbq->min_buffers_needed = 1;
	vbq->drv_priv = cio2;
	vbq->lock = &q->lock;
	r = vb2_queue_init(vbq);
	if (r) {
		dev_err(&cio2->pci_dev->dev,
			"failed to initialize videobuf2 queue (%d)\n", r);
		goto fail_vbq;
	}

	/* Initialize vdev */
	snprintf(vdev->name, sizeof(vdev->name),
		 "%s %td", CIO2_NAME, q - cio2->queue);
	vdev->release = video_device_release_empty;
	vdev->fops = &cio2_v4l2_fops;
	vdev->ioctl_ops = &cio2_v4l2_ioctl_ops;
	vdev->lock = &cio2->lock;
	vdev->v4l2_dev = &cio2->v4l2_dev;
	vdev->queue = &q->vbq;
	video_set_drvdata(vdev, cio2);
	r = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (r) {
		dev_err(&cio2->pci_dev->dev,
			"failed to register video device (%d)\n", r);
		goto fail_vdev;
	}

	/* Create link from CIO2 subdev to output node */
	r = media_entity_create_link(
		&subdev->entity, CIO2_PAD_SOURCE, &vdev->entity, 0,
		MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (r)
		goto fail_link;

	return 0;

fail_link:
	video_unregister_device(&q->vdev);
fail_vdev:
	vb2_queue_release(vbq);
fail_vbq:
	v4l2_device_unregister_subdev(subdev);
fail_subdev:
	media_entity_cleanup(&vdev->entity);
fail_vdev_media_entity:
	media_entity_cleanup(&subdev->entity);
fail_subdev_media_entity:
	cio2_fbpt_exit(q, &cio2->pci_dev->dev);
fail_fbpt:
	mutex_destroy(&q->lock);

	return r;
}

static void cio2_queue_exit(struct cio2_device *cio2, struct cio2_queue *q)
{
	video_unregister_device(&q->vdev);
	media_entity_cleanup(&q->vdev.entity);
	vb2_queue_release(&q->vbq);
	v4l2_device_unregister_subdev(&q->subdev);
	media_entity_cleanup(&q->subdev.entity);
	cio2_fbpt_exit(q, &cio2->pci_dev->dev);
	mutex_destroy(&q->lock);
}

static int cio2_queues_init(struct cio2_device *cio2)
{
	int i, r;

	for (i = 0; i < CIO2_QUEUES; i++) {
		r = cio2_queue_init(cio2, &cio2->queue[i]);
		if (r)
			break;
	}

	if (i == CIO2_QUEUES)
		return 0;

	for (i--; i >= 0; i--)
		cio2_queue_exit(cio2, &cio2->queue[i]);

	return r;
}

static void cio2_queues_exit(struct cio2_device *cio2)
{
	unsigned int i;

	for (i = 0; i < CIO2_QUEUES; i++) {
		cio2_queue_exit(cio2, &cio2->queue[i]);
	}
}

/**************** PCI interface ****************/

static int cio2_pci_config_setup(struct pci_dev *dev)
{
	u16 pci_command;
	int r = pci_enable_msi(dev);

	if (r) {
		dev_err(&dev->dev, "failed to enable MSI (%d)\n", r);
		return r;
	}

	pci_read_config_word(dev, PCI_COMMAND, &pci_command);
	pci_command |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER |
		PCI_COMMAND_INTX_DISABLE;
	pci_write_config_word(dev, PCI_COMMAND, pci_command);

	return 0;
}

static int cio2_pci_probe(struct pci_dev *pci_dev,
			  const struct pci_device_id *id)
{
	struct cio2_device *cio2;
	phys_addr_t phys;
	void __iomem *const *iomap;
	int r;

	cio2 = devm_kzalloc(&pci_dev->dev, sizeof(*cio2), GFP_KERNEL);
	if (!cio2)
		return -ENOMEM;
	cio2->pci_dev = pci_dev;

	r = pcim_enable_device(pci_dev);
	if (r) {
		dev_err(&pci_dev->dev, "failed to enable device (%d)\n", r);
		return r;
	}

	dev_info(&pci_dev->dev, "device 0x%x (rev: 0x%x)\n",
		 pci_dev->device, pci_dev->revision);

	phys = pci_resource_start(pci_dev, CIO2_PCI_BAR);

	r = pcim_iomap_regions(pci_dev, 1 << CIO2_PCI_BAR, pci_name(pci_dev));
	if (r) {
		dev_err(&pci_dev->dev, "failed to remap I/O memory (%d)\n", r);
		return -ENODEV;
	}

	iomap = pcim_iomap_table(pci_dev);
	if (!iomap) {
		dev_err(&pci_dev->dev, "failed to iomap table\n");
		return -ENODEV;
	}

	cio2->base = iomap[CIO2_PCI_BAR];

	pci_set_drvdata(pci_dev, cio2);

	pci_set_master(pci_dev);

	r = pci_set_dma_mask(pci_dev, CIO2_DMA_MASK);
	if (r) {
		dev_err(&pci_dev->dev, "failed to set DMA mask (%d)\n", r);
		return -ENODEV;
	}

	r = cio2_pci_config_setup(pci_dev);
	if (r)
		return -ENODEV;

	mutex_init(&cio2->lock);

	cio2->vb2_alloc_ctx = vb2_dma_sg_init_ctx(&pci_dev->dev);
	if (IS_ERR(cio2->vb2_alloc_ctx)) {
		r = PTR_ERR(cio2->vb2_alloc_ctx);
		goto fail_mutex_destroy;
	}
	cio2->media_dev.dev = &cio2->pci_dev->dev;
	strlcpy(cio2->media_dev.model, CIO2_DEVICE_NAME,
		sizeof(cio2->media_dev.model));
	snprintf(cio2->media_dev.bus_info, sizeof(cio2->media_dev.bus_info),
		 "PCI:%s", pci_name(cio2->pci_dev));
	cio2->media_dev.driver_version = LINUX_VERSION_CODE;
	cio2->media_dev.hw_revision = 0;

	r = media_device_register(&cio2->media_dev);
	if (r < 0)
		goto fail_vb2_dma_sg_cleanup_ctx;

	cio2->v4l2_dev.mdev = &cio2->media_dev;
	r = v4l2_device_register(&pci_dev->dev, &cio2->v4l2_dev);
	if (r) {
		dev_err(&pci_dev->dev,
			"failed to register V4L2 device (%d)\n", r);
		goto fail_media_device_unregister;
	}

	r = cio2_queues_init(cio2);
	if (r)
		goto fail_v4l2_device_unregister;

	r = cio2_fbpt_init_dummy(cio2);
	if (r)
		goto fail_cio2_queue_exit;

	/* Register notifier for subdevices we care */
	r = cio2_notifier_init(cio2);
	if (r)
		goto fail_cio2_fbpt_exit_dummy;

	r = devm_request_irq(&pci_dev->dev, pci_dev->irq, cio2_irq,
			     IRQF_SHARED, CIO2_NAME, cio2);
	if (r) {
		dev_err(&pci_dev->dev, "failed to request IRQ (%d)\n", r);
		goto fail;
	}

	pm_runtime_put_noidle(&pci_dev->dev);
	pm_runtime_allow(&pci_dev->dev);

	return 0;

fail:
	cio2_notifier_exit(cio2);
fail_cio2_fbpt_exit_dummy:
	cio2_fbpt_exit_dummy(cio2);
fail_cio2_queue_exit:
	cio2_queues_exit(cio2);
fail_v4l2_device_unregister:
	v4l2_device_unregister(&cio2->v4l2_dev);
fail_media_device_unregister:
	media_device_unregister(&cio2->media_dev);
fail_vb2_dma_sg_cleanup_ctx:
	vb2_dma_sg_cleanup_ctx(cio2->vb2_alloc_ctx);
	cio2->vb2_alloc_ctx = NULL;
fail_mutex_destroy:
	mutex_destroy(&cio2->lock);

	return r;
}

static void cio2_pci_remove(struct pci_dev *pci_dev)
{
	struct cio2_device *cio2 = pci_get_drvdata(pci_dev);
	unsigned int i;

	cio2_notifier_exit(cio2);
	cio2_fbpt_exit_dummy(cio2);
	for (i = 0; i < CIO2_QUEUES; i++)
		cio2_queue_exit(cio2, &cio2->queue[i]);
	v4l2_device_unregister(&cio2->v4l2_dev);
	media_device_unregister(&cio2->media_dev);
	vb2_dma_sg_cleanup_ctx(cio2->vb2_alloc_ctx);
	cio2->vb2_alloc_ctx = NULL;
	mutex_destroy(&cio2->lock);
}

static int cio2_runtime_suspend(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cio2_device *cio2 = pci_get_drvdata(pci_dev);
	void __iomem *const base = cio2->base;
	u16 pm;

	writel(CIO2_D0I3C_I3, base + CIO2_REG_D0I3C);
	dev_dbg(dev, "cio2 runtime suspend.\n");

	pci_read_config_word(pci_dev, pci_dev->pm_cap + CIO2_PMCSR_OFFSET,
				&pm);
	pm = (pm >> CIO2_PMCSR_D0D3_SHIFT) << CIO2_PMCSR_D0D3_SHIFT;
	pm |= CIO2_PMCSR_D3;
	pci_write_config_word(pci_dev, pci_dev->pm_cap + CIO2_PMCSR_OFFSET,
				pm);

	return 0;
}

static int cio2_runtime_resume(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cio2_device *cio2 = pci_get_drvdata(pci_dev);
	void __iomem *const base = cio2->base;
	u16 pm;

	writel(CIO2_D0I3C_RR, base + CIO2_REG_D0I3C);
	dev_dbg(dev, "cio2 runtime resume.\n");

	pci_read_config_word(pci_dev, pci_dev->pm_cap + CIO2_PMCSR_OFFSET,
				&pm);
	pm = (pm >> CIO2_PMCSR_D0D3_SHIFT) << CIO2_PMCSR_D0D3_SHIFT;
	pci_write_config_word(pci_dev, pci_dev->pm_cap + CIO2_PMCSR_OFFSET,
				pm);

	return 0;
}

static int cio2_suspend(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cio2_device *cio2 = pci_get_drvdata(pci_dev);
	struct cio2_queue *q = cio2->cur_queue;

	dev_dbg(dev, "cio2 suspend\n");
	if (!cio2->streaming)
		return 0;

	/* Stop stream */
	cio2_hw_exit(cio2, q);
	pm_runtime_force_suspend(dev);
	return 0;
}

static int cio2_resume(struct device *dev)
{
	struct pci_dev *pci_dev = to_pci_dev(dev);
	struct cio2_device *cio2 = pci_get_drvdata(pci_dev);
	int r = 0;
	struct cio2_queue *q = cio2->cur_queue;

	dev_dbg(dev, "cio2 resume\n");
	if (!cio2->streaming)
		return 0;
	/* Start stream */
	r = pm_runtime_force_resume(&cio2->pci_dev->dev);
	if (r < 0) {
		dev_err(&cio2->pci_dev->dev,
			"failed to set power %d\n", r);
		return r;
	}

	r = cio2_fbpt_rearrange(cio2, q);
	if (r)
		return r;

	q->bufs_first = 0;
	q->bufs_next = 0;
	r = cio2_hw_init(cio2, q);
	if (r)
		dev_err(dev, "fail to init cio2 hw\n");

	return r;
}

static const struct dev_pm_ops cio2_pm_ops = {
	SET_RUNTIME_PM_OPS(&cio2_runtime_suspend,
		&cio2_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(&cio2_suspend, &cio2_resume)
};

static const struct pci_device_id cio2_pci_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, CIO2_PCI_ID) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, cio2_pci_id_table);

static struct pci_driver cio2_pci_driver = {
	.name = CIO2_NAME,
	.id_table = cio2_pci_id_table,
	.probe = cio2_pci_probe,
	.remove = cio2_pci_remove,
	.driver = {
		.pm = &cio2_pm_ops,
	},
};

module_pci_driver(cio2_pci_driver);

MODULE_AUTHOR("Tuukka Toivonen <tuukka.toivonen@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Jian Xu Zheng <jian.xu.zheng@intel.com>");
MODULE_AUTHOR("Yuning Pu <yuning.pu@intel.com>");
MODULE_AUTHOR("Yong Zhi <yong.zhi@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IPU3 CIO2 driver");
