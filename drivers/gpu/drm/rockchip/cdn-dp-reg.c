/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/reset.h>

#include "cdn-dp-core.h"
#include "cdn-dp-reg.h"

#define CDN_DP_SPDIF_CLK		200000000
#define FW_ALIVE_TIMEOUT_US		1000000
#define MAILBOX_TIMEOUT_US		5000000

void cdn_dp_set_fw_clk(struct cdn_dp_device *dp, int clk)
{
	writel(clk / 1000000, dp->regs + SW_CLK_H);
}

void dp_clock_reset_seq(struct cdn_dp_device *dp)
{
	writel(0xfff, dp->regs + SOURCE_DPTX_CAR);
	writel(0x7, dp->regs + SOURCE_PHY_CAR);
	writel(0xf, dp->regs + SOURCE_PKT_CAR);
	writel(0xff, dp->regs + SOURCE_AIF_CAR);
	writel(0xf, dp->regs + SOURCE_CIPHER_CAR);
	writel(0x3, dp->regs + SOURCE_CRYPTO_CAR);
	writel(0, dp->regs + APB_INT_MASK);
}

static int cdn_dp_mailbox_read(struct cdn_dp_device *dp)
{
	int val, ret;

	if (!dp->fw_loaded)
		return 0;

	ret = readx_poll_timeout(readl, dp->regs + MAILBOX_EMPTY_ADDR,
				 val, !val, 1000, MAILBOX_TIMEOUT_US);
	if (ret < 0) {
		dev_err(dp->dev, "failed to read mailbox, keep alive = %x\n",
			readl(dp->regs + KEEP_ALIVE));
		return ret;
	}

	return readl(dp->regs + MAILBOX0_RD_DATA) & 0xff;
}

static int cdp_dp_mailbox_write(struct cdn_dp_device *dp, u8 val)
{
	int ret, full;

	if (!dp->fw_loaded)
		return 0;

	ret = readx_poll_timeout(readl, dp->regs + MAILBOX_FULL_ADDR,
				 full, !full, 1000, MAILBOX_TIMEOUT_US);
	if (ret < 0) {
		dev_err(dp->dev, "mailbox is full, keep alive = %x\n",
			readl(dp->regs + KEEP_ALIVE));
		return ret;
	}

	writel(val, dp->regs + MAILBOX0_WR_DATA);

	return 0;
}

static int cdn_dp_mailbox_response(struct cdn_dp_device *dp, u8 module_id,
				   u8 opcode, u8 *buff, u8 buff_size)
{
	int ret, i = 0;
	u8 header[4];

	/* read the header of the message */
	while (i < 4) {
		ret = cdn_dp_mailbox_read(dp);
		if (ret < 0)
			return ret;

		header[i++] = ret;
	}

	if (opcode != header[0] || module_id != header[1] ||
	    buff_size != ((header[2] << 8) | header[3])) {
		dev_err(dp->dev, "mailbox response failed");

		/*
		 * If the message in mailbox is not what we want, we need to
		 * clear the mailbox by read.
		 */
		i = (header[2] << 8) | header[3];
		while (i--)
			if (cdn_dp_mailbox_read(dp) < 0)
				break;

		return -EINVAL;
	}

	i = 0;
	while (i < buff_size) {
		ret = cdn_dp_mailbox_read(dp);
		if (ret < 0)
			return ret;

		buff[i++] = ret;
	}

	return 0;
}

static int cdn_dp_mailbox_send(struct cdn_dp_device *dp, u8 module_id,
			       u8 opcode, u16 size, u8 *message)
{
	u8 header[4];
	int ret, i;

	header[0] = opcode;
	header[1] = module_id;
	header[2] = size >> 8;
	header[3] = size & 0xff;

	for (i = 0; i < 4; i++) {
		ret = cdp_dp_mailbox_write(dp, header[i]);
		if (ret)
			return ret;
	}

	while (size--) {
		ret = cdp_dp_mailbox_write(dp, *message++);
		if (ret)
			return ret;
	}

	return 0;
}

static int cdn_dp_reg_write(struct cdn_dp_device *dp, u16 addr, u32 val)
{
	u8 msg[6];

	msg[0] = (addr >> 8) & 0xff;
	msg[1] = addr & 0xff;
	msg[2] = (val >> 24) & 0xff;
	msg[3] = (val >> 16) & 0xff;
	msg[4] = (val >> 8) & 0xff;
	msg[5] = val & 0xff;
	return cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_WRITE_REGISTER,
				   ARRAY_SIZE(msg), msg);
}

int cdn_dp_load_firmware(struct cdn_dp_device *dp, const u32 *i_mem,
			 u32 i_size, const u32 *d_mem, u32 d_size)
{
	int i, reg, ret;

	/* reset ucpu before load firmware*/
	writel(APB_IRAM_PATH | APB_DRAM_PATH | APB_XT_RESET,
	       dp->regs + APB_CTRL);

	for (i = 0; i < i_size; i += 4)
		writel(*i_mem++, dp->regs + ADDR_IMEM + i);

	for (i = 0; i < d_size; i += 4)
		writel(*d_mem++, dp->regs + ADDR_DMEM + i);

	/* un-reset ucpu */
	writel(0, dp->regs + APB_CTRL);

	/* check the keep alive register to make sure fw working */
	ret = readx_poll_timeout(readl, dp->regs + KEEP_ALIVE,
				 reg, reg, 2000, FW_ALIVE_TIMEOUT_US);
	if (ret < 0) {
		dev_err(dp->dev, "failed to loaded the FW reg = %x\n", reg);
		return -EINVAL;
	}

	reg = readl(dp->regs + VER_L) & 0xff;
	dp->fw_version = reg;
	reg = readl(dp->regs + VER_H) & 0xff;
	dp->fw_version |= reg << 8;
	reg = readl(dp->regs + VER_LIB_L_ADDR) & 0xff;
	dp->fw_version |= reg << 16;
	reg = readl(dp->regs + VER_LIB_H_ADDR) & 0xff;
	dp->fw_version |= reg << 24;

	dp->fw_loaded = 1;

	return 0;
}

int cdn_dp_reg_write_bit(struct cdn_dp_device *dp, u16 addr, u8 start_bit,
			 u8 bits_no, u32 val)
{
	u8 field[8];

	field[0] = (addr >> 8) & 0xff;
	field[1] = addr & 0xff;
	field[2] = start_bit;
	field[3] = bits_no;
	field[4] = (val >> 24) & 0xff;
	field[5] = (val >> 16) & 0xff;
	field[6] = (val >> 8) & 0xff;
	field[7] = val & 0xff;

	return cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_WRITE_FIELD,
			    sizeof(field), field);
}

int cdn_dp_active(struct cdn_dp_device *dp, bool enable)
{
	u8 active = enable ? 1 : 0;
	u8 resp;
	int ret;

	/* set firmware status, 1: avtive; 0: standby */
	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_GENERAL,
				  GENERAL_MAIN_CONTROL, 1, &active);
	if (ret)
		return ret;

	ret = cdn_dp_mailbox_response(dp, MB_MODULE_ID_GENERAL,
				      GENERAL_MAIN_CONTROL, &resp, 1);
	if (ret)
		return ret;

	return resp ? 0 : -EINVAL;
}

int cdn_dp_set_host_cap(struct cdn_dp_device *dp)
{
	u8 msg[8];
	int ret;

	msg[0] = DP_LINK_BW_5_4;
	msg[1] = dp->cap_lanes;
	msg[2] = VOLTAGE_LEVEL_2;
	msg[3] = PRE_EMPHASIS_LEVEL_3;
	msg[4] = PRBS7 | D10_2 | TRAINING_PTN1 | TRAINING_PTN2;
	msg[5] = FAST_LT_NOT_SUPPORT;
	msg[6] = LANE_MAPPING_NORMAL;
	msg[7] = ENHANCED;

	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX,
				  DPTX_SET_HOST_CAPABILITIES,
				  ARRAY_SIZE(msg), msg);
	if (ret)
		return ret;

	ret = cdn_dp_reg_write(dp, DP_AUX_SWAP_INVERSION_CONTROL,
			       AUX_HOST_INVERT);
	if (ret)
		return ret;

	return 0;
}

int cdn_dp_event_config(struct cdn_dp_device *dp)
{
	u8 msg[5] = {0, 0, 0, 0, 0};

	msg[0] = DPTX_EVENT_ENABLE_HPD | DPTX_EVENT_ENABLE_TRAINING;

	return cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_ENABLE_EVENT,
				    ARRAY_SIZE(msg), msg);
}

int cdn_dp_get_event(struct cdn_dp_device *dp)
{
	return readl(dp->regs + SW_EVENTS0);
}

int cdn_dp_get_hpd_status(struct cdn_dp_device *dp)
{
	u8 status;
	int ret;

	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_HPD_STATE,
				  0, NULL);
	if (ret)
		return ret;

	ret = cdn_dp_mailbox_response(dp, MB_MODULE_ID_DP_TX,
				      DPTX_HPD_STATE, &status, 1);
	if (ret)
		return ret;

	return status;
}

int cdn_dp_get_edid_block(void *data, u8 *edid,
			  unsigned int block, size_t length)
{
	struct cdn_dp_device *dp = data;
	u8 msg[2], reg[EDID_DATA + EDID_BLOCK_SIZE];
	int ret;

	if (length != EDID_BLOCK_SIZE)
		return -EINVAL;

	msg[0] = block / 2;
	msg[1] = block % 2;

	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_GET_EDID,
				  2, msg);
	if (ret)
		return ret;

	ret = cdn_dp_mailbox_response(dp, MB_MODULE_ID_DP_TX,
				      DPTX_GET_EDID, reg,
				      EDID_DATA + EDID_BLOCK_SIZE);
	if (ret)
		return ret;

	if (reg[EDID_LENGTH_BYTE] != EDID_BLOCK_SIZE ||
	    reg[EDID_SEGMENT_BUMBER] != block / 2) {
		dev_err(dp->dev, "edid block size err\n");
		return -EINVAL;
	}

	memcpy(edid, &reg[EDID_DATA], EDID_BLOCK_SIZE);

	return 0;
}

int cdn_dp_training_start(struct cdn_dp_device *dp)
{
	u8 msg, event[2];
	unsigned long timeout;
	int ret;

	msg = LINK_TRAINING_RUN;

	/* start training */
	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_TRAINING_CONTROL,
				  1, &msg);
	if (ret)
		return ret;

	/* the whole training should finish in 500ms */
	timeout = jiffies + msecs_to_jiffies(500);
	while (1) {
		msleep(20);
		ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX,
					  DPTX_READ_EVENT, 0, NULL);
		if (ret)
			return ret;

		ret = cdn_dp_mailbox_response(dp, MB_MODULE_ID_DP_TX,
					      DPTX_READ_EVENT, event, 2);
		if (ret)
			return ret;

		if (event[1] & EQ_PHASE_FINISHED)
			break;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	}

	return 0;
}

int cdn_dp_get_lt_status(struct cdn_dp_device *dp)
{
	u8 status[10];
	int ret;

	ret = cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_READ_LINK_STAT,
				  0, NULL);
	if (ret)
		return ret;

	ret = cdn_dp_mailbox_response(dp, MB_MODULE_ID_DP_TX,
				      DPTX_READ_LINK_STAT, status, 10);
	if (ret)
		return ret;

	dp->link.rate = status[0];
	dp->link.num_lanes = status[1];

	return 0;
}

int cdn_dp_set_video_status(struct cdn_dp_device *dp, int active)
{
	u8 msg;

	msg = !!active;

	return cdn_dp_mailbox_send(dp, MB_MODULE_ID_DP_TX, DPTX_SET_VIDEO,
				   1, &msg);
}

static int cdn_dp_get_msa_misc(struct video_info *video,
			       struct drm_display_mode *mode)
{
	u8 val0, val1;
	u32 msa_misc;

	switch (video->color_fmt) {
	case PXL_RGB:
	case Y_ONLY:
		val0 = 0;
		break;
	case YCBCR_4_4_4:
		val0 = 6;
		break;
	case YCBCR_4_2_2:
		val0 = 5;
		break;
	case YCBCR_4_2_0:
		val0 = 5;
		break;
	};

	switch (video->color_depth) {
	case 6:
		val1 = 0;
		break;
	case 8:
		val1 = 1;
		break;
	case 10:
		val1 = 2;
		break;
	case 12:
		val1 = 3;
		break;
	case 16:
		val1 = 4;
		break;
	};

	msa_misc = 2 * val0 + 32 * val1 +
		   ((video->color_fmt == Y_ONLY) ? (1 << 14) : 0);

	return msa_misc;
}

int cdn_dp_config_video(struct cdn_dp_device *dp)
{
	struct video_info *video = &dp->video_info;
	struct drm_display_mode *mode = &dp->mode;
	u32 val, link_rate;
	u8 bit_per_pix;
	int ret;

	bit_per_pix = (video->color_fmt == YCBCR_4_2_2) ?
		      (video->color_depth * 2) : (video->color_depth * 3);

	val = VIF_BYPASS_INTERLACE;

	ret = cdn_dp_reg_write(dp, BND_HSYNC2VSYNC, val);
	if (ret)
		return ret;

	ret = cdn_dp_reg_write(dp, HSYNC2VSYNC_POL_CTRL, 0);
	if (ret)
		return ret;

	link_rate = drm_dp_bw_code_to_link_rate(dp->link.rate) / 1000;

	val = TU_SIZE * mode->clock * bit_per_pix;
	val /= dp->link.num_lanes * link_rate * 8 * 1000;
	val += TU_SIZE << 8;

	ret = cdn_dp_reg_write(dp, DP_FRAMER_TU, val);
	if (ret)
		return ret;

	switch (video->color_depth) {
	case 6:
		val = BCS_6;
		break;
	case 8:
		val = BCS_8;
		break;
	case 10:
		val = BCS_10;
		break;
	case 12:
		val = BCS_12;
		break;
	case 16:
		val = BCS_16;
		break;
	};

	val += video->color_fmt << 8;
	ret = cdn_dp_reg_write(dp, DP_FRAMER_PXL_REPR, val);
	if (ret)
		return ret;

	val = video->h_sync_polarity ? DP_FRAMER_SP_HSP : 0;
	val |= video->v_sync_polarity ? DP_FRAMER_SP_VSP : 0;
	ret = cdn_dp_reg_write(dp, DP_FRAMER_SP, val);
	if (ret)
		return ret;

	val = (mode->hsync_start - mode->hdisplay) << 16;
	val |= mode->htotal - mode->hsync_end;
	ret = cdn_dp_reg_write(dp, DP_FRONT_BACK_PORCH, val);
	if (ret)
		return ret;

	val = mode->hdisplay * bit_per_pix / 8;
	ret = cdn_dp_reg_write(dp, DP_BYTE_COUNT, val);
	if (ret)
		return ret;

	val = mode->htotal | ((mode->htotal - mode->hsync_start) << 16);
	ret = cdn_dp_reg_write(dp, MSA_HORIZONTAL_0, val);
	if (ret)
		return ret;

	val = mode->hsync_end - mode->hsync_start;
	val |= (mode->hdisplay << 16) | (video->h_sync_polarity << 15);
	ret = cdn_dp_reg_write(dp, MSA_HORIZONTAL_1, val);
	if (ret)
		return ret;

	val = mode->vtotal;
	val |= ((mode->vtotal - mode->vsync_start) << 16);

	ret = cdn_dp_reg_write(dp, MSA_VERTICAL_0, val);
	if (ret)
		return ret;

	val = mode->vsync_end - mode->vsync_start;
	val |= mode->vdisplay << 16;
	val |= (video->v_sync_polarity << 15);
	ret = cdn_dp_reg_write(dp, MSA_VERTICAL_1, val);
	if (ret)
		return ret;

	val = cdn_dp_get_msa_misc(video, mode);
	ret = cdn_dp_reg_write(dp, MSA_MISC, val);
	if (ret)
		return ret;

	ret = cdn_dp_reg_write(dp, STREAM_CONFIG, 1);
	if (ret)
		return ret;

	val = mode->hsync_end - mode->hsync_start;
	val |= (mode->hdisplay << 16);
	ret = cdn_dp_reg_write(dp, DP_HORIZONTAL, val);
	if (ret)
		return ret;

	val = mode->vtotal;
	val -= (mode->vtotal - mode->vdisplay);
	val |= (mode->vtotal - mode->vsync_start) << 16;

	ret = cdn_dp_reg_write(dp, DP_VERTICAL_0, val);
	if (ret)
		return ret;

	val = mode->vtotal;
	ret = cdn_dp_reg_write(dp, DP_VERTICAL_1, val);
	if (ret)
		return ret;

	val =  0;
	return cdn_dp_reg_write_bit(dp, DP_VB_ID, 2, 1, val);
}

int cdn_dp_audio_stop(struct cdn_dp_device *dp, struct audio_info *audio)
{
	int ret = cdn_dp_reg_write(dp, AUDIO_PACK_CONTROL, AUDIO_PACK_EN(0));

	if (ret)
		return ret;
	writel(0x1F0707, dp->regs + SPDIF_CTRL_ADDR);
	writel(0, dp->regs + AUDIO_SRC_CNTL);
	writel(0, dp->regs + AUDIO_SRC_CNFG);
	writel(1, dp->regs + AUDIO_SRC_CNTL);
	writel(0, dp->regs + AUDIO_SRC_CNTL);

	writel(0, dp->regs + SMPL2PKT_CNTL);
	writel(1, dp->regs + SMPL2PKT_CNTL);
	writel(0, dp->regs + SMPL2PKT_CNTL);

	writel(1, dp->regs + FIFO_CNTL);
	writel(0, dp->regs + FIFO_CNTL);

	if (audio->format == AFMT_SPDIF)
		clk_disable_unprepare(dp->spdif_clk);

	return 0;
}

int cdn_dp_audio_mute(struct cdn_dp_device *dp, bool enable)
{
	return cdn_dp_reg_write_bit(dp, DP_VB_ID, 4, 1, enable);
}

int cdn_dp_audio_config_set(struct cdn_dp_device *dp, struct audio_info *audio)
{
	int lanes_param, i2s_port_en_val, val, i;
	int ret;

	if (audio->channels == 2 && dp->link.num_lanes == 1)
		lanes_param = 1;
	else if (audio->channels == 2)
		lanes_param = 3;
	else
		lanes_param = 0;

	if (audio->channels == 2)
		i2s_port_en_val = 1;
	else if (audio->channels == 4)
		i2s_port_en_val = 3;
	else
		i2s_port_en_val = 0xf;

	if (audio->format == AFMT_SPDIF) {
		reset_control_assert(dp->spdif_rst);
		reset_control_deassert(dp->spdif_rst);
	}

	ret = cdn_dp_reg_write(dp, CM_LANE_CTRL, 0x8000);
	if (ret)
		return ret;

	ret = cdn_dp_reg_write(dp, CM_CTRL, 0);
	if (ret)
		return ret;

	if (audio->format == AFMT_I2S) {
		writel(0x0, dp->regs + SPDIF_CTRL_ADDR);

		writel(SYNC_WR_TO_CH_ZERO, dp->regs + FIFO_CNTL);

		val = audio->channels - 1;
		val |= (audio->channels / 2 - 1) << 5;
		val |= BIT(8);
		val |= lanes_param << 11;
		writel(val, dp->regs + SMPL2PKT_CNFG);

		if (audio->sample_width == 16)
			val = 0;
		else if (audio->sample_width == 24)
			val = 1 << 9;
		else
			val = 2 << 9;

		val |= (audio->channels - 1) << 2;
		val |= i2s_port_en_val << 17;
		val |= 2 << 11;
		writel(val, dp->regs + AUDIO_SRC_CNFG);

		for (i = 0; i < (audio->channels + 1) / 2; i++) {
			if (audio->sample_width == 16)
				val = (0x08 << 8) | (0x08 << 20);
			else if (audio->sample_width == 24)
				val = (0x0b << 8) | (0x0b << 20);

			val |= ((2 * i) << 4) | ((2 * i + 1) << 16);
			writel(val, dp->regs + STTS_BIT_CH(i));
		}

		switch (audio->sample_rate) {
		case 32000:
			val = SAMPLING_FREQ(3) |
			      ORIGINAL_SAMP_FREQ(0xc);
			break;
		case 44100:
			val = SAMPLING_FREQ(0) |
			      ORIGINAL_SAMP_FREQ(0xf);
			break;
		case 48000:
			val = SAMPLING_FREQ(2) |
			      ORIGINAL_SAMP_FREQ(0xd);
			break;
		case 88200:
			val = SAMPLING_FREQ(8) |
			      ORIGINAL_SAMP_FREQ(0x7);
			break;
		case 96000:
			val = SAMPLING_FREQ(0xa) |
			      ORIGINAL_SAMP_FREQ(5);
			break;
		case 176400:
			val = SAMPLING_FREQ(0xc) |
			      ORIGINAL_SAMP_FREQ(3);
			break;
		case 192000:
			val = SAMPLING_FREQ(0xe) |
			      ORIGINAL_SAMP_FREQ(1);
			break;
		}
		val |= 4;
		writel(val, dp->regs + COM_CH_STTS_BITS);

		writel(2, dp->regs + SMPL2PKT_CNTL);
		writel(2, dp->regs + AUDIO_SRC_CNTL);
	} else {
		val = 0x1F0707;
		writel(val, dp->regs + SPDIF_CTRL_ADDR);

		writel(SYNC_WR_TO_CH_ZERO, dp->regs + FIFO_CNTL);

		val = 0x101 | (3 << 11);
		writel(val, dp->regs + SMPL2PKT_CNFG);
		writel(2, dp->regs + SMPL2PKT_CNTL);

		val = 0x3F0707;
		writel(val, dp->regs + SPDIF_CTRL_ADDR);

		clk_prepare_enable(dp->spdif_clk);
		clk_set_rate(dp->spdif_clk, CDN_DP_SPDIF_CLK);
	}

	return cdn_dp_reg_write(dp, AUDIO_PACK_CONTROL, AUDIO_PACK_EN(1));
}
