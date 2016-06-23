/*
 * Copyright (C) 2016 Chris Zhong <zyw@rock-chips.com>
 * Copyright (C) 2016 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _ROCKCHIP_EDP_CORE_H
#define _ROCKCHIP_EDP_CORE_H

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>
#include "rockchip_drm_drv.h"
#include "cdn-dp-reg.h"

enum AUDIO_FORMAT {
	AFMT_I2S = 0,
	AFMT_SPDIF = 1,
	AFMT_UNUSED,
};

struct audio_info {
	enum AUDIO_FORMAT format;
	int sample_rate;
	int channels;
	int sample_width;
};

struct video_info {
	bool h_sync_polarity;
	bool v_sync_polarity;
	bool interlaced;
	int color_depth;
	enum VIC_PXL_ENCODING_FORMAT color_fmt;
};

struct cdn_firmware_header {
	u32 size_bytes; /* size of the entire header+image(s) in bytes */
	u32 header_size; /* size of just the header in bytes */
	u32 iram_size; /* size of iram */
	u32 dram_size; /* size of dram */
};

struct cdn_dp_device {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_display_mode mode;
	struct platform_device *audio_pdev;

	const struct firmware *fw;	/* cdn dp firmware */
	unsigned int fw_version;	/* cdn fw version */
	u32 fw_retry;
	bool fw_loaded;
	void __iomem *regs;
	struct regmap *grf;
	unsigned int irq;
	struct clk *core_clk;
	struct clk *pclk;
	struct clk *spdif_clk;
	struct reset_control *spdif_rst;
	struct audio_info audio_info;
	struct video_info video_info;
	struct extcon_dev *extcon;
	struct notifier_block event_nb;
	struct delayed_work event_wq;

	u8 cap_lanes;
	bool hpd_status;

	int dpms_mode;
	struct drm_dp_link link;
	bool sink_has_audio;
};

void dp_clock_reset_seq(struct cdn_dp_device *dp);

void cdn_dp_set_fw_clk(struct cdn_dp_device *dp, int clk);
int cdn_dp_load_firmware(struct cdn_dp_device *dp, const u32 *i_mem,
			 u32 i_size, const u32 *d_mem, u32 d_size);
int cdn_dp_active(struct cdn_dp_device *dp, bool enable);
int cdn_dp_set_host_cap(struct cdn_dp_device *dp);
int cdn_dp_event_config(struct cdn_dp_device *dp);
int cdn_dp_get_event(struct cdn_dp_device *dp);
int cdn_dp_get_hpd_status(struct cdn_dp_device *dp);
int cdn_dp_dpcd_write(struct cdn_dp_device *dp, u32 addr, u8 value);
int cdn_dp_dpcd_read(struct cdn_dp_device *dp, u32 addr);
int cdn_dp_get_edid_block(void *dp, u8 *edid,
			  unsigned int block, size_t length);
int cdn_dp_training_start(struct cdn_dp_device *dp);
int cdn_dp_get_lt_status(struct cdn_dp_device *dp);
int cdn_dp_set_video_status(struct cdn_dp_device *dp, int active);
int cdn_dp_config_video(struct cdn_dp_device *dp);
int cdn_dp_audio_stop(struct cdn_dp_device *dp, struct audio_info *audio);
int cdn_dp_audio_mute(struct cdn_dp_device *dp, bool enable);
int cdn_dp_audio_config_set(struct cdn_dp_device *dp, struct audio_info *audio);

#endif  /* _ROCKCHIP_EDP_CORE_H */
