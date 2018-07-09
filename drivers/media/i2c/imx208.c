/*
 * Copyright (c) 2018 Intel Corporation.
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

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <asm/unaligned.h>

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN V4L2_CID_GAIN
#endif

#define IMX208_REG_VALUE_08BIT		1
#define IMX208_REG_VALUE_16BIT		2
#define IMX208_REG_VALUE_24BIT		3

#define IMX208_REG_MODE_SELECT		0x0100
#define IMX208_MODE_STANDBY		0x00
#define IMX208_MODE_STREAMING		0x01

/* Chip ID */
#define IMX208_REG_CHIP_ID		0x0000
#define IMX208_CHIP_ID			0x0208

/* V_TIMING internal */
#define IMX208_REG_VTS			0x0340
#define IMX208_VTS_60FPS		0x0472
//#define IMX208_VTS_30FPS		0x0472
#define IMX208_VTS_BINNING	0x0239
#define IMX208_VTS_60FPS_MIN		0x0458
//#define IMX208_VTS_30FPS_MIN		0x0458
#define IMX208_VTS_BINNING_MIN	0x0230
#define IMX208_VTS_MAX			0xffff

/* HBLANK control - read only */
#define IMX208_PPL_384MHZ		1124
//#define IMX208_PPL_192MHZ		2248
#define IMX208_PPL_96MHZ		1124
//#define IMX208_PPL_48MHZ		2248

/* Exposure control */
#define IMX208_REG_EXPOSURE		0x0202
#define IMX208_EXPOSURE_MIN		4
#define IMX208_EXPOSURE_STEP		1
#define IMX208_EXPOSURE_DEFAULT		0x190
#define IMX208_EXPOSURE_MAX		65535

/* Analog gain control */
#define IMX208_REG_ANALOG_GAIN		0x0204
#define IMX208_ANA_GAIN_MIN		0
#define IMX208_ANA_GAIN_MAX		0x00e0
#define IMX208_ANA_GAIN_STEP		1
#define IMX208_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX208_REG_GR_DIGITAL_GAIN	0x020e
#define IMX208_REG_R_DIGITAL_GAIN	0x0210
#define IMX208_REG_B_DIGITAL_GAIN	0x0212
#define IMX208_REG_GB_DIGITAL_GAIN	0x0214
#define IMX208_DGTL_GAIN_MIN		0
#define IMX208_DGTL_GAIN_MAX		4096   /* Max = 0xFFF */
#define IMX208_DGTL_GAIN_DEFAULT	0x100
#define IMX208_DGTL_GAIN_STEP           1

/* Orientation */
#define REG_MIRROR_FLIP_CONTROL	0x0101
#define REG_CONFIG_MIRROR_FLIP		0x03

#define IMX208_REG_TEST_PATTERN_MODE	0x0600

struct imx208_reg {
	u16 address;
	u8 val;
};

struct imx208_reg_list {
	u32 num_of_regs;
	const struct imx208_reg *regs;
};

/* Link frequency config */
struct imx208_link_freq_config {
	u32 pixels_per_line;

	/* PLL registers for this link frequency */
	struct imx208_reg_list reg_list;
};

/* Mode : resolution and related config&values */
struct imx208_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;

	/* V-timing */
	u32 vts_def;
	u32 vts_min;

	/* Index of Link frequency config to be used */
	u32 link_freq_index;
	/* Default register values */
	struct imx208_reg_list reg_list;
};

static const struct imx208_reg mipi_data_rate[] = {
	// PLL Setting
	{0x0305, 0x02},       // PRE_PLL_CLK_DIV
	{0x0307, 0x50},        // PLL_MULTIPLIER
	{0x303C, 0x3C},       // PLL STABLE TIME
};

static const struct imx208_reg mode_1936x1096_60fps_regs[] = {
	// Mode Setting
	{0x0340, 0x04},       // frame_length_lines [15:8]
	{0x0341, 0x72},       // frame_length_lines [7:0]
	{0x0342, 0x04},       // line_length_pck [15:8]
	{0x0343, 0x64},       // line_length_pck [7:0]
	{0x034C, 0x07},       // x_output_size [15:8]
	{0x034D, 0x90},       // x_output_size [7:0]
	{0x034E, 0x04},       // y_output_size [15:8]
	{0x034F, 0x48},       // y_output_size [7:0]
	{0x0381, 0x01},       // x_even_inc [3:0]
	{0x0383, 0x01},       // x_odd_inc [3:0]
	{0x0385, 0x01},       // y_even_inc [3:0]
	{0x0387, 0x01},       // y_odd_inc [3:0]
	{0x3048, 0x00},       // VMODEFDS
	{0x3050, 0x01},       // OPSYCK_DIV
	{0x30D5, 0x00},       // HADDEN, HADDMODE
	{0x3301, 0x00},       // RGLANESEL
	{0x3318, 0x62},       // MIPI Global Timing Table
	// Shutter Gain Setting
	{0x0202, 0x01},       // coarse_integration_time [15:8]
	{0x0203, 0x90},       // coarse_integration_time [7:0]
	{0x0205, 0x00},       // ana_gain_global
};

static const struct imx208_reg mode_968_548_60fps_regs[] = {
	// Mode Setting
	{0x0340, 0x02},       // frame_length_lines [15:8]
	{0x0341, 0x39},       // frame_length_lines [7:0]
	{0x0342, 0x08},       // line_length_pck [15:8]
	{0x0343, 0xC8},       // line_length_pck [7:0]
	{0x034C, 0x03},       // x_output_size [15:8]
	{0x034D, 0xC8},       // x_output_size [7:0]
	{0x034E, 0x02},       // y_output_size [15:8]
	{0x034F, 0x24},       // y_output_size [7:0]
	{0x0381, 0x01},       // x_even_inc [3:0]
	{0x0383, 0x03},       // x_odd_inc [3:0]
	{0x0385, 0x01},       // y_even_inc [3:0]
	{0x0387, 0x03},       // y_odd_inc [3:0]
	{0x3048, 0x01},       // VMODEFDS
	{0x3050, 0x02},       // OPSYCK_DIV
	{0x30D5, 0x03},       // HADDEN, HADDMODE
	{0x3301, 0x10},       // RGLANESEL
	{0x3318, 0x75},       // MIPI Global Timing Table
	// Shutter Gain Setting
	{0x0202, 0x01},       // coarse_integration_time [15:8]
	{0x0203, 0x90},       // coarse_integration_time [7:0]
	{0x0205, 0x00},       // ana_gain_global
};

static const char * const imx208_test_pattern_menu[] = {
	"Disabled",
	"Solid Color",
	"100% Color Bar",
	"Fade to grey Color Bar",
	"PN9",
	"Fixed Pattern1",
	"Fixed Pattern2",
	"Fixed Pattern3",
	"Fixed Pattern4",
	"Fixed Pattern5",
	"Fixed Pattern6"
};

/* Configurations for supported link frequencies */
#define IMX208_LINK_FREQ_384MHZ	384000000ULL
#define IMX208_LINK_FREQ_192MHZ	192000000ULL
#define IMX208_LINK_FREQ_96MHZ  96000000ULL
#define IMX208_LINK_FREQ_48MHZ  48000000ULL

enum {
	IMX208_LINK_FREQ_384MHZ_INDEX,
	IMX208_LINK_FREQ_96MHZ_INDEX,
};

/*
 * pixel_rate = link_freq * data-rate * nr_of_lanes / bits_per_sample
 * data rate => double data rate; number of lanes => 2; bits per pixel => 10
 */
static u64 link_freq_to_pixel_rate(u64 f)
{
	f *= 2 * 2;
	do_div(f, 10);

	return f;
}

/* Menu items for LINK_FREQ V4L2 control */
static const s64 link_freq_menu_items[] = {
	IMX208_LINK_FREQ_384MHZ,
	IMX208_LINK_FREQ_96MHZ,
};

/* Link frequency configs */
static const struct imx208_link_freq_config link_freq_configs[] = {
	{
		.pixels_per_line = IMX208_PPL_384MHZ,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mipi_data_rate),
			.regs = mipi_data_rate,
		}
	},
	{
		.pixels_per_line = IMX208_PPL_96MHZ,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mipi_data_rate),
			.regs = mipi_data_rate,
		}
	},
};

/* Mode configs */
static const struct imx208_mode supported_modes[] = {
	{
		.width = 1936,
		.height = 1096,
		.vts_def = IMX208_VTS_60FPS,
		.vts_min = IMX208_VTS_60FPS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1936x1096_60fps_regs),
			.regs = mode_1936x1096_60fps_regs,
		},
		.link_freq_index = 0,
	},
	{
		.width = 968,
		.height = 548,
		.vts_def = IMX208_VTS_BINNING,
		.vts_min = IMX208_VTS_BINNING_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_968_548_60fps_regs),
			.regs = mode_968_548_60fps_regs,
		},
		.link_freq_index = IMX208_LINK_FREQ_96MHZ_INDEX,
	},
};

struct imx208 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	/* Current mode */
	const struct imx208_mode *cur_mode;

	/* Mutex for serialized access */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct imx208 *to_imx208(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx208, sd);
}

/* Read registers up to 2 at a time */
static int imx208_read_reg(struct imx208 *imx208, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 4 at a time */
static int imx208_write_reg(struct imx208 *imx208, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx208_write_regs(struct imx208 *imx208,
			      const struct imx208_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx208_write_reg(imx208, regs[i].address, 1,
					regs[i].val);
		if (ret) {
			dev_err_ratelimited(
				&client->dev,
				"Failed to write reg 0x%4.4x. error = %d\n",
				regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Open sub-device */
static int imx208_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	return 0;
}


static int imx208_update_digital_gain(struct imx208 *imx208, u32 len, u32 val)
{
	int ret;

	ret = imx208_write_reg(imx208, IMX208_REG_GR_DIGITAL_GAIN,
				IMX208_REG_VALUE_16BIT,
				val);
	if (ret)
		return ret;
	ret = imx208_write_reg(imx208, IMX208_REG_GB_DIGITAL_GAIN,
				IMX208_REG_VALUE_16BIT,
				val);
	if (ret)
		return ret;
	ret = imx208_write_reg(imx208, IMX208_REG_R_DIGITAL_GAIN,
				IMX208_REG_VALUE_16BIT,
				val);
	if (ret)
		return ret;
	ret = imx208_write_reg(imx208, IMX208_REG_B_DIGITAL_GAIN,
				IMX208_REG_VALUE_16BIT,
				val);
	if (ret)
		return ret;
	return 0;
}

static int imx208_enable_test_pattern(struct imx208 *imx208, u32 pattern)
{
	u32 val;

	val = (pattern <= 4) ? pattern : pattern + 0xFB;

	return imx208_write_reg(imx208, IMX208_REG_TEST_PATTERN_MODE,
				IMX208_REG_VALUE_16BIT, val);
}

static int imx208_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx208 *imx208 =
		container_of(ctrl->handler, struct imx208, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	int ret = 0;
	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx208_write_reg(imx208, IMX208_REG_ANALOG_GAIN,
				IMX208_REG_VALUE_16BIT,
				ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx208_write_reg(imx208, IMX208_REG_EXPOSURE,
				IMX208_REG_VALUE_16BIT,
				ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx208_update_digital_gain(imx208, IMX208_REG_VALUE_16BIT,
				ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* Update VTS that meets expected vertical blanking */
		ret = imx208_write_reg(imx208, IMX208_REG_VTS,
				       IMX208_REG_VALUE_16BIT,
				       imx208->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx208_enable_test_pattern(imx208, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx208_ctrl_ops = {
	.s_ctrl = imx208_set_ctrl,
};

static int imx208_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	/* Only one bayer order(GRBG) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx208_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx208_update_pad_format(const struct imx208_mode *mode,
				      struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __imx208_get_pad_format(struct imx208 *imx208,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&imx208->sd, cfg,
							  fmt->pad);
	else
		imx208_update_pad_format(imx208->cur_mode, fmt);

	return 0;
}

static int imx208_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *fmt)
{
	struct imx208 *imx208 = to_imx208(sd);
	int ret;

	mutex_lock(&imx208->mutex);
	ret = __imx208_get_pad_format(imx208, cfg, fmt);
	mutex_unlock(&imx208->mutex);

	return ret;
}

static int imx208_set_pad_format(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_format *fmt)
{
	struct imx208 *imx208 = to_imx208(sd);
	const struct imx208_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	s32 vblank_def;
	s32 vblank_min;
	s64 h_blank;
	s64 pixel_rate;
	s64 link_freq;

	mutex_lock(&imx208->mutex);

	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;

	mode = v4l2_find_nearest_size(
		supported_modes, ARRAY_SIZE(supported_modes), width, height,
		fmt->format.width, fmt->format.height);
	imx208_update_pad_format(mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx208->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx208->link_freq, mode->link_freq_index);

		link_freq = link_freq_menu_items[mode->link_freq_index];
		pixel_rate = link_freq_to_pixel_rate(link_freq);
		__v4l2_ctrl_s_ctrl_int64(imx208->pixel_rate, pixel_rate);
		/* Update limits and set FPS to default */
		vblank_def = imx208->cur_mode->vts_def -
			     imx208->cur_mode->height;
		vblank_min = imx208->cur_mode->vts_min -
			     imx208->cur_mode->height;
		__v4l2_ctrl_modify_range(
			imx208->vblank, vblank_min,
			IMX208_VTS_MAX - imx208->cur_mode->height, 1,
			vblank_def);
		__v4l2_ctrl_s_ctrl(imx208->vblank, vblank_def);
		h_blank =
			link_freq_configs[mode->link_freq_index].pixels_per_line
			 - imx208->cur_mode->width;
		__v4l2_ctrl_modify_range(imx208->hblank, h_blank,
					 h_blank, 1, h_blank);
	}

	mutex_unlock(&imx208->mutex);

	return 0;
}

/* Start streaming */
static int imx208_start_streaming(struct imx208 *imx208)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	const struct imx208_reg_list *reg_list;
	int ret, link_freq_index;

	/* Setup PLL */
	link_freq_index = imx208->cur_mode->link_freq_index;
	reg_list = &link_freq_configs[link_freq_index].reg_list;
	ret = imx208_write_regs(imx208, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set plls\n", __func__);
		return ret;
	}

	/* Apply default values of current mode */
	reg_list = &imx208->cur_mode->reg_list;
	ret = imx208_write_regs(imx208, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}
#if 0
	/* Set Orientation be 180 degree */
	ret = imx208_write_reg(imx208, REG_MIRROR_FLIP_CONTROL,
				IMX208_REG_VALUE_08BIT, REG_CONFIG_MIRROR_FLIP);
	if (ret) {
		dev_err(&client->dev, "%s failed to set orientation\n",
			__func__);
		return ret;
	}
#endif
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx208->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return imx208_write_reg(imx208, IMX208_REG_MODE_SELECT,
				 IMX208_REG_VALUE_08BIT,
				 IMX208_MODE_STREAMING);
}

/* Stop streaming */
static int imx208_stop_streaming(struct imx208 *imx208)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	int ret;

	/* set stream off register */
	ret = imx208_write_reg(imx208, IMX208_REG_MODE_SELECT,
		IMX208_REG_VALUE_08BIT, IMX208_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	/* Return success even if it was an error, as there is nothing the
	 * caller can do about it.
	 */
	return 0;
}

static int imx208_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx208 *imx208 = to_imx208(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx208->mutex);
	if (imx208->streaming == enable) {
		mutex_unlock(&imx208->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx208_start_streaming(imx208);
		if (ret)
			goto err_rpm_put;
	} else {
		imx208_stop_streaming(imx208);
		pm_runtime_put(&client->dev);
	}

	imx208->streaming = enable;
	mutex_unlock(&imx208->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx208->mutex);

	return ret;
}

static int __maybe_unused imx208_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx208 *imx208 = to_imx208(sd);

	if (imx208->streaming)
		imx208_stop_streaming(imx208);

	return 0;
}

static int __maybe_unused imx208_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx208 *imx208 = to_imx208(sd);
	int ret;

	if (imx208->streaming) {
		ret = imx208_start_streaming(imx208);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx208_stop_streaming(imx208);
	imx208->streaming = 0;
	return ret;
}

/* Verify chip ID */
static int imx208_identify_module(struct imx208 *imx208)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	int ret;
	u32 val;

	ret = imx208_read_reg(imx208, IMX208_REG_CHIP_ID,
			       IMX208_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			IMX208_CHIP_ID);
		return ret;
	}

	if (val != IMX208_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX208_CHIP_ID, val);
		return -EIO;
	}

	dev_err(&client->dev, "chip id!: %x %x\n", IMX208_CHIP_ID, val);

	return 0;
}

static const struct v4l2_subdev_video_ops imx208_video_ops = {
	.s_stream = imx208_set_stream,
};

static const struct v4l2_subdev_pad_ops imx208_pad_ops = {
	.enum_mbus_code = imx208_enum_mbus_code,
	.get_fmt = imx208_get_pad_format,
	.set_fmt = imx208_set_pad_format,
	.enum_frame_size = imx208_enum_frame_size,
};

static const struct v4l2_subdev_ops imx208_subdev_ops = {
	.video = &imx208_video_ops,
	.pad = &imx208_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx208_internal_ops = {
	.open = imx208_open,
};

/* Initialize control handlers */
static int imx208_init_controls(struct imx208 *imx208)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx208->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max;
	s64 vblank_def;
	s64 vblank_min;
	s64 pixel_rate_min;
	s64 pixel_rate_max;
	int ret;

	ctrl_hdlr = &imx208->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	mutex_init(&imx208->mutex);
	ctrl_hdlr->lock = &imx208->mutex;
	imx208->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
				&imx208_ctrl_ops,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_menu_items) - 1,
				0,
				link_freq_menu_items);

	if (imx208->link_freq)
		imx208->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate_max = link_freq_to_pixel_rate(link_freq_menu_items[0]);
	pixel_rate_min = link_freq_to_pixel_rate(link_freq_menu_items[1]);
	/* By default, PIXEL_RATE is read only */
	imx208->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx208_ctrl_ops,
					V4L2_CID_PIXEL_RATE,
					pixel_rate_min, pixel_rate_max,
					1, pixel_rate_max);


	vblank_def = imx208->cur_mode->vts_def - imx208->cur_mode->height;
	vblank_min = imx208->cur_mode->vts_min - imx208->cur_mode->height;
	imx208->vblank = v4l2_ctrl_new_std(
				ctrl_hdlr, &imx208_ctrl_ops, V4L2_CID_VBLANK,
				vblank_min,
				IMX208_VTS_MAX - imx208->cur_mode->height, 1,
				vblank_def);

	imx208->hblank = v4l2_ctrl_new_std(
				ctrl_hdlr, &imx208_ctrl_ops, V4L2_CID_HBLANK,
				IMX208_PPL_384MHZ - imx208->cur_mode->width,
				IMX208_PPL_384MHZ - imx208->cur_mode->width,
				1,
				IMX208_PPL_384MHZ - imx208->cur_mode->width);

	if (imx208->hblank)
		imx208->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	exposure_max = imx208->cur_mode->vts_def - 8;
	imx208->exposure = v4l2_ctrl_new_std(
				ctrl_hdlr, &imx208_ctrl_ops,
				V4L2_CID_EXPOSURE, IMX208_EXPOSURE_MIN,
				IMX208_EXPOSURE_MAX, IMX208_EXPOSURE_STEP,
				IMX208_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx208_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
				IMX208_ANA_GAIN_MIN, IMX208_ANA_GAIN_MAX,
				IMX208_ANA_GAIN_STEP, IMX208_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx208_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
				IMX208_DGTL_GAIN_MIN, IMX208_DGTL_GAIN_MAX,
				IMX208_DGTL_GAIN_STEP,
				IMX208_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx208_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx208_test_pattern_menu) - 1,
				     0, 0, imx208_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	imx208->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx208->mutex);

	return ret;
}

static void imx208_free_controls(struct imx208 *imx208)
{
	v4l2_ctrl_handler_free(imx208->sd.ctrl_handler);
	mutex_destroy(&imx208->mutex);
}

static int imx208_probe(struct i2c_client *client)
{
	struct imx208 *imx208;
	int ret;
	u32 val = 0;
dev_err(&client->dev, "%s probe start!!!\n", __func__);
	device_property_read_u32(&client->dev, "clock-frequency", &val);
	if (val != 19200000)
		return -EINVAL;

	imx208 = devm_kzalloc(&client->dev, sizeof(*imx208), GFP_KERNEL);
	if (!imx208)
		return -ENOMEM;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx208->sd, client, &imx208_subdev_ops);

	/* Check module identity */
	ret = imx208_identify_module(imx208);
	if (ret)
		return ret;

	/* Set default mode to max resolution */
	imx208->cur_mode = &supported_modes[0];

	ret = imx208_init_controls(imx208);
	if (ret)
		return ret;

	/* Initialize subdev */
	imx208->sd.internal_ops = &imx208_internal_ops;
	imx208->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx208->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/* Initialize source pad */
	imx208->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&imx208->sd.entity, 1, &imx208->pad, 0);
	if (ret) {
		dev_err(&client->dev, "%s failed:%d\n", __func__, ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&imx208->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
dev_err(&client->dev, "%s probe done\n", __func__);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx208->sd.entity);

error_handler_free:
	imx208_free_controls(imx208);

	return ret;
}

static int imx208_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx208 *imx208 = to_imx208(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx208_free_controls(imx208);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct dev_pm_ops imx208_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx208_suspend, imx208_resume)
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id imx208_acpi_ids[] = {
	{ "INT3478" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(acpi, imx208_acpi_ids);
#endif

static struct i2c_driver imx208_i2c_driver = {
	.driver = {
		.name = "imx208",
		.pm = &imx208_pm_ops,
		.acpi_match_table = ACPI_PTR(imx208_acpi_ids),
	},
	.probe_new = imx208_probe,
	.remove = imx208_remove,
};

module_i2c_driver(imx208_i2c_driver);

MODULE_AUTHOR("Yeh, Andy <andy.yeh@intel.com>");
MODULE_AUTHOR("Chen, Ping-chung <ping-chung.chen@intel.com>");
MODULE_DESCRIPTION("Sony IMX208 sensor driver");
MODULE_LICENSE("GPL v2");
