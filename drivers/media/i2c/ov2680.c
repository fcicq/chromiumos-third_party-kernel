/*
 * ov2680 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define CHIP_ID		0x2680

#define REG_SC_CTRL_MODE		0x0100
#define     SC_CTRL_MODE_SW_STANDBY	0x0
#define     SC_CTRL_MODE_STREAMING	BIT(0)

#define REG_NULL			0xFFFF

/*
 * Registers are not maintained when in MODULE_POWER_OFF or
 * MODULE_HW_STANDBY mode
 */
struct ov2680_mode {
	u32 width;
	u32 height;
	const struct ov2680_reg *reg_list;
};

struct ov2680_priv {
	struct i2c_client	*client;
	struct clk		*xclk;
	struct regulator	*avdd_regulator;	/* Analog power */
	struct regulator	*dovdd_regulator;	/* Digital I/O power */
				/* use internal DVDD power */
	struct gpio_desc	*reset_gpio;

	bool			is_streaming;
	struct mutex		mutex;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl_handler ctrl_handler;

	const struct ov2680_mode *cur_mode;
};
#define to_ov2680(sd) container_of(sd, struct ov2680_priv, subdev)

struct ov2680_reg {
	u16 addr;
	u8 val;
};

/* PLL settings bases on 24M xclk */
static struct ov2680_reg ov2680_globe_setting[] = {
	{0x0103, 0x01},
	{0x3002, 0x00},
	{0x3016, 0x1c},
	{0x3018, 0x44},
	{0x3020, 0x00},
	{0x3080, 0x02},
	{0x3082, 0x37},
	{0x3084, 0x09},
	{0x3085, 0x04},
	{0x3086, 0x01},
	{0x3503, 0x03},
	{0x350b, 0x36},
	{0x3600, 0xb4},
	{0x3603, 0x39},
	{0x3604, 0x24},
	{0x3605, 0x00},
	{0x3620, 0x26},
	{0x3621, 0x37},
	{0x3622, 0x04},
	{0x3628, 0x00},
	{0x3705, 0x3c},
	{0x370c, 0x50},
	{0x370d, 0xc0},
	{0x3718, 0x88},
	{0x3720, 0x00},
	{0x3721, 0x00},
	{0x3722, 0x00},
	{0x3723, 0x00},
	{0x3738, 0x00},
	{0x3717, 0x58},
	{0x3781, 0x80},
	{0x3789, 0x60},
	{0x3800, 0x00},
	{0x3819, 0x04},
	{0x4000, 0x81},
	{0x4001, 0x40},
	{0x4602, 0x02},
	{0x481f, 0x36},
	{0x4825, 0x36},
	{0x4837, 0x18},
	{0x5002, 0x30},
	{0x5004, 0x04},
	{0x5005, 0x00},
	{0x5006, 0x04},
	{0x5007, 0x00},
	{0x5008, 0x04},
	{0x5009, 0x00},
	{0x5080, 0x00},
	{0x5781, 0x0f},
	{0x5782, 0x04},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x00},
	{0x578a, 0x01},
	{0x578b, 0x02},
	{0x578c, 0x03},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x04},
	{0x5792, 0x00},
	{0x5793, 0x00},
	{0x5794, 0x03},
	{0x0100, 0x00},
	{REG_NULL, 0x00}
};

static struct ov2680_reg ov2680_1600x1200_regs[] = {
	{0x3086, 0x00},
	{0x3501, 0x48},
	{0x3502, 0xe0},
	{0x370a, 0x21},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x06},
	{0x3805, 0x4f},
	{0x3806, 0x04},
	{0x3807, 0xbf},
	{0x3808, 0x06},
	{0x3809, 0x40},
	{0x380a, 0x04},
	{0x380b, 0xb0},
	{0x380c, 0x06},
	{0x380d, 0xa8},
	{0x380e, 0x05},
	{0x380f, 0x0e},
	{0x3810, 0x00},
	{0x3811, 0x00},
	{0x3812, 0x00},
	{0x3813, 0x00},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x4008, 0x00},
	{0x4009, 0x0b},
	{0x5081, 0x01},
	{0x5704, 0x06},
	{0x5705, 0x50},
	{0x5706, 0x04},
	{0x5707, 0xcc},
	{0x3820, 0xc0},
	{0x3821, 0x00},
	{REG_NULL, 0x00}
};

static struct ov2680_reg ov2680_1600x900_regs[] = {
	{0x3086, 0x00},
	{0x3501, 0x48},
	{0x3502, 0xe0},
	{0x370a, 0x21},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x96},
	{0x3804, 0x06},
	{0x3805, 0x4f},
	{0x3806, 0x04},
	{0x3807, 0x39},
	{0x3808, 0x06},
	{0x3809, 0x40},
	{0x380a, 0x03},
	{0x380b, 0x84},
	{0x380c, 0x06},
	{0x380d, 0xa8},
	{0x380e, 0x05},
	{0x380f, 0x0e},
	{0x3810, 0x00},
	{0x3811, 0x00},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{0x5081, 0x41},
	{0x5704, 0x06},
	{0x5705, 0x50},
	{0x5706, 0x03},
	{0x5707, 0x94},
	{0x3820, 0xc0},
	{0x3821, 0x00},
	{REG_NULL, 0x00}
};

static struct ov2680_reg ov2680_1280x960_regs[] = {
	{0x3086, 0x00},
	{0x3501, 0x48},
	{0x3502, 0xe0},
	{0x370a, 0x21},
	{0x3801, 0xa0},
	{0x3802, 0x00},
	{0x3803, 0x78},
	{0x3804, 0x05},
	{0x3805, 0xbf},
	{0x3806, 0x04},
	{0x3807, 0x57},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x03},
	{0x380b, 0xc0},
	{0x380c, 0x06},
	{0x380d, 0xa8},
	{0x380e, 0x05},
	{0x380f, 0x0e},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{0x5081, 0x41},
	{0x5704, 0x10},
	{0x5705, 0xa0},
	{0x5706, 0x0c},
	{0x5707, 0x78},
	{0x3820, 0xc0},
	{0x3821, 0x00},
	{REG_NULL, 0x00}
};

static struct ov2680_reg ov2680_1280x720_regs[] = {
	{0x3086, 0x00},
	{0x3501, 0x48},
	{0x3502, 0xe0},
	{0x370a, 0x21},
	{0x3801, 0xa0},
	{0x3802, 0x00},
	{0x3803, 0xf2},
	{0x3804, 0x05},
	{0x3805, 0xbf},
	{0x3806, 0x03},
	{0x3807, 0xdd},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x06},
	{0x380d, 0xa8},
	{0x380e, 0x05},
	{0x380f, 0x0e},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{0x5081, 0x41},
	{0x5704, 0x10},
	{0x5705, 0xa0},
	{0x5706, 0x0c},
	{0x5707, 0x78},
	{0x3820, 0xc0},
	{0x3821, 0x00},
	{REG_NULL, 0x00}
};

#define OV2680_LINK_FREQ_330MHZ		330000000
static const s64 link_freq_menu_items[] = {
	OV2680_LINK_FREQ_330MHZ
};

static const struct ov2680_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.reg_list = ov2680_1600x1200_regs,
	},
	{
		.width = 1600,
		.height = 900,
		.reg_list = ov2680_1600x900_regs,
	},
	{
		.width = 1280,
		.height = 960,
		.reg_list = ov2680_1280x960_regs,
	},
	{
		.width = 1280,
		.height = 720,
		.reg_list = ov2680_1280x720_regs,
	},
};

static int ov2680_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"Write reg(0x%x val:0x%x) failed!\n", reg, val);

	return ret;
}

static int ov2680_write_array(struct i2c_client *client,
			      const struct ov2680_reg *regs)
{
	int i, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov2680_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

static int ov2680_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev, "Read reg 0x%x failed!\n", reg);

	return ret;
}

static int ov2680_poweron(struct ov2680_priv *priv)
{
	int ret;
	struct device *dev = &priv->client->dev;

	ret = clk_prepare_enable(priv->xclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xclk\n");
		return ret;
	}
	clk_set_rate(priv->xclk, 24000000);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	/* AVDD and DOVDD may rise in any order */
	ret = regulator_enable(priv->avdd_regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to enable AVDD regulator\n");
		goto disable_xclk;
	}
	ret = regulator_enable(priv->dovdd_regulator);
	if (ret < 0) {
		dev_err(dev, "Failed to enable DOVDD regulator\n");
		goto disable_avdd;
	}
	/* The minimum delay between AVDD and reset rising can be 0 */
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	/* 8192 xclk cycles prior to the first SCCB transaction.
	 * NOTE: An additional 1ms must be added to wait for
	 *       SCCB to become stable when using internal DVDD.
	 */
	usleep_range(1350, 1500);

	return 0;

disable_avdd:
	regulator_disable(priv->avdd_regulator);
disable_xclk:
	clk_disable_unprepare(priv->xclk);

	return ret;
}

static void ov2680_poweroff(struct ov2680_priv *priv)
{
	/* 512 xclk cycles after the last SCCB transaction or MIPI frame end */
	usleep_range(30, 50);
	clk_disable_unprepare(priv->xclk);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	regulator_disable(priv->dovdd_regulator);
	regulator_disable(priv->avdd_regulator);
}

static int ov2680_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov2680_priv *priv = to_ov2680(sd);
	int ret = 0;

	mutex_lock(&priv->mutex);

	if (priv->is_streaming && on)
		goto unlock_and_return;

	if (on)
		ret = ov2680_poweron(priv);
	else
		ov2680_poweroff(priv);

unlock_and_return:
	mutex_unlock(&priv->mutex);

	return ret;
}

static int ov2680_get_reso_dist(const struct ov2680_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov2680_mode *ov2680_find_best_fit(
					struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov2680_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov2680_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2680_priv *priv = to_ov2680(sd);
	const struct ov2680_mode *mode;

	mutex_lock(&priv->mutex);

	mode = ov2680_find_best_fit(fmt);
	priv->cur_mode = mode;

	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	mutex_unlock(&priv->mutex);

	return 0;
}

static int ov2680_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2680_priv *priv = to_ov2680(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mutex_lock(&priv->mutex);

	mf->width = priv->cur_mode->width;
	mf->height = priv->cur_mode->height;
	mf->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	mf->field = V4L2_FIELD_NONE;

	mutex_unlock(&priv->mutex);

	return 0;
}

static int ov2680_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ov2680_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fse->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	fse->min_width  = supported_modes[index].width;
	fse->max_width  = supported_modes[index].width;
	fse->max_height = supported_modes[index].height;
	fse->min_height = supported_modes[index].height;

	return 0;
}

static int ov2680_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov2680_priv *priv = to_ov2680(sd);
	struct i2c_client *client = priv->client;
	int ret = 0;

	mutex_lock(&priv->mutex);

	if (on == priv->is_streaming)
		goto unlock_and_return;

	if (on) {
		ret = ov2680_write_array(client, ov2680_globe_setting);
		if (ret)
			goto unlock_and_return;
		/* 0.2ms is the max PLL startup/lock time */
		usleep_range(200, 250);
		ret = ov2680_write_array(client, priv->cur_mode->reg_list);
		if (ret)
			goto unlock_and_return;
		ret = ov2680_write_reg(client, REG_SC_CTRL_MODE,
				       SC_CTRL_MODE_STREAMING);
		if (ret)
			goto unlock_and_return;
	} else {
		ret = ov2680_write_reg(client, REG_SC_CTRL_MODE,
				       SC_CTRL_MODE_SW_STANDBY);
		if (ret)
			goto unlock_and_return;
	}

	priv->is_streaming = on;

unlock_and_return:
	mutex_unlock(&priv->mutex);
	return ret;
}

static int ov2680_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov2680_priv *priv = to_ov2680(sd);
	struct v4l2_mbus_framefmt *try_fmt;

	mutex_lock(&priv->mutex);

	try_fmt = v4l2_subdev_get_try_format(sd, fh->pad, 0);
	/* Initialize try_fmt */
	try_fmt->width = priv->cur_mode->width;
	try_fmt->height = priv->cur_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&priv->mutex);

	return 0;
}

static struct v4l2_subdev_core_ops ov2680_core_ops = {
	.s_power = ov2680_s_power,
};

static struct v4l2_subdev_video_ops ov2680_video_ops = {
	.s_stream = ov2680_s_stream,
};

static struct v4l2_subdev_pad_ops ov2680_pad_ops = {
	.enum_mbus_code = ov2680_enum_mbus_code,
	.enum_frame_size = ov2680_enum_frame_sizes,
	.get_fmt = ov2680_get_fmt,
	.set_fmt = ov2680_set_fmt,
};

static struct v4l2_subdev_ops ov2680_subdev_ops = {
	.core	= &ov2680_core_ops,
	.video	= &ov2680_video_ops,
	.pad	= &ov2680_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov2680_internal_ops = {
	.open = ov2680_open,
};

static int ov2680_initialize_controls(struct ov2680_priv *priv)
{
	struct v4l2_ctrl_handler *handler;
	int ret;

	handler = &priv->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 1);
	if (ret)
		return ret;
	handler->lock = &priv->mutex;

	priv->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						 V4L2_CID_LINK_FREQ,
						 0, 0, link_freq_menu_items);
	if (priv->link_freq)
		priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (handler->error) {
		v4l2_ctrl_handler_free(handler);
		return handler->error;
	}

	priv->subdev.ctrl_handler = handler;

	return 0;
}

static int ov2680_check_sensor_id(struct ov2680_priv *priv,
			      struct i2c_client *client)
{
	struct device *dev = &priv->client->dev;
	u8 id_msb, id_lsb;
	int ret;

	ret = ov2680_poweron(priv);
	if (ret)
		return ret;
	ov2680_read_reg(client, 0x300a, &id_msb);
	ov2680_read_reg(client, 0x300b, &id_lsb);
	ov2680_poweroff(priv);

	if (CHIP_ID != (id_msb << 8 | id_lsb)) {
		dev_err(dev, "Wrong camera sensor id(%02x%02x)\n",
			id_msb, id_lsb);
		return -EINVAL;
	}

	dev_info(dev, "Detected OV%04x sensor\n", CHIP_ID);

	return 0;
}

static int ov2680_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ov2680_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->cur_mode = &supported_modes[0];

	priv->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(priv->xclk)) {
		dev_err(dev, "Failed to get xclk\n");
		return -EINVAL;
	}

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio)) {
		dev_err(dev, "Failed to get reset-gpios\n");
		return -EINVAL;
	}

	priv->avdd_regulator = devm_regulator_get(dev, "avdd");
	if (IS_ERR(priv->avdd_regulator)) {
		dev_err(dev, "Failed to get avdd-supply\n");
		return -EINVAL;
	}
	priv->dovdd_regulator = devm_regulator_get(dev, "dovdd");
	if (IS_ERR(priv->dovdd_regulator)) {
		dev_err(dev, "Failed to get dovdd-supply\n");
		return -EINVAL;
	}

	ret = ov2680_check_sensor_id(priv, client);
	if (ret)
		return ret;

	mutex_init(&priv->mutex);
	v4l2_i2c_subdev_init(&priv->subdev, client, &ov2680_subdev_ops);
	ret = ov2680_initialize_controls(priv);
	if (ret)
		goto destroy_mutex;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev.internal_ops = &ov2680_internal_ops;
	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&priv->subdev.entity, 1, &priv->pad, 0);
	if (ret < 0)
		goto free_ctrl_handler;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto clean_entity;
	}

	return 0;

clean_entity:
	media_entity_cleanup(&priv->subdev.entity);
free_ctrl_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
destroy_mutex:
	mutex_destroy(&priv->mutex);

	return ret;
}

static int ov2680_remove(struct i2c_client *client)
{
	struct ov2680_priv *priv = i2c_get_clientdata(client);

	ov2680_poweroff(priv);
	v4l2_async_unregister_subdev(&priv->subdev);
	media_entity_cleanup(&priv->subdev.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	mutex_destroy(&priv->mutex);

	return 0;
}

static const struct of_device_id ov2680_of_match[] = {
	{ .compatible = "ovti,ov2680" },
	{},
};

static struct i2c_driver ov2680_i2c_driver = {
	.driver = {
		.name = "ov2680",
		.owner = THIS_MODULE,
		.of_match_table = ov2680_of_match
	},
	.probe		= &ov2680_probe,
	.remove		= &ov2680_remove,
};

module_i2c_driver(ov2680_i2c_driver);

MODULE_DESCRIPTION("OmniVision ov2680 sensor driver");
MODULE_LICENSE("GPL v2");
