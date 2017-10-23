/*
 * ov5695 driver
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

#define CHIP_ID		0x5695

#define REG_SC_CTRL_MODE		0x0100
#define     SC_CTRL_MODE_SW_STANDBY	0x0
#define     SC_CTRL_MODE_STREAMING	BIT(0)

#define REG_NULL			0xFFFF

static const struct v4l2_mbus_framefmt default_format = {
	.width		= 800,
	.height		= 600,
	.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
	.colorspace	= V4L2_COLORSPACE_JPEG,
	.field		= V4L2_FIELD_NONE,
};

/*
 * Registers are not maintained when in MODULE_POWER_OFF or
 * MODULE_HW_STANDBY mode
 */
enum ov5695_state {
	MODULE_POWER_OFF = 0,
	MODULE_SW_STANDBY,	/* state after reset, SCCB(I2C) works */
	/* TODO: there's another HW STANDBY mode, SCCB can't work */
	MODULE_STREAMING,
};

/* TODO: Need a lock to protect *priv, especially the priv->state */
struct ov5695_priv {
	struct i2c_client	*client;
	struct clk		*mclk;
	struct regulator        *avdd_regulator;
	struct regulator        *dovdd_regulator;
	struct regulator        *dvdd_regulator;

	struct gpio_desc	*reset_gpio;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*link_freq;

	enum ov5695_state	state;

	struct v4l2_mbus_framefmt	fmt;
};
#define to_ov5695(sd) container_of(sd, struct ov5695_priv, subdev)

struct regval_list{
	u16 addr;
	u8 val;
};

/* TODO: Ask vendor for regs settings */
static struct regval_list ov5695_default_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x0300, 0x04},
	{0x0301, 0x00},
	{0x0302, 0x69},
	{0x0303, 0x00},
	{0x0304, 0x00},
	{0x0305, 0x01},
	{0x0307, 0x00},
	{0x030b, 0x00},
	{0x030c, 0x00},
	{0x030d, 0x1e},
	{0x030e, 0x04},
	{0x030f, 0x03},
	{0x0312, 0x01},
	{0x3000, 0x00},
	{0x3002, 0x21},
	{0x3022, 0x51},
	{0x3106, 0x15},
	{0x3107, 0x01},
	{0x3108, 0x05},
	{0x3500, 0x00},
	{0x3501, 0x45},
	{0x3502, 0x00},
	{0x3503, 0x08},
	{0x3504, 0x03},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x350c, 0x00},
	//{0x3500, 0x00},
	//{0x3501, 0x3d},
	//{0x3502, 0x00},
	{0x350d, 0x80},
	//{0x350a, 0x00},
	//{0x350b, 0x40},
	{0x3510, 0x00},
	{0x3511, 0x02},
	{0x3512, 0x00},
	{0x3601, 0x55},
	{0x3602, 0x58},
	{0x3614, 0x30},
	{0x3615, 0x77},
	{0x3621, 0x08},
	{0x3624, 0x40},
	{0x3633, 0x0c},
	{0x3634, 0x0c},
	{0x3635, 0x0c},
	{0x3636, 0x0c},
	{0x3638, 0x00},
	{0x3639, 0x00},
	{0x363a, 0x00},
	{0x363b, 0x00},
	{0x363c, 0xff},
	{0x363d, 0xfa},
	{0x3650, 0x44},
	{0x3651, 0x44},
	{0x3652, 0x44},
	{0x3653, 0x44},
	{0x3654, 0x44},
	{0x3655, 0x44},
	{0x3656, 0x44},
	{0x3657, 0x44},
	{0x3660, 0x00},
	{0x3661, 0x00},
	{0x3662, 0x00},
	{0x366a, 0x00},
	{0x366e, 0x0c},
	{0x3673, 0x04},
	{0x3700, 0x14},
	{0x3703, 0x0c},
	{0x3715, 0x01},
	{0x3733, 0x10},
	{0x3734, 0x40},
	{0x373f, 0xa0},
	{0x3765, 0x20},
	{0x37a1, 0x1d},
	{0x37a8, 0x26},
	{0x37ab, 0x14},
	{0x37c2, 0x04},
	{0x37cb, 0x09},
	{0x37cc, 0x13},
	{0x37cd, 0x1f},
	{0x37ce, 0x1f},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xaf},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x03},
	{0x380b, 0xcc},
	{0x380c, 0x02},
	{0x380d, 0xa0},
	{0x380e, 0x08},
	{0x380f, 0xb8},
	{0x3810, 0x00},
	{0x3811, 0x06},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x03},
	{0x3815, 0x01},
	{0x3816, 0x03},
	{0x3817, 0x01},
	{0x3818, 0x00},
	{0x3819, 0x00},
	{0x381a, 0x00},
	{0x381b, 0x01},
	{0x3820, 0x8b},
	{0x3821, 0x01},
	{0x3c80, 0x08},
	{0x3c82, 0x00},
	{0x3c83, 0x00},
	{0x3c88, 0x00},
	{0x3d85, 0x14},
	{0x3f02, 0x08},
	{0x3f03, 0x10},
	{0x4008, 0x02},
	{0x4009, 0x09},
	{0x404e, 0x20},
	{0x4501, 0x00},
	{0x4502, 0x10},
	{0x4800, 0x00},
	{0x481f, 0x2a},
	{0x4837, 0x13},
	{0x5000, 0x17},
	{0x5780, 0x3e},
	{0x5781, 0x0f},
	{0x5782, 0x44},
	{0x5783, 0x02},
	{0x5784, 0x01},
	{0x5785, 0x01},
	{0x5786, 0x00},
	{0x5787, 0x04},
	{0x5788, 0x02},
	{0x5789, 0x0f},
	{0x578a, 0xfd},
	{0x578b, 0xf5},
	{0x578c, 0xf5},
	{0x578d, 0x03},
	{0x578e, 0x08},
	{0x578f, 0x0c},
	{0x5790, 0x08},
	{0x5791, 0x06},
	{0x5792, 0x00},
	{0x5793, 0x52},
	{0x5794, 0xa3},
	{0x5b00, 0x00},
	{0x5b01, 0x1c},
	{0x5b02, 0x00},
	{0x5b03, 0x7f},
	{0x5b05, 0x6c},
	{0x5e10, 0xfc},
	{0x4010, 0xf1},
	{0x3503, 0x08},
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3509, 0xf8},
	//{0x3811, 0x06},
	//{0x5b03, 0xf0},
	//{0x0100, 0x01},

	{0x5301, 0x45},
	{0x366e, 0x0c},
	{0x3800, 0x00}, // xstart = 0
	{0x3801, 0x00}, // x start
	{0x3802, 0x00}, // y start = 0
	{0x3803, 0x00}, // y start
	{0x3804, 0x0a}, // xend = 2623
	{0x3805, 0x3f}, // xend
	{0x3806, 0x07}, // yend = 1955
	{0x3807, 0xaf}, // yend
	{0x3808, 0x05}, // x output size = 1296
	{0x3809, 0x10}, // x output size
	{0x380a, 0x03}, // y output size = 972
	{0x380b, 0xcc}, // y output size
	{0x380c, 0x02},
	{0x380d, 0xa0},
	{0x380e, 0x08},
	{0x380f, 0xb8},
	{0x3811, 0x06}, // isp x win
	{0x3813, 0x06}, // isp y win
	{0x3814, 0x03}, // x inc
	{0x3816, 0x03}, // y inc
	{0x3817, 0x01}, // hsync start
	{0x3820, 0x8b}, // flip off, v bin off
	{0x3821, 0x01}, // mirror on, h bin on
	{0x4501, 0x00}, // black line number
	{0x4008, 0x02}, // blc level trigger
	//{0x350b, 0x80}, // gain = 8x
	{0x4009, 0x09}, // MIPI global timing
	//{0x0100, 0x01},

	{REG_NULL, 0x00}
};

#define OV5695_LINK_FREQ_500MHZ		500000000
static const s64 link_freq_menu_items[] = {
	OV5695_LINK_FREQ_500MHZ
};

static int write_reg(struct i2c_client *client, u16 reg, u8 val)
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

static int ov5695_write_array(struct i2c_client *client,
			      const struct regval_list *regs)
{
	int i, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

static int ov5695_read_reg(struct i2c_client *client, u16 reg, u8 *val)
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

static void ov5695_dbg_frame_count(struct ov5695_priv *priv)
{
	u8 cnt_hi, cnt_lo;
	struct i2c_client *client = priv->client;

	ov5695_read_reg(client, 0x4848, &cnt_hi);
	ov5695_read_reg(client, 0x4849, &cnt_lo);
	dev_info(&client->dev, "Frame cnt = 0x%08x\n", (cnt_hi << 8) + cnt_lo);
}

/* Power on/off, sensor */
static int __ov5695_s_power(struct ov5695_priv *priv, int on)
{
	int ret;
	struct device *dev = &priv->client->dev;

	//TODO: check the return values
	//      check the delay values
	if (on && priv->state == MODULE_POWER_OFF) {
		ret = clk_prepare_enable(priv->mclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable mclk\n");
			return ret;
		}

		/* AVDD and DOVDD may rise in any order */
		ret = regulator_enable(priv->avdd_regulator);
		if (ret < 0) {
			dev_err(dev, "Failed to enable regulator\n");
			return ret;
			dev_err(dev, "Failed to enable AVDD regulator\n");
			goto exit;
		}
		ret = regulator_enable(priv->dovdd_regulator);
		if (ret < 0) {
			dev_err(dev, "Failed to enable DOVDD regulator\n");
			goto disable_avdd;
		}
		/* DVDD must rise after AVDD and DOVDD */
		ret = regulator_enable(priv->dvdd_regulator);
		if (ret < 0) {
			dev_err(dev, "Failed to enable DVDD regulator\n");
			goto disable_dovdd;
		}

		mdelay(1);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		mdelay(1);
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		mdelay(1); /* Wait for modules settle */

		priv->state = MODULE_SW_STANDBY;
		dev_info(dev, "Camera in STANDBY MODE\n");
	} else {
		ov5695_dbg_frame_count(priv); //TODO remove

		//clk_disable_unprepare(priv->mclk); TODO debug
		regulator_disable(priv->dvdd_regulator);
		regulator_disable(priv->dovdd_regulator);
		regulator_disable(priv->avdd_regulator);

		priv->state = MODULE_POWER_OFF;
		dev_info(dev, "Camera in POWER OFF MODE\n");
	}

	return 0;

disable_dovdd:
		regulator_disable(priv->dovdd_regulator);
disable_avdd:
		regulator_disable(priv->avdd_regulator);
exit:
		return ret;
}

static int ov5695_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5695_priv *priv = to_ov5695(sd);

	return __ov5695_s_power(priv, on);
}

static int ov5695_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5695_priv *priv = to_ov5695(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	v4l2_info(sd, "TODO: size(%dx%d), fmt: %d\n",
		  mf->width, mf->height, mf->code);

	if (mf->code != priv->fmt.code) {
		v4l2_err(sd, "code %d not supported!\n", mf->code);
		return -EINVAL;
	}

	if (priv->state > MODULE_SW_STANDBY)
		return -EBUSY;

	/* TODO set format size */

	return 0;
}

static int ov5695_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5695_priv *priv = to_ov5695(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	*mf = priv->fmt;

	return 0;
}

static int ov5695_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov5695_priv *priv = to_ov5695(sd);

	/* TODO: Add more codes */
	if (code->index != 0)
		return -EINVAL;

	code->code = priv->fmt.code;

	return 0;
}

static int ov5695_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov5695_priv *priv = to_ov5695(sd);

	/* TODO: Add more framesizes */
	if (fse->index != 0)
		return -EINVAL;

	fse->code = priv->fmt.code;

	fse->min_width  = priv->fmt.width;
	fse->max_width  = priv->fmt.width;
	fse->max_height = priv->fmt.height;
	fse->min_height = priv->fmt.height;

	return 0;
}

int ov5695_enum_frame_interval(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov5695_priv *priv = to_ov5695(sd);

	/* TODO: Add more framesizes */
	if (fie->index != 0) {
		v4l2_warn(sd, "TODO: add more frames\n");
		return -EINVAL;
	}

	fie->code = priv->fmt.code;
	fie->width = priv->fmt.width;
	fie->height = priv->fmt.height;
	fie->interval.numerator = 1; //TODO
	fie->interval.denominator = 30; //TODO

	return 0;
}

static int ov5695_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov5695_priv *priv = to_ov5695(sd);
	int streaming = priv->state == MODULE_STREAMING;
	struct i2c_client *client = priv->client;
	int ret;

	if (on == streaming)
		return 0;

	if (on) {
		ret = ov5695_write_array(client, ov5695_default_regs);
		if (ret)
			return ret;
		/*
		ret = ov5695_write_array(client, ov5695_1296x972_regs);
		if (ret)
			return ret;
		*/
		ret = write_reg(client, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
		if (ret)
			return ret;

		msleep(2); /* TODO check delay */

		priv->state = MODULE_STREAMING;
		dev_info(&priv->client->dev, "Camera in STREAMING MODE\n");
	} else {
		ret = write_reg(client, REG_SC_CTRL_MODE, SC_CTRL_MODE_SW_STANDBY);
		if (ret)
			return ret;

		msleep(2); /* TODO check delay */

		priv->state = MODULE_SW_STANDBY;
	}

	return 0;
}

static struct v4l2_subdev_core_ops ov5695_core_ops = {
	.s_power = ov5695_s_power,
};

static struct v4l2_subdev_video_ops ov5695_video_ops = {
	.s_stream = ov5695_s_stream,
};

static struct v4l2_subdev_pad_ops ov5695_pad_ops = {
	.enum_mbus_code = ov5695_enum_mbus_code,
	.enum_frame_size = ov5695_enum_frame_sizes,
	.enum_frame_interval = ov5695_enum_frame_interval,
	.get_fmt = ov5695_get_fmt,
	.set_fmt = ov5695_set_fmt,
};

static struct v4l2_subdev_ops ov5695_subdev_ops = {
	.core	= &ov5695_core_ops,
	.video	= &ov5695_video_ops,
	.pad	= &ov5695_pad_ops,
};

static int ov5695_initialize_controls(struct ov5695_priv *priv)
{
	struct v4l2_ctrl_handler *handler;
	int ret;

	handler = &priv->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 1);
	if (ret)
		return ret;

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

static int ov5695_check_sensor_id(struct ov5695_priv *priv,
				  struct i2c_client *client)
{
	struct device *dev = &priv->client->dev;
	u8 id_msb, id_lsb;

	// TODO check ret value
	__ov5695_s_power(priv, 1);
	ov5695_read_reg(client, 0x300b, &id_msb);
	ov5695_read_reg(client, 0x300c, &id_lsb);
	__ov5695_s_power(priv, 0);

	if (CHIP_ID != (id_msb << 8 | id_lsb)) {
		dev_err(dev, "Wrong camera sensor id(%02x%02x)\n",
			id_msb, id_lsb);
		return -EINVAL;
	}

	dev_info(dev, "Detected OV%04x sensor\n", CHIP_ID);

	return 0;
}

static int ov5695_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ov5695_priv *priv;
	struct v4l2_subdev *sd;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->fmt = default_format;

	priv->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "Failed to get mclk\n");
		return -EINVAL;
	}
	ret = clk_set_rate(priv->mclk, 24000000);
	if (ret < 0) {
		dev_err(dev, "Failed to set mclk rate (24M)\n");
		return ret;
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
	priv->dvdd_regulator = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(priv->dvdd_regulator)) {
		dev_err(dev, "Failed to get dvdd-supply\n");
		return -EINVAL;
	}

	priv->state = MODULE_POWER_OFF;
	ret = ov5695_check_sensor_id(priv, client);
	if (ret)
		return ret;

	sd = &priv->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov5695_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ret = ov5695_initialize_controls(priv);
	if (ret)
		return ret;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &priv->pad, 0);
	if (ret < 0)
		return ret;
#endif

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto clean_entity;
	}

	dev_info(dev, "Probe successfully\n");

	return 0;

clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif

	return ret;
}

static int ov5695_remove(struct i2c_client *client)
{
	struct ov5695_priv *priv = i2c_get_clientdata(client);

	__ov5695_s_power(priv, 0);
	v4l2_async_unregister_subdev(&priv->subdev);

	return 0;
}

static const struct i2c_device_id ov5695_id[] = {
	{ "ov5695", 0 },
	{ }
};

static const struct of_device_id ov5695_of_match[] = {
	{ .compatible = "ov5695" },
	{},
};

static struct i2c_driver ov5695_i2c_driver = {
	.driver = {
		.name = "ov5695",
		.owner = THIS_MODULE,
		.of_match_table = ov5695_of_match
	},
	.probe		= &ov5695_probe,
	.remove		= &ov5695_remove,
	.id_table	= ov5695_id,
};

module_i2c_driver(ov5695_i2c_driver);

MODULE_DESCRIPTION("OmniVision ov5695 sensor driver");
MODULE_LICENSE("GPL v2");
