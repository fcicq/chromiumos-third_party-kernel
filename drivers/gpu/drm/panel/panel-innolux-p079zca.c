/*
 * Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int bpc;
	struct {
		unsigned int width;
		unsigned int height;
	} size;
};

struct panel_desc_dsi {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

struct innolux_panel {
	struct drm_panel base;
	struct mipi_dsi_device *link;
	const struct panel_desc_dsi *dsi_desc;

	struct backlight_device *backlight;
	struct regulator *vddi;
	struct regulator *avdd;
	struct regulator *avee;
	struct gpio_desc *enable_gpio;

	bool prepared;
	bool enabled;
};

static inline struct innolux_panel *to_innolux_panel(struct drm_panel *panel)
{
	return container_of(panel, struct innolux_panel, base);
}

static int innolux_panel_disable(struct drm_panel *panel)
{
	struct innolux_panel *innolux = to_innolux_panel(panel);
	int err;

	if (!innolux->enabled)
		return 0;

	innolux->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(innolux->backlight);

	err = mipi_dsi_dcs_set_display_off(innolux->link);
	if (err < 0)
		DRM_DEV_ERROR(panel->dev, "failed to set display off: %d\n",
			      err);

	innolux->enabled = false;

	return 0;
}

static int innolux_panel_unprepare(struct drm_panel *panel)
{
	struct innolux_panel *innolux = to_innolux_panel(panel);
	int err;

	if (!innolux->prepared)
		return 0;

	err = mipi_dsi_dcs_enter_sleep_mode(innolux->link);
	if (err < 0) {
		DRM_DEV_ERROR(panel->dev, "failed to enter sleep mode: %d\n",
			      err);
		return err;
	}

	gpiod_set_value_cansleep(innolux->enable_gpio, 0);

	/* T8: 80ms - 1000ms */
	msleep(80);

	regulator_disable(innolux->avee);
	regulator_disable(innolux->avdd);
	regulator_disable(innolux->vddi);

	innolux->prepared = false;

	return 0;
}

static int innolux_panel_prepare(struct drm_panel *panel)
{
	struct innolux_panel *innolux = to_innolux_panel(panel);
	int err, regulator_err;

	if (innolux->prepared)
		return 0;

	gpiod_set_value_cansleep(innolux->enable_gpio, 0);

	err = regulator_enable(innolux->vddi);
	if (err < 0)
		return err;

	err = regulator_enable(innolux->avdd);
	if (err < 0)
		goto disable_vddi;

	err = regulator_enable(innolux->avee);
	if (err < 0)
		goto disable_avdd;

	/* T2: 15ms - 1000ms */
	usleep_range(15000, 16000);

	gpiod_set_value_cansleep(innolux->enable_gpio, 1);

	/* T4: 15ms - 1000ms */
	usleep_range(15000, 16000);

	err = mipi_dsi_dcs_exit_sleep_mode(innolux->link);
	if (err < 0) {
		DRM_DEV_ERROR(panel->dev, "failed to exit sleep mode: %d\n",
			      err);
		goto poweroff;
	}

	/* T6: 120ms - 1000ms*/
	msleep(120);

	err = mipi_dsi_dcs_set_display_on(innolux->link);
	if (err < 0) {
		DRM_DEV_ERROR(panel->dev, "failed to set display on: %d\n",
			      err);
		goto poweroff;
	}

	/* T7: 5ms */
	usleep_range(5000, 6000);

	innolux->prepared = true;

	return 0;

poweroff:
	gpiod_set_value_cansleep(innolux->enable_gpio, 0);
	regulator_disable(innolux->avee);
disable_avdd:
	regulator_disable(innolux->avdd);
disable_vddi:
	regulator_disable(innolux->vddi);

	return err;
}

static int innolux_panel_enable(struct drm_panel *panel)
{
	struct innolux_panel *innolux = to_innolux_panel(panel);
	int ret;

	if (innolux->enabled)
		return 0;

	innolux->backlight->props.power = FB_BLANK_UNBLANK;
	ret = backlight_update_status(innolux->backlight);
	if (ret) {
		DRM_DEV_ERROR(panel->drm->dev,
			      "Failed to enable backlight %d\n", ret);
		return ret;
	}

	innolux->enabled = true;

	return 0;
}

static const struct drm_display_mode innolux_p079zca_mode = {
	.clock = 56900,
	.hdisplay = 768,
	.hsync_start = 768 + 40,
	.hsync_end = 768 + 40 + 40,
	.htotal = 768 + 40 + 40 + 40,
	.vdisplay = 1024,
	.vsync_start = 1024 + 20,
	.vsync_end = 1024 + 20 + 4,
	.vtotal = 1024 + 20 + 4 + 20,
	.vrefresh = 60,
};

static const struct panel_desc_dsi innolux_p079zca_panel_desc = {
	.desc = {
		.modes = &innolux_p079zca_mode,
		.bpc = 8,
		.size = {
			.width = 120,
			.height = 160,
		},
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
		 MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static int innolux_panel_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	struct innolux_panel *innolux = to_innolux_panel(panel);
	const struct drm_display_mode *m = innolux->dsi_desc->desc.modes;

	mode = drm_mode_duplicate(panel->drm, m);
	if (!mode) {
		DRM_DEV_ERROR(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			      m->hdisplay, m->vdisplay, m->vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm =
			innolux->dsi_desc->desc.size.width;
	panel->connector->display_info.height_mm =
			innolux->dsi_desc->desc.size.height;
	panel->connector->display_info.bpc = innolux->dsi_desc->desc.bpc;

	return 1;
}

static const struct drm_panel_funcs innolux_panel_funcs = {
	.disable = innolux_panel_disable,
	.unprepare = innolux_panel_unprepare,
	.prepare = innolux_panel_prepare,
	.enable = innolux_panel_enable,
	.get_modes = innolux_panel_get_modes,
};

static const struct of_device_id innolux_of_match[] = {
	{ .compatible = "innolux,p079zca",
	  .data = &innolux_p079zca_panel_desc
	},
};
MODULE_DEVICE_TABLE(of, innolux_of_match);

static int innolux_panel_add(struct mipi_dsi_device *dsi,
			     const struct panel_desc_dsi *desc)
{
	struct innolux_panel *innolux;
	struct device *dev = &dsi->dev;
	struct device_node *np;
	int err;

	innolux = devm_kzalloc(dev, sizeof(*innolux), GFP_KERNEL);
	if (!innolux)
		return -ENOMEM;

	innolux->dsi_desc = desc;
	innolux->vddi = devm_regulator_get(dev, "power");
	if (IS_ERR(innolux->vddi))
		return PTR_ERR(innolux->vddi);

	innolux->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(innolux->avdd))
		return PTR_ERR(innolux->avdd);

	innolux->avee = devm_regulator_get(dev, "avee");
	if (IS_ERR(innolux->avee))
		return PTR_ERR(innolux->avee);

	innolux->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(innolux->enable_gpio)) {
		err = PTR_ERR(innolux->enable_gpio);
		dev_dbg(dev, "failed to get enable gpio: %d\n", err);
		innolux->enable_gpio = NULL;
	}

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		innolux->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!innolux->backlight)
			return -EPROBE_DEFER;
	}

	drm_panel_init(&innolux->base);
	innolux->base.funcs = &innolux_panel_funcs;
	innolux->base.dev = dev;

	err = drm_panel_add(&innolux->base);
	if (err < 0)
		goto put_backlight;

	dev_set_drvdata(dev, innolux);
	innolux->link = dsi;

	return 0;
put_backlight:
	put_device(&innolux->backlight->dev);

	return err;
}

static void innolux_panel_del(struct innolux_panel *innolux)
{
	if (innolux->base.dev)
		drm_panel_remove(&innolux->base);

	put_device(&innolux->backlight->dev);
}

static int innolux_panel_probe(struct mipi_dsi_device *dsi)
{

	const struct panel_desc_dsi *dsi_desc;
	const struct of_device_id *id;
	int err;

	id = of_match_node(innolux_of_match, dsi->dev.of_node);
	if (!id)
		return -ENODEV;

	dsi_desc = id->data;
	dsi->mode_flags = dsi_desc->flags;
	dsi->format = dsi_desc->format;
	dsi->lanes = dsi_desc->lanes;

	err = innolux_panel_add(dsi, dsi_desc);
	if (err < 0)
		return err;

	return mipi_dsi_attach(dsi);
}

static int innolux_panel_remove(struct mipi_dsi_device *dsi)
{
	struct innolux_panel *innolux = mipi_dsi_get_drvdata(dsi);
	int err;

	err = innolux_panel_unprepare(&innolux->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to unprepare panel: %d\n",
			      err);

	err = innolux_panel_disable(&innolux->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to disable panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to detach from DSI host: %d\n",
			      err);

	drm_panel_detach(&innolux->base);
	innolux_panel_del(innolux);

	return 0;
}

static void innolux_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct innolux_panel *innolux = mipi_dsi_get_drvdata(dsi);

	innolux_panel_unprepare(&innolux->base);
	innolux_panel_disable(&innolux->base);
}

static struct mipi_dsi_driver innolux_panel_driver = {
	.driver = {
		.name = "panel-innolux-dsi",
		.of_match_table = innolux_of_match,
	},
	.probe = innolux_panel_probe,
	.remove = innolux_panel_remove,
	.shutdown = innolux_panel_shutdown,
};
module_mipi_dsi_driver(innolux_panel_driver);

MODULE_AUTHOR("Chris Zhong <zyw@rock-chips.com>");
MODULE_AUTHOR("Lin Huang <hl@rock-chips.com>");
MODULE_DESCRIPTION("Innolux P079ZCA panel driver");
MODULE_LICENSE("GPL v2");
