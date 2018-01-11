/*
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Authors: Philippe Cornu <philippe.cornu@st.com>
 *          Yannick Fertre <yannick.fertre@st.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
/* TODO Please check backlight gpio & pwm */
/* TODO Please check backlight put_device usage */
/* TODO Please adjust final timings */
/* TODO Please check content of prepare/unprepare/enable/disable */
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>

#define DRV_NAME "raydium_rm68200"

struct rm68200 {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *bl_dev;
	struct gpio_desc *reset_gpio;
	struct regulator *supply;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 52582,
	.hdisplay = 720,
	.hsync_start = 720 + 38,
	.hsync_end = 720 + 38 + 8,
	.htotal = 720 + 38 + 8 + 38,
	.vdisplay = 1280,
	.vsync_start = 1280 + 12,
	.vsync_end = 1280 + 12 + 4,
	.vtotal = 1280 + 12 + 4 + 12,
	.vrefresh = 50,
	.flags = 0,
	.width_mm = 68,
	.height_mm = 122,
};

static inline struct rm68200 *panel_to_rm68200(struct drm_panel *panel)
{
	return container_of(panel, struct rm68200, panel);
}

static void rm68200_dcs_write_buf(struct rm68200 *ctx, const void *data,
				  size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	if (mipi_dsi_dcs_write_buffer(dsi, data, len) < 0)
		DRM_WARN("mipi dsi dcs write buffer failed\n");
}

#define dcs_write_seq(ctx, seq...)			\
({							\
	static const u8 d[] = { seq };			\
	rm68200_dcs_write_buf(ctx, d, ARRAY_SIZE(d));	\
})

static int rm68200_init_sequence(struct rm68200 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	dcs_write_seq(ctx, 0xFE, 0x01);
	dcs_write_seq(ctx, 0x24, 0xC0);
	dcs_write_seq(ctx, 0x25, 0x53);
	dcs_write_seq(ctx, 0x26, 0x00);
	dcs_write_seq(ctx, 0x2B, 0xE5);
	dcs_write_seq(ctx, 0x27, 0x0A);
	dcs_write_seq(ctx, 0x29, 0x0A);
	dcs_write_seq(ctx, 0x16, 0x52);
	dcs_write_seq(ctx, 0x2F, 0x53);
	dcs_write_seq(ctx, 0x34, 0x5A);
	dcs_write_seq(ctx, 0x1B, 0x00);
	dcs_write_seq(ctx, 0x12, 0x0A);
	dcs_write_seq(ctx, 0x1A, 0x06);
	dcs_write_seq(ctx, 0x46, 0x56);
	dcs_write_seq(ctx, 0x52, 0xA0);
	dcs_write_seq(ctx, 0x53, 0x00);
	dcs_write_seq(ctx, 0x54, 0xA0);
	dcs_write_seq(ctx, 0x55, 0x00);

	dcs_write_seq(ctx, 0x5F, 0x11); /* 2 data lanes */

	dcs_write_seq(ctx, 0xFE, 0x03);
	dcs_write_seq(ctx, 0x00, 0x05);
	dcs_write_seq(ctx, 0x02, 0x0B);
	dcs_write_seq(ctx, 0x03, 0x0F);
	dcs_write_seq(ctx, 0x04, 0x7D);
	dcs_write_seq(ctx, 0x05, 0x00);
	dcs_write_seq(ctx, 0x06, 0x50);
	dcs_write_seq(ctx, 0x07, 0x05);
	dcs_write_seq(ctx, 0x08, 0x16);
	dcs_write_seq(ctx, 0x09, 0x0D);
	dcs_write_seq(ctx, 0x0A, 0x11);
	dcs_write_seq(ctx, 0x0B, 0x7D);
	dcs_write_seq(ctx, 0x0C, 0x00);
	dcs_write_seq(ctx, 0x0D, 0x50);
	dcs_write_seq(ctx, 0x0E, 0x07);
	dcs_write_seq(ctx, 0x0F, 0x08);
	dcs_write_seq(ctx, 0x10, 0x01);
	dcs_write_seq(ctx, 0x11, 0x02);
	dcs_write_seq(ctx, 0x12, 0x00);
	dcs_write_seq(ctx, 0x13, 0x7D);
	dcs_write_seq(ctx, 0x14, 0x00);
	dcs_write_seq(ctx, 0x15, 0x85);
	dcs_write_seq(ctx, 0x16, 0x08);
	dcs_write_seq(ctx, 0x17, 0x03);
	dcs_write_seq(ctx, 0x18, 0x04);
	dcs_write_seq(ctx, 0x19, 0x05);
	dcs_write_seq(ctx, 0x1A, 0x06);
	dcs_write_seq(ctx, 0x1B, 0x00);
	dcs_write_seq(ctx, 0x1C, 0x7D);
	dcs_write_seq(ctx, 0x1D, 0x00);
	dcs_write_seq(ctx, 0x1E, 0x85);
	dcs_write_seq(ctx, 0x1F, 0x08);
	dcs_write_seq(ctx, 0x20, 0x00);
	dcs_write_seq(ctx, 0x21, 0x00);
	dcs_write_seq(ctx, 0x22, 0x00);
	dcs_write_seq(ctx, 0x23, 0x00);
	dcs_write_seq(ctx, 0x24, 0x00);
	dcs_write_seq(ctx, 0x25, 0x00);
	dcs_write_seq(ctx, 0x26, 0x00);
	dcs_write_seq(ctx, 0x27, 0x00);
	dcs_write_seq(ctx, 0x28, 0x00);
	dcs_write_seq(ctx, 0x29, 0x00);
	dcs_write_seq(ctx, 0x2A, 0x07);
	dcs_write_seq(ctx, 0x2B, 0x08);
	dcs_write_seq(ctx, 0x2D, 0x01);
	dcs_write_seq(ctx, 0x2F, 0x02);
	dcs_write_seq(ctx, 0x30, 0x00);
	dcs_write_seq(ctx, 0x31, 0x40);
	dcs_write_seq(ctx, 0x32, 0x05);
	dcs_write_seq(ctx, 0x33, 0x08);
	dcs_write_seq(ctx, 0x34, 0x54);
	dcs_write_seq(ctx, 0x35, 0x7D);
	dcs_write_seq(ctx, 0x36, 0x00);
	dcs_write_seq(ctx, 0x37, 0x03);
	dcs_write_seq(ctx, 0x38, 0x04);
	dcs_write_seq(ctx, 0x39, 0x05);
	dcs_write_seq(ctx, 0x3A, 0x06);
	dcs_write_seq(ctx, 0x3B, 0x00);
	dcs_write_seq(ctx, 0x3D, 0x40);
	dcs_write_seq(ctx, 0x3F, 0x05);
	dcs_write_seq(ctx, 0x40, 0x08);
	dcs_write_seq(ctx, 0x41, 0x54);
	dcs_write_seq(ctx, 0x42, 0x7D);
	dcs_write_seq(ctx, 0x43, 0x00);
	dcs_write_seq(ctx, 0x44, 0x00);
	dcs_write_seq(ctx, 0x45, 0x00);
	dcs_write_seq(ctx, 0x46, 0x00);
	dcs_write_seq(ctx, 0x47, 0x00);
	dcs_write_seq(ctx, 0x48, 0x00);
	dcs_write_seq(ctx, 0x49, 0x00);
	dcs_write_seq(ctx, 0x4A, 0x00);
	dcs_write_seq(ctx, 0x4B, 0x00);
	dcs_write_seq(ctx, 0x4C, 0x00);
	dcs_write_seq(ctx, 0x4D, 0x00);
	dcs_write_seq(ctx, 0x4E, 0x00);
	dcs_write_seq(ctx, 0x4F, 0x00);
	dcs_write_seq(ctx, 0x50, 0x00);
	dcs_write_seq(ctx, 0x51, 0x00);
	dcs_write_seq(ctx, 0x52, 0x00);
	dcs_write_seq(ctx, 0x53, 0x00);
	dcs_write_seq(ctx, 0x54, 0x00);
	dcs_write_seq(ctx, 0x55, 0x00);
	dcs_write_seq(ctx, 0x56, 0x00);
	dcs_write_seq(ctx, 0x58, 0x00);
	dcs_write_seq(ctx, 0x59, 0x00);
	dcs_write_seq(ctx, 0x5A, 0x00);
	dcs_write_seq(ctx, 0x5B, 0x00);
	dcs_write_seq(ctx, 0x5C, 0x00);
	dcs_write_seq(ctx, 0x5D, 0x00);
	dcs_write_seq(ctx, 0x5E, 0x00);
	dcs_write_seq(ctx, 0x5F, 0x00);
	dcs_write_seq(ctx, 0x60, 0x00);
	dcs_write_seq(ctx, 0x61, 0x00);
	dcs_write_seq(ctx, 0x62, 0x00);
	dcs_write_seq(ctx, 0x63, 0x00);
	dcs_write_seq(ctx, 0x64, 0x00);
	dcs_write_seq(ctx, 0x65, 0x00);
	dcs_write_seq(ctx, 0x66, 0x00);
	dcs_write_seq(ctx, 0x67, 0x00);
	dcs_write_seq(ctx, 0x68, 0x00);
	dcs_write_seq(ctx, 0x69, 0x00);
	dcs_write_seq(ctx, 0x6A, 0x00);
	dcs_write_seq(ctx, 0x6B, 0x00);
	dcs_write_seq(ctx, 0x6C, 0x00);
	dcs_write_seq(ctx, 0x6D, 0x00);
	dcs_write_seq(ctx, 0x6E, 0x00);
	dcs_write_seq(ctx, 0x6F, 0x00);
	dcs_write_seq(ctx, 0x70, 0x00);
	dcs_write_seq(ctx, 0x71, 0x00);
	dcs_write_seq(ctx, 0x72, 0x20);
	dcs_write_seq(ctx, 0x73, 0x00);
	dcs_write_seq(ctx, 0x74, 0x08);
	dcs_write_seq(ctx, 0x75, 0x08);
	dcs_write_seq(ctx, 0x76, 0x08);
	dcs_write_seq(ctx, 0x77, 0x08);
	dcs_write_seq(ctx, 0x78, 0x08);
	dcs_write_seq(ctx, 0x79, 0x08);
	dcs_write_seq(ctx, 0x7A, 0x00);
	dcs_write_seq(ctx, 0x7B, 0x00);
	dcs_write_seq(ctx, 0x7C, 0x00);
	dcs_write_seq(ctx, 0x7D, 0x00);
	dcs_write_seq(ctx, 0x7E, 0xBF);
	dcs_write_seq(ctx, 0x7F, 0x02);
	dcs_write_seq(ctx, 0x80, 0x06);
	dcs_write_seq(ctx, 0x81, 0x14);
	dcs_write_seq(ctx, 0x82, 0x10);
	dcs_write_seq(ctx, 0x83, 0x16);
	dcs_write_seq(ctx, 0x84, 0x12);
	dcs_write_seq(ctx, 0x85, 0x08);
	dcs_write_seq(ctx, 0x86, 0x3F);
	dcs_write_seq(ctx, 0x87, 0x3F);
	dcs_write_seq(ctx, 0x88, 0x3F);
	dcs_write_seq(ctx, 0x89, 0x3F);
	dcs_write_seq(ctx, 0x8A, 0x3F);
	dcs_write_seq(ctx, 0x8B, 0x0C);
	dcs_write_seq(ctx, 0x8C, 0x0A);
	dcs_write_seq(ctx, 0x8D, 0x0E);
	dcs_write_seq(ctx, 0x8E, 0x3F);
	dcs_write_seq(ctx, 0x8F, 0x3F);
	dcs_write_seq(ctx, 0x90, 0x00);
	dcs_write_seq(ctx, 0x91, 0x04);
	dcs_write_seq(ctx, 0x92, 0x3F);
	dcs_write_seq(ctx, 0x93, 0x3F);
	dcs_write_seq(ctx, 0x94, 0x3F);
	dcs_write_seq(ctx, 0x95, 0x3F);
	dcs_write_seq(ctx, 0x96, 0x05);
	dcs_write_seq(ctx, 0x97, 0x01);
	dcs_write_seq(ctx, 0x98, 0x3F);
	dcs_write_seq(ctx, 0x99, 0x3F);
	dcs_write_seq(ctx, 0x9A, 0x0F);
	dcs_write_seq(ctx, 0x9B, 0x0B);
	dcs_write_seq(ctx, 0x9C, 0x0D);
	dcs_write_seq(ctx, 0x9D, 0x3F);
	dcs_write_seq(ctx, 0x9E, 0x3F);
	dcs_write_seq(ctx, 0x9F, 0x3F);
	dcs_write_seq(ctx, 0xA0, 0x3F);
	dcs_write_seq(ctx, 0xA2, 0x3F);
	dcs_write_seq(ctx, 0xA3, 0x09);
	dcs_write_seq(ctx, 0xA4, 0x13);
	dcs_write_seq(ctx, 0xA5, 0x17);
	dcs_write_seq(ctx, 0xA6, 0x11);
	dcs_write_seq(ctx, 0xA7, 0x15);
	dcs_write_seq(ctx, 0xA9, 0x07);
	dcs_write_seq(ctx, 0xAA, 0x03);
	dcs_write_seq(ctx, 0xAB, 0x3F);
	dcs_write_seq(ctx, 0xAC, 0x3F);
	dcs_write_seq(ctx, 0xAD, 0x05);
	dcs_write_seq(ctx, 0xAE, 0x01);
	dcs_write_seq(ctx, 0xAF, 0x17);
	dcs_write_seq(ctx, 0xB0, 0x13);
	dcs_write_seq(ctx, 0xB1, 0x15);
	dcs_write_seq(ctx, 0xB2, 0x11);
	dcs_write_seq(ctx, 0xB3, 0x0F);
	dcs_write_seq(ctx, 0xB4, 0x3F);
	dcs_write_seq(ctx, 0xB5, 0x3F);
	dcs_write_seq(ctx, 0xB6, 0x3F);
	dcs_write_seq(ctx, 0xB7, 0x3F);
	dcs_write_seq(ctx, 0xB8, 0x3F);
	dcs_write_seq(ctx, 0xB9, 0x0B);
	dcs_write_seq(ctx, 0xBA, 0x0D);
	dcs_write_seq(ctx, 0xBB, 0x09);
	dcs_write_seq(ctx, 0xBC, 0x3F);
	dcs_write_seq(ctx, 0xBD, 0x3F);
	dcs_write_seq(ctx, 0xBE, 0x07);
	dcs_write_seq(ctx, 0xBF, 0x03);
	dcs_write_seq(ctx, 0xC0, 0x3F);
	dcs_write_seq(ctx, 0xC1, 0x3F);
	dcs_write_seq(ctx, 0xC2, 0x3F);
	dcs_write_seq(ctx, 0xC3, 0x3F);
	dcs_write_seq(ctx, 0xC4, 0x02);
	dcs_write_seq(ctx, 0xC5, 0x06);
	dcs_write_seq(ctx, 0xC6, 0x3F);
	dcs_write_seq(ctx, 0xC7, 0x3F);
	dcs_write_seq(ctx, 0xC8, 0x08);
	dcs_write_seq(ctx, 0xC9, 0x0C);
	dcs_write_seq(ctx, 0xCA, 0x0A);
	dcs_write_seq(ctx, 0xCB, 0x3F);
	dcs_write_seq(ctx, 0xCC, 0x3F);
	dcs_write_seq(ctx, 0xCD, 0x3F);
	dcs_write_seq(ctx, 0xCE, 0x3F);
	dcs_write_seq(ctx, 0xCF, 0x3F);
	dcs_write_seq(ctx, 0xD0, 0x0E);
	dcs_write_seq(ctx, 0xD1, 0x10);
	dcs_write_seq(ctx, 0xD2, 0x14);
	dcs_write_seq(ctx, 0xD3, 0x12);
	dcs_write_seq(ctx, 0xD4, 0x16);
	dcs_write_seq(ctx, 0xD5, 0x00);
	dcs_write_seq(ctx, 0xD6, 0x04);
	dcs_write_seq(ctx, 0xD7, 0x3F);
	dcs_write_seq(ctx, 0xDC, 0x02);
	dcs_write_seq(ctx, 0xDE, 0x12);
	dcs_write_seq(ctx, 0xFE, 0x0E);
	dcs_write_seq(ctx, 0x01, 0x75);
	dcs_write_seq(ctx, 0xFE, 0x04);
	dcs_write_seq(ctx, 0x60, 0x00);
	dcs_write_seq(ctx, 0x61, 0x0C);
	dcs_write_seq(ctx, 0x62, 0x12);
	dcs_write_seq(ctx, 0x63, 0x0E);
	dcs_write_seq(ctx, 0x64, 0x06);
	dcs_write_seq(ctx, 0x65, 0x12);
	dcs_write_seq(ctx, 0x66, 0x0E);
	dcs_write_seq(ctx, 0x67, 0x0B);
	dcs_write_seq(ctx, 0x68, 0x15);
	dcs_write_seq(ctx, 0x69, 0x0B);
	dcs_write_seq(ctx, 0x6A, 0x10);
	dcs_write_seq(ctx, 0x6B, 0x07);
	dcs_write_seq(ctx, 0x6C, 0x0F);
	dcs_write_seq(ctx, 0x6D, 0x12);
	dcs_write_seq(ctx, 0x6E, 0x0C);
	dcs_write_seq(ctx, 0x6F, 0x00);
	dcs_write_seq(ctx, 0x70, 0x00);
	dcs_write_seq(ctx, 0x71, 0x0C);
	dcs_write_seq(ctx, 0x72, 0x12);
	dcs_write_seq(ctx, 0x73, 0x0E);
	dcs_write_seq(ctx, 0x74, 0x06);
	dcs_write_seq(ctx, 0x75, 0x12);
	dcs_write_seq(ctx, 0x76, 0x0E);
	dcs_write_seq(ctx, 0x77, 0x0B);
	dcs_write_seq(ctx, 0x78, 0x15);
	dcs_write_seq(ctx, 0x79, 0x0B);
	dcs_write_seq(ctx, 0x7A, 0x10);
	dcs_write_seq(ctx, 0x7B, 0x07);
	dcs_write_seq(ctx, 0x7C, 0x0F);
	dcs_write_seq(ctx, 0x7D, 0x12);
	dcs_write_seq(ctx, 0x7E, 0x0C);
	dcs_write_seq(ctx, 0x7F, 0x00);
	dcs_write_seq(ctx, 0xFE, 0x00);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	mdelay(200);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	mdelay(200);

	return 0;
}

static int rm68200_disable(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);

	if (!ctx->enabled)
		return 0;

	/* Power off the backlight */
	if (ctx->bl_dev) {
		ctx->bl_dev->props.power = FB_BLANK_POWERDOWN;
		ctx->bl_dev->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(ctx->bl_dev);
	}

	ctx->enabled = false;

	return 0;
}

static int rm68200_unprepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		DRM_WARN("failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		DRM_WARN("failed to enter sleep mode: %d\n", ret);

	msleep(120);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	regulator_disable(ctx->supply);

	ctx->prepared = false;

	return 0;
}

static int rm68200_prepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable supply: %d\n", ret);
		return ret;
	}

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(100);
	}

	ret = rm68200_init_sequence(ctx);
	if (ret)
		return ret;

	ctx->prepared = true;

	return 0;
}

static int rm68200_enable(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);

	if (ctx->enabled)
		return 0;

	/* Power on the backlight */
	if (ctx->bl_dev) {
		ctx->bl_dev->props.state &= ~BL_CORE_FBBLANK;
		ctx->bl_dev->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->bl_dev);
	}

	ctx->enabled = true;

	return 0;
}

static int rm68200_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			  default_mode.hdisplay, default_mode.vdisplay,
			  default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs rm68200_drm_funcs = {
	.disable   = rm68200_disable,
	.unprepare = rm68200_unprepare,
	.prepare   = rm68200_prepare,
	.enable    = rm68200_enable,
	.get_modes = rm68200_get_modes,
};

static int rm68200_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *backlight;
	struct rm68200 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpio\n");
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return PTR_ERR(ctx->supply);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &rm68200_drm_funcs;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->bl_dev = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->bl_dev)
			return -EPROBE_DEFER;
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach failed. Is host ready?\n");
		drm_panel_remove(&ctx->panel);
		if (ctx->bl_dev)
			put_device(&ctx->bl_dev->dev);
		return ret;
	}

	DRM_INFO(DRV_NAME "_panel %ux%u@%u %ubpp dsi %udl - ready\n",
		 default_mode.hdisplay, default_mode.vdisplay,
		 default_mode.vrefresh,
		 mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	return 0;
}

static int rm68200_remove(struct mipi_dsi_device *dsi)
{
	struct rm68200 *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->bl_dev)
		put_device(&ctx->bl_dev->dev);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id raydium_rm68200_of_match[] = {
	{ .compatible = "raydium,rm68200" },
	{ }
};
MODULE_DEVICE_TABLE(of, raydium_rm68200_of_match);

static struct mipi_dsi_driver raydium_rm68200_driver = {
	.probe  = rm68200_probe,
	.remove = rm68200_remove,
	.driver = {
		.name = DRV_NAME "_panel",
		.of_match_table = raydium_rm68200_of_match,
	},
};
module_mipi_dsi_driver(raydium_rm68200_driver);

MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_AUTHOR("Yannick Fertre <yannick.fertre@st.com>");
MODULE_DESCRIPTION("DRM Driver for Raydium RM68200 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
