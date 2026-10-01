// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3FC3 (AMS638YQ01) 1080x2400 AMOLED MIPI-DSI command-mode panel
 * as fitted to the Samsung Galaxy A32 (SM-A325F).
 *
 * Ported from the downstream smcdsd driver
 * (kernel/downstream/.../smcdsd_panel/panels/s6e3fc3_a32.c +
 * s6e3fc3_a32_param.h, read-only references). Every DCS byte below is traced
 * to a param.h line in the comment above it; anything without downstream
 * evidence is marked TODO-BLOCKED and left out, never invented.
 *
 * Architecture follows panel-samsung-s6e3ha2.c (DSI + mipi_dsi_dcs_* +
 * backlight + regulator/reset + mode) as the upstream template.
 *
 * HARDWARE-UNPROVEN: compiles and passes host table tests, but no unit has
 * yet confirmed probe/ID/init/mode/backlight over real DSI hardware.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include "panel-samsung-s6e3fc3-data.h"

/* Downstream: EXTEND_BRIGHTNESS 486, UI_MAX_BRIGHTNESS 255,
 * UI_DEFAULT_BRIGHTNESS 128 (s6e3fc3_a32_param.h:10-12). */
#define S6E3FC3_EXTEND_BRIGHTNESS	486
#define S6E3FC3_UI_MAX_BRIGHTNESS	255
#define S6E3FC3_DEFAULT_BRIGHTNESS	128

/* Brightness above UI max selects the HBM path
 * (s6e3fc3_a32.c:50 LEVEL_IS_HBM). */
#define S6E3FC3_LEVEL_IS_HBM(br)	((br) > S6E3FC3_UI_MAX_BRIGHTNESS)

/* Panel rev gating (s6e3fc3_a32.c:1394, panel_rev = lcdtype & 0xFF).
 * Live-measured unit reports ID 0x800004, i.e. rev 4, so only the rev>=3
 * code paths are ported here. The rev<3 paths (DIM_GPARA+DIM table at
 * s6e3fc3_a32.c:548-551, brightness_table_id_00_02, LCD_SEQ_INIT_00_02)
 * are NOT-YET-IMPLEMENTED: no rev<3 unit to validate against. */
#define S6E3FC3_PANEL_REV		4

/* DCS register addresses (s6e3fc3_a32_param.h:25-31). */
#define S6E3FC3_REG_BRIGHTNESS		0x51

/* A32 mode geometry from display_lcd_a32_common.dtsi:
 * resolution 1080x2400 (:105), phys 67x148mm (:106-107),
 * porches V 2/12/5 H 10/14/30 (:109-114), 4 lanes CMD mode (:99-103),
 * dfps 60+90Hz (:120-125), live default 90Hz. */
#define S6E3FC3_HDISPLAY	1080
#define S6E3FC3_VDISPLAY	2400
#define S6E3FC3_HSYNC		10
#define S6E3FC3_HBP		14
#define S6E3FC3_HFP		30
#define S6E3FC3_VSYNC		2
#define S6E3FC3_VBP		12
#define S6E3FC3_VFP		5
#define S6E3FC3_HTOTAL		(S6E3FC3_HDISPLAY + S6E3FC3_HSYNC + \
				 S6E3FC3_HBP + S6E3FC3_HFP)		/* 1134 */
#define S6E3FC3_VTOTAL		(S6E3FC3_VDISPLAY + S6E3FC3_VSYNC + \
				 S6E3FC3_VBP + S6E3FC3_VFP)		/* 2419 */
/* Pixel clocks are derived (htotal*vtotal*vrefresh/1000), not measured:
 * 90Hz: 1134*2419*90/1000 = 246883; 60Hz: 1134*2419*60/1000 = 164589. */
#define S6E3FC3_CLOCK_90HZ	246883
#define S6E3FC3_CLOCK_60HZ	164589
#define S6E3FC3_WIDTH_MM	67
#define S6E3FC3_HEIGHT_MM	148

struct s6e3fc3 {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *bl_dev;

	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;

	/* Downstream lcd->fps probe default is FPS_60 (s6e3fc3_a32.c:1409);
	 * the 90Hz dfps level (display_lcd_a32_common.dtsi:120-125) is
	 * exposed as a mode but there is no evidence for the live boot
	 * default beyond the probe value, so boot at 60Hz. Per-mode FPS
	 * switching is NOT-YET-IMPLEMENTED (no mode-switch hook updates
	 * this; see get_modes). */
	bool fps_90;
};

static int s6e3fc3_dcs_write(struct s6e3fc3 *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	return mipi_dsi_dcs_write_buffer(dsi, data, len);
}

#define s6e3fc3_dcs_write_seq_static(ctx, seq...) do {	\
	static const u8 d[] = { seq };			\
	int ret;					\
	ret = s6e3fc3_dcs_write(ctx, d, ARRAY_SIZE(d));	\
	if (ret < 0)					\
		return ret;				\
} while (0)

#define s6e3fc3_call_write_func(ret, func) do {	\
	ret = (func);				\
	if (ret < 0)				\
		return ret;			\
} while (0)

/* --- TEST_KEY unlock/lock (s6e3fc3_a32_param.h:164-192) --- */
static int s6e3fc3_test_key_on_f0(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_TEST_KEY_ON_F0 (param.h:164-167) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xf0, 0x5a, 0x5a);
	return 0;
}

static int s6e3fc3_test_key_off_f0(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_TEST_KEY_OFF_F0 (param.h:169-172) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xf0, 0xa5, 0xa5);
	return 0;
}

static int s6e3fc3_test_key_on_fc(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_TEST_KEY_ON_FC (param.h:174-177) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xfc, 0x5a, 0x5a);
	return 0;
}

static int s6e3fc3_test_key_off_fc(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_TEST_KEY_OFF_FC (param.h:179-182) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xfc, 0xa5, 0xa5);
	return 0;
}

/* --- init-table helpers (LCD_SEQ_INIT_03, param.h:1357-1383) ---
 * Rev>=3 init table (live unit is rev 4). LCD_SEQ_INIT_00_02
 * (param.h:1329-1355, rev<3) is NOT ported, see S6E3FC3_PANEL_REV. */
static int s6e3fc3_global_para_set(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_GLOBAL_PARA_SET (param.h:204-212) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xf2,
		0x00, 0x05, 0x0e, 0x58, 0x50, 0x00, 0x0c, 0x00,
		0x04, 0x30, 0xb1, 0x30, 0xb1, 0x0c, 0x04, 0xbc,
		0x26, 0xe9, 0x0c, 0x00, 0x04, 0x10, 0x00, 0x10,
		0x26, 0xa8, 0x10, 0x00, 0x10, 0x10, 0x34, 0x10,
		0x00, 0x40, 0x30, 0xc8, 0x00, 0xc8, 0x00, 0x00,
		0xce);
	return 0;
}

static int s6e3fc3_ltps_update(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_LTPS_UPDATE (param.h:214-217) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xf7, 0x0f);
	return 0;
}

static int s6e3fc3_caset_paset(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_CASET 1080 cols (param.h:224-227);
	 * sibling A22 uses 0x00,0x00,0x02,0xCF for its 720-col panel
	 * (s6e3fc3_a22_param.h:225-228) -- A32 values win here. */
	s6e3fc3_dcs_write_seq_static(ctx, 0x2a, 0x00, 0x00, 0x04, 0x37);
	/* SEQ_S6E3FC3_PASET 2400 rows (param.h:229-232);
	 * A22 uses 0x00,0x00,0x06,0x3F (1600 rows, :230-233) -- A32 wins. */
	s6e3fc3_dcs_write_seq_static(ctx, 0x2b, 0x00, 0x00, 0x09, 0x5f);
	return 0;
}

static int s6e3fc3_ffc_set(struct s6e3fc3 *ctx)
{
	/* FFC GPARA+SET pairs (param.h:235-252) for data_rate 1660. */
	s6e3fc3_dcs_write_seq_static(ctx, 0xb0, 0x00, 0x2a, 0xc5);
	s6e3fc3_dcs_write_seq_static(ctx, 0xc5, 0x0d, 0x10, 0x80, 0x45);
	s6e3fc3_dcs_write_seq_static(ctx, 0xb0, 0x00, 0x3e, 0xc5);
	s6e3fc3_dcs_write_seq_static(ctx, 0xc5, 0x34, 0x40);
	return 0;
}

static int s6e3fc3_err_fg_set(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_ERR_FG_ENABLE + ERR_FG_SET (param.h:254-262) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xe5, 0x15);
	s6e3fc3_dcs_write_seq_static(ctx, 0xed, 0x44, 0x4c, 0x20);
	return 0;
}

static int s6e3fc3_pcd_set(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_PCD (param.h:264-267) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xcc, 0x5c, 0x51);
	return 0;
}

static int s6e3fc3_acl_default(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_ACL_GPARA + SEQ_S6E3FC3_ACL_DEFAULT (param.h:269-279) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xb0, 0x03, 0xb3, 0x65);
	s6e3fc3_dcs_write_seq_static(ctx, 0x65,
		0x55, 0x00, 0xb0, 0x51, 0x66, 0x98, 0x15, 0x55,
		0x55, 0x55, 0x08, 0xf1, 0xc6, 0x48, 0x40, 0x00,
		0x20, 0x10, 0x09);
	return 0;
}

static int s6e3fc3_freq_set(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_FREQ_GPARA + SEQ_S6E3FC3_FREQ_SET (param.h:281-289) */
	s6e3fc3_dcs_write_seq_static(ctx, 0xb0, 0x00, 0x27, 0xf2);
	s6e3fc3_dcs_write_seq_static(ctx, 0xf2, 0x00);
	return 0;
}

static int s6e3fc3_te_on(struct s6e3fc3 *ctx)
{
	/* SEQ_S6E3FC3_TE_ON (param.h:219-222). Tearing-effect output is
	 * enabled via this DCS command; the A32 DT evidence names no TE GPIO
	 * (only CON/DET/PCD/ERR detect GPIOs), so no te-gpios property. */
	s6e3fc3_dcs_write_seq_static(ctx, 0x35, 0x00);
	return 0;
}

/* --- brightness-path helpers (low_level_set_brightness, s6e3fc3_a32.c:776) --- */

static int s6e3fc3_set_fps(struct s6e3fc3 *ctx)
{
	/* smcdsd_panel_set_fps with_br=1 branch (s6e3fc3_a32.c:463-485):
	 * FPS_60HZ / FPS_90HZ (param.h:291-299). */
	if (ctx->fps_90)
		s6e3fc3_dcs_write_seq_static(ctx, 0x60, 0x08, 0x00);
	else
		s6e3fc3_dcs_write_seq_static(ctx, 0x60, 0x00, 0x00);
	return 0;
}

static int s6e3fc3_set_hbm(struct s6e3fc3 *ctx, unsigned int brightness)
{
	/* HBM_TABLE[TRANS_DIMMING_ON] (param.h:708-711); probe forces
	 * trans_dimming=TRANS_DIMMING_ON (s6e3fc3_a32.c:1408), so normal
	 * brightness sends HBM_OFF (0x53,0x28) and HBM sends HBM_ON
	 * (0x53,0xE8) (param.h:306-314). */
	if (S6E3FC3_LEVEL_IS_HBM(brightness))
		s6e3fc3_dcs_write_seq_static(ctx, 0x53, 0xe8);
	else
		s6e3fc3_dcs_write_seq_static(ctx, 0x53, 0x28);
	return 0;
}

static int s6e3fc3_set_sync(struct s6e3fc3 *ctx, unsigned int brightness)
{
	/* Rev>=3 branch of smcdsd_panel_set_wrctrld without
	 * force_normal_transition (s6e3fc3_a32.c:552-565): SYNC_CONTROL_GPARA
	 * (param.h:368-371) + SYNC_TABLE[hbm] (param.h:715) =
	 * SMOOTH (0x63,0x60, :373-376) for normal, NORMAL (0x63,0x20,
	 * :378-381) for HBM. The VINT pre/post writes (:556-560, :521-523)
	 * only run on mask-layer transitions and are NOT ported (no mask
	 * layer in DRM; TODO-BLOCKED on selfmask evidence). */
	s6e3fc3_dcs_write_seq_static(ctx, 0xb0, 0x00, 0x91, 0x63);
	if (S6E3FC3_LEVEL_IS_HBM(brightness))
		s6e3fc3_dcs_write_seq_static(ctx, 0x63, 0x20);
	else
		s6e3fc3_dcs_write_seq_static(ctx, 0x63, 0x60);
	return 0;
}

static int s6e3fc3_set_elvss(struct s6e3fc3 *ctx)
{
	/* smcdsd_panel_set_elvss (s6e3fc3_a32.c:487-511): TSET byte =
	 * temperature; NORMAL_TEMPERATURE is 0 (param.h:14), so the first
	 * (and, with no thermal input, only) write is {0xB5, 0x00}.
	 * Base table SEQ_S6E3FC3_ELVSS_SET (param.h:301-304). */
	s6e3fc3_dcs_write_seq_static(ctx, 0xb5, 0x00);
	return 0;
}

static int s6e3fc3_set_acl(struct s6e3fc3 *ctx, unsigned int brightness)
{
	/* brightness_opr_table[1] with adaptive_control on (probe default at
	 * s6e3fc3_a32.c:1384) and no mask layer (param.h:721-728):
	 * 0..255 -> ACL 15P, 256..486 -> ACL 8P. Tables ACL_15P/ACL_08P
	 * (param.h:388-396); ACL_OFF (param.h:383-386) is mask/AOD-only. */
	if (S6E3FC3_LEVEL_IS_HBM(brightness))
		s6e3fc3_dcs_write_seq_static(ctx, 0x55, 0x01);
	else
		s6e3fc3_dcs_write_seq_static(ctx, 0x55, 0x03);
	return 0;
}

static int s6e3fc3_set_aor(struct s6e3fc3 *ctx, unsigned int brightness)
{
	u8 data[4] = { 0x63, 0x00, 0x00, 0x18 };
	int ret;

	/* smcdsd_panel_set_aor (s6e3fc3_a32.c:513-530): AOR_GPARA
	 * (param.h:398-401) then aor_table[brightness] (param.h:838+).
	 * Normal range is constant (see data.h); HBM rows come from the
	 * HBM AOR table. */
	if (S6E3FC3_LEVEL_IS_HBM(brightness))
		memcpy(&data[1], s6e3fc3_aor_hbm_table[brightness - 256], 3);

	s6e3fc3_call_write_func(ret,
		s6e3fc3_dcs_write(ctx,
			(const u8[]){ 0xb0, 0x00, 0x76, 0x63 }, 4));
	s6e3fc3_call_write_func(ret, s6e3fc3_dcs_write(ctx, data, 4));

	return 0;
}

static int s6e3fc3_set_gamma(struct s6e3fc3 *ctx, unsigned int brightness)
{
	u16 level = s6e3fc3_gamma_table[brightness];
	u8 data[3];

	/* smcdsd_panel_set_wrctrld bl_reg pack (s6e3fc3_a32.c:541-543,
	 * LDI_REG_BRIGHTNESS 0x51 at param.h:25):
	 * byte1 = bits[9:8], byte2 = bits[7:0] of the gamma level. */
	data[0] = S6E3FC3_REG_BRIGHTNESS;
	data[1] = (level >> 8) & 0x03;
	data[2] = level & 0xff;

	return s6e3fc3_dcs_write(ctx, data, ARRAY_SIZE(data));
}

static int s6e3fc3_low_level_set_brightness(struct s6e3fc3 *ctx,
					    unsigned int brightness)
{
	int ret;

	/* low_level_set_brightness order (s6e3fc3_a32.c:776-800), minus the
	 * mask-only pre_mask_delay/mask_delay mdelays (:782-787, :573-577)
	 * which never fire without a mask layer. */
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_on_f0(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_fps(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_sync(ctx, brightness));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_hbm(ctx, brightness));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_gamma(ctx, brightness));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_elvss(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_acl(ctx, brightness));
	s6e3fc3_call_write_func(ret, s6e3fc3_set_aor(ctx, brightness));
	s6e3fc3_call_write_func(ret, s6e3fc3_ltps_update(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_off_f0(ctx));

	return 0;
}

static int s6e3fc3_get_brightness(struct backlight_device *bl_dev)
{
	return bl_dev->props.brightness;
}

static int s6e3fc3_set_brightness(struct backlight_device *bl_dev)
{
	struct s6e3fc3 *ctx = bl_get_data(bl_dev);
	unsigned int brightness = bl_dev->props.brightness;

	if (brightness > S6E3FC3_EXTEND_BRIGHTNESS) {
		dev_err(ctx->dev, "invalid brightness: %u\n", brightness);
		return -EINVAL;
	}

	/* panel_set_brightness only acts while enabled (downstream gates on
	 * PANEL_STATE_RESUMED, s6e3fc3_a32.c:880-894). BACKLIGHT_POWER_ON is
	 * set in enable(); disable() parks it at OFF. (BACKLIGHT_POWER_REDUCED
	 * is deliberately not used: deprecated in 6.18, "don't use in new
	 * code" per include/linux/backlight.h.) */
	if (bl_dev->props.power != BACKLIGHT_POWER_ON)
		return 0;

	return s6e3fc3_low_level_set_brightness(ctx, brightness);
}

static const struct backlight_ops s6e3fc3_bl_ops = {
	.get_brightness = s6e3fc3_get_brightness,
	.update_status = s6e3fc3_set_brightness,
};

/* --- panel power / init --- */

static int s6e3fc3_panel_init(struct s6e3fc3 *ctx)
{
	int ret;

	/* s6e3fc3_init (s6e3fc3_a32.c:1141-1184), minus the DCS reads (ID at
	 * :1146, init_info at :1181 -- TODO-BLOCKED, no verified ID on a
	 * mainline-probed unit) and minus the selfmask SRAM upload
	 * (:1164-1174, TODO-BLOCKED on the selfmask image payload).
	 * The 10ms settle after the (omitted) ID read (:1148) is kept for
	 * panel timing, same rationale as the kept 90ms delay below. */
	usleep_range(10000, 11000);

	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_exit_sleep_mode(to_mipi_dsi_device(ctx->dev)));
	usleep_range(100, 110);
	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_exit_sleep_mode(to_mipi_dsi_device(ctx->dev)));
	usleep_range(100, 110);
	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_exit_sleep_mode(to_mipi_dsi_device(ctx->dev)));
	usleep_range(100, 110);

	usleep_range(30000, 31000);

	/* LCD_SEQ_INIT_03, the rev>=3 init table (param.h:1357-1383). */
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_on_f0(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_on_fc(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_global_para_set(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_ltps_update(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_caset_paset(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_ffc_set(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_err_fg_set(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_pcd_set(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_acl_default(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_freq_set(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_ltps_update(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_te_on(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_off_fc(ctx));
	s6e3fc3_call_write_func(ret, s6e3fc3_test_key_off_f0(ctx));

	/* Brightness Setting (s6e3fc3_a32.c:1176-1177, force=1). */
	s6e3fc3_call_write_func(ret,
		s6e3fc3_low_level_set_brightness(ctx, ctx->bl_dev->props.brightness));

	/* usleep 90ms before read_init_info (:1179); the read itself is
	 * TODO-BLOCKED (see above), the delay is kept for panel timing. */
	usleep_range(90000, 91000);

	return 0;
}

static int s6e3fc3_power_on(struct s6e3fc3 *ctx)
{
	int ret;

	/* panel_power_enable: 1p8 -> 2ms -> 3p0 -> 2ms
	 * (display_lcd_a32_common.dtsi:56-62). supplies[0]=vddio (1.8V),
	 * supplies[1]=vci (3.0V). */
	ret = regulator_enable(ctx->supplies[0].consumer);
	if (ret < 0)
		return ret;
	usleep_range(2000, 2100);
	ret = regulator_enable(ctx->supplies[1].consumer);
	if (ret < 0) {
		regulator_disable(ctx->supplies[0].consumer);
		return ret;
	}
	usleep_range(2000, 2100);

	/* panel_reset_enable: 1ms/high/1ms/low/1ms/high/11ms
	 * (display_lcd_a32_common.dtsi:71-80). */
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(11000, 11100);

	return 0;
}

static int s6e3fc3_power_off(struct s6e3fc3 *ctx)
{
	/* panel_power_disable: 3ms, 3p0 off, 5ms, 1p8 off, 5ms
	 * (display_lcd_a32_common.dtsi:63-70). Reset GPIO left high;
	 * panel_reset_disable (:81-85) only runs on full reset. */
	usleep_range(3000, 3100);
	regulator_disable(ctx->supplies[1].consumer);
	usleep_range(5000, 5100);
	regulator_disable(ctx->supplies[0].consumer);
	usleep_range(5000, 5100);

	return 0;
}

static int s6e3fc3_prepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = container_of(panel, struct s6e3fc3, panel);
	int ret;

	ret = s6e3fc3_power_on(ctx);
	if (ret < 0)
		return ret;

	ret = s6e3fc3_panel_init(ctx);
	if (ret < 0)
		goto err;

	return 0;

err:
	s6e3fc3_power_off(ctx);
	return ret;
}

static int s6e3fc3_enable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = container_of(panel, struct s6e3fc3, panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	/* s6e3fc3_displayon: Display On (0x29) (s6e3fc3_a32.c:1186-1196). */
	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_set_display_on(dsi));
	ctx->bl_dev->props.power = BACKLIGHT_POWER_ON;

	return 0;
}

static int s6e3fc3_disable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = container_of(panel, struct s6e3fc3, panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	/* s6e3fc3_exit (s6e3fc3_a32.c:1111-1139), minus the debug reads
	 * (dsierr/rddpm/rddsm at :1117-1119). */
	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_set_display_off(dsi));
	msleep(20);
	s6e3fc3_call_write_func(ret, mipi_dsi_dcs_enter_sleep_mode(dsi));
	msleep(120);

	ctx->bl_dev->props.power = BACKLIGHT_POWER_OFF;

	return 0;
}

static int s6e3fc3_unprepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = container_of(panel, struct s6e3fc3, panel);

	return s6e3fc3_power_off(ctx);
}

/* --- modes --- */

static const struct drm_display_mode s6e3fc3_mode_90hz = {
	.clock = S6E3FC3_CLOCK_90HZ,
	.hdisplay = S6E3FC3_HDISPLAY,
	.hsync_start = S6E3FC3_HDISPLAY + S6E3FC3_HFP,
	.hsync_end = S6E3FC3_HDISPLAY + S6E3FC3_HFP + S6E3FC3_HSYNC,
	.htotal = S6E3FC3_HTOTAL,
	.vdisplay = S6E3FC3_VDISPLAY,
	.vsync_start = S6E3FC3_VDISPLAY + S6E3FC3_VFP,
	.vsync_end = S6E3FC3_VDISPLAY + S6E3FC3_VFP + S6E3FC3_VSYNC,
	.vtotal = S6E3FC3_VTOTAL,
};

static const struct drm_display_mode s6e3fc3_mode_60hz = {
	.clock = S6E3FC3_CLOCK_60HZ,
	.hdisplay = S6E3FC3_HDISPLAY,
	.hsync_start = S6E3FC3_HDISPLAY + S6E3FC3_HFP,
	.hsync_end = S6E3FC3_HDISPLAY + S6E3FC3_HFP + S6E3FC3_HSYNC,
	.htotal = S6E3FC3_HTOTAL,
	.vdisplay = S6E3FC3_VDISPLAY,
	.vsync_start = S6E3FC3_VDISPLAY + S6E3FC3_VFP,
	.vsync_end = S6E3FC3_VDISPLAY + S6E3FC3_VFP + S6E3FC3_VSYNC,
	.vtotal = S6E3FC3_VTOTAL,
};

static int s6e3fc3_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode90, *mode60;

	/* Expose the dfps pair (display_lcd_a32_common.dtsi:120-125).
	 * 60Hz is preferred to match the probe FPS_60 default (s6e3fc3_a32.c:1409);
	 * the 0x60 FPS DCS is fixed at that default -- selecting the 90Hz mode
	 * does NOT reprogram it yet (TODO-BLOCKED on a per-mode update hook). */
	mode90 = drm_mode_duplicate(connector->dev, &s6e3fc3_mode_90hz);
	if (!mode90)
		return -ENOMEM;
	drm_mode_set_name(mode90);
	mode90->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode90);

	mode60 = drm_mode_duplicate(connector->dev, &s6e3fc3_mode_60hz);
	if (!mode60)
		return -ENOMEM;
	drm_mode_set_name(mode60);
	mode60->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode60);

	connector->display_info.width_mm = S6E3FC3_WIDTH_MM;
	connector->display_info.height_mm = S6E3FC3_HEIGHT_MM;

	return 2;
}

static const struct drm_panel_funcs s6e3fc3_drm_funcs = {
	.prepare = s6e3fc3_prepare,
	.enable = s6e3fc3_enable,
	.disable = s6e3fc3_disable,
	.unprepare = s6e3fc3_unprepare,
	.get_modes = s6e3fc3_get_modes,
};

static int s6e3fc3_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3fc3 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3fc3, panel,
				   &s6e3fc3_drm_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	/* Probe FPS_60 default (s6e3fc3_a32.c:1409); see fps_90 comment. */
	ctx->fps_90 = false;

	/* Downstream: DSI CMD mode, 4 lanes, 24-bit packed
	 * (display_lcd_a32_common.dtsi:99-103); cont_clock 0 (:118).
	 * MODE_LPM is the command-mode convention (cf. sofet00). */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* supplies[0]=vddio: LCD_1P8_EN (pio25); supplies[1]=vci: LCD_3P0_EN
	 * (pio153) (display_lcd_a32_common.dtsi:31-44). */
	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to get regulators: %d\n", ret);
		return ret;
	}

	/* LCD_RST (pio45) (display_lcd_a32_common.dtsi:24-30). */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	/* Backlight range is the platform brightness index 0..486
	 * (EXTEND_BRIGHTNESS, param.h:10); HBM lives above UI max 255. */
	ctx->bl_dev = devm_backlight_device_register(dev, "s6e3fc3", dev, ctx,
						     &s6e3fc3_bl_ops, NULL);
	if (IS_ERR(ctx->bl_dev)) {
		dev_err(dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl_dev);
	}
	ctx->bl_dev->props.max_brightness = S6E3FC3_EXTEND_BRIGHTNESS;
	ctx->bl_dev->props.brightness = S6E3FC3_DEFAULT_BRIGHTNESS;
	ctx->bl_dev->props.power = BACKLIGHT_POWER_OFF;

	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		goto remove_panel;

	return ret;

remove_panel:
	drm_panel_remove(&ctx->panel);
	return ret;
}

static void s6e3fc3_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fc3 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3fc3_of_match[] = {
	{ .compatible = "samsung,s6e3fc3-a32" },
	{ }
};
MODULE_DEVICE_TABLE(of, s6e3fc3_of_match);

static struct mipi_dsi_driver s6e3fc3_driver = {
	.probe = s6e3fc3_probe,
	.remove = s6e3fc3_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc3",
		.of_match_table = s6e3fc3_of_match,
	},
};
module_mipi_dsi_driver(s6e3fc3_driver);

MODULE_AUTHOR("A32 Mainline Bringup");
MODULE_DESCRIPTION("Samsung S6E3FC3 (Galaxy A32) DSI command-mode panel");
MODULE_LICENSE("GPL");
