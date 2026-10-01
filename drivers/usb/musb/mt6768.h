// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6768 MUSB/UDC integration glue - shared constants and pure logic.
 *
 * This header is deliberately kernel-independent in its lower half so the
 * same role-state machine and resource expectations can be unit-tested on
 * the host (native gcc, no hardware). Kernel-only helpers are guarded by
 * __KERNEL__.
 *
 * Behavioral reference (downstream, read-only, NOT copied as code):
 * - Controller DTS: usb0@11200000, compatible "mediatek,mt6768-usb20",
 *   reg 0x11200000/0x10000 + 0x11CC0000/0x10000, GIC_SPI 97 LEVEL_LOW,
 *   mode=2 multipoint=1 num_eps=16, clocks infracfg_ao CLK_IFR_ICUSB +
 *   topckgen USB_TOP_SEL + UNIVPLL3_D4, pericfg handle
 *   (downstream mt6768.dts:2321-2336).
 * - PHY base via of_iomap(node,1) + 0x800/0x818/0x820 pokes
 *   (usb20_phy.c:139-170); MAC+PHY reset = pericfg bit29 pulse
 *   (usb20_otg_if.c:105-112).
 * - Driver pattern: platform_device_alloc("musb-hdrc") (usb20/mt6768/
 *   usb20.c:1868; siblings mt6765/83/39 identical).
 * - Live userspace: sys.usb.controller=musb-hdrc (getprop),
 *   vendor.usb.controller "musb-hdrc", configfs UDC bind, cmode 2/1
 *   writes (init.mt6768.usb.rc:49,92,126,801-803).
 *
 * Mainline mapping (evidence, kernel/mainline/):
 * - drivers/usb/musb/mediatek.c matches "mediatek,mtk-musb" (fallback) and
 *   already implements the generic glue: 3 clocks (main/mcu/univpll),
 *   generic PHY handling, usb_role_switch, musb-hdrc child device,
 *   runtime PM. Reused as-is; this file does NOT duplicate the MUSB core,
 *   gadget, or host code.
 * - Documentation/devicetree/bindings/usb/mediatek,musb.yaml already lists
 *   "mediatek,mt6768-musb" + "mediatek,mtk-musb" with clocks
 *   main/mcu/univpll, phys, dr_mode, usb-role-switch.
 * - Documentation/devicetree/bindings/phy/mediatek,tphy.yaml maps
 *   "mediatek,mt6768-tphy" -> "mediatek,generic-tphy-v1"; the driver maps
 *   generic-tphy-v1 -> V1 pdata. No new PHY driver is needed; downstream
 *   phy_tuning tables / tune@1..9 / HS values are NOT copied here.
 */

#ifndef __MUSB_MT6768_H
#define __MUSB_MT6768_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/* ---- A32-evidenced resource expectations (for validation, not MMIO) ---- */

/* Downstream MAC window: 0x11200000/0x10000. Mainline musb decodes only the
 * first 0x1000 (see mt6768.dtsi usb2 node); the remainder is reserved/
 * QMU-adjacent downstream space, NOT modelled as musb regs here. */
#define MT6768_MUSB_MAC_BASE		0x11200000u
#define MT6768_MUSB_MAC_SIZE_DOWNSTREAM	0x10000u
#define MT6768_MUSB_MAC_SIZE_MAINLINE	0x1000u

/* Downstream second window 0x11CC0000/0x10000 is the PHY block. In mainline
 * it is modelled as the usb2phy (t-phy@11cc0000) node, NOT as a musb reg. */
#define MT6768_MUSB_PHY_BASE		0x11CC0000u
#define MT6768_MUSB_PHY_U2_OFF_0	0x0800u

/* GIC SPI 97, LEVEL_LOW (downstream + mt6768.dtsi agree). */
#define MT6768_MUSB_IRQ_SPI		97u

/* Clock IDs expected on the musb node (mt6768.dtsi + binding agree). */
#define MT6768_MUSB_CLK_MAIN		"main"
#define MT6768_MUSB_CLK_MCU		"mcu"
#define MT6768_MUSB_CLK_UNIVPLL		"univpll"
#define MT6768_MUSB_CLKS_NUM		3

/* Pericfg MAC+PHY reset: downstream pulses bit 29 of the pericfg word
 * (usb20_otg_if.c:105-112, musb_otg_reset_usb). The glue resolves pericfg
 * at runtime via the "mediatek,mt6768-pericfg" syscon; NO new DT property
 * is introduced (the musb binding has additionalProperties:false). */
#define MT6768_PERICFG_COMPAT		"mediatek,mt6768-pericfg"
#define MT6768_PERICFG_USB_RST_BIT	29u

/* Downstream PHY poke offsets (usb20_phy.c:156-165), recorded for audit
 * ONLY. The generic-tphy-v1 driver owns PHY programming; this glue never
 * writes these offsets and invents no HS tuning values. */
#define MT6768_PHY_POKE_U2_820		0x820u
#define MT6768_PHY_POKE_U2_800		0x800u
#define MT6768_PHY_POKE_U2_818		0x818u

/* Downstream reports mode=2 multipoint=1 num_eps=16. Mainline generic glue
 * uses 8 EPs / 11 ram bits (mediatek.c). The downstream num_eps is a
 * downstream-usb20 property, NOT a proven musb hdrc_config value, so the
 * glue below keeps the upstream-proven 8-EP FIFO shape and classifies a
 * 16-EP shape as UNPROVEN (see the .c file). */
#define MT6768_MUSB_EPS_GENERIC_PROVEN	8
#define MT6768_MUSB_RAM_BITS_GENERIC	11
#define MT6768_MUSB_EPS_DOWNSTREAM_RAW	16

/* ---- Role state machine (pure logic, host-testable) ---- */

enum mt6768_usb_role {
	MT6768_ROLE_NONE = 0,
	MT6768_ROLE_HOST = 1,
	MT6768_ROLE_DEVICE = 2,
};

#define MT6768_EINVAL_CUSTOM	(-22)

/*
 * mt6768_role_next - compute the next role-switch state.
 * @cur: current role; @req: requested role; @port_otg: true when the
 * musb port_mode is OTG (mode changes only allowed then, mirroring
 * mtk_musb_set_mode() in mediatek.c).
 *
 * Returns the new role, or MT6768_EINVAL_CUSTOM for invalid requests
 * (unknown role value, or a mode change on a fixed-mode port).
 * Same-role requests are a no-op returning @cur.
 */
static inline int mt6768_role_next(int cur, int req, int port_otg)
{
	if (req != MT6768_ROLE_NONE &&
	    req != MT6768_ROLE_HOST &&
	    req != MT6768_ROLE_DEVICE)
		return MT6768_EINVAL_CUSTOM;

	if (req == cur)
		return cur;

	if (!port_otg)
		return MT6768_EINVAL_CUSTOM;

	return req;
}

/*
 * mt6768_role_name - human-readable role name for debug traces.
 * Never returns NULL.
 */
static inline const char *mt6768_role_name(int role)
{
	switch (role) {
	case MT6768_ROLE_HOST:
		return "host";
	case MT6768_ROLE_DEVICE:
		return "device";
	case MT6768_ROLE_NONE:
		return "none";
	default:
		return "invalid";
	}
}

#ifdef __KERNEL__
#include <linux/device.h>
#include <linux/ratelimit.h>

/*
 * MT6768 debug helpers: all probe/clock/PHY/reset/role/PM/UDC-register
 * traces are rate-limited dev_dbg() so a chattering controller cannot spam
 * the log. Extra register dumps sit behind the module parameter in mt6768.c
 * (debug_level), default off.
 */
#define mt6768_dbg(dev, fmt, ...) \
	dev_dbg_ratelimited(dev, "mt6768-musb: " fmt, ##__VA_ARGS__)

#define mt6768_info_rl(dev, fmt, ...) \
	dev_info_ratelimited(dev, "mt6768-musb: " fmt, ##__VA_ARGS__)
#endif /* __KERNEL__ */

#endif /* __MUSB_MT6768_H */
