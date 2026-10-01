// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6768 MUSB/UDC integration glue (Track C, offline bring-up).
 *
 * Purpose: map the downstream MT6768 USB sequence onto the mainline MUSB
 * platform glue + generic T-PHY so that /sys/class/udc/ exposes the REAL
 * A32 controller (no dummy UDC, no hardcoded fake entries) and the
 * already-present configfs RNDIS userspace can bind to usb0.
 *
 * What this file IS:
 * - A thin platform driver for "mediatek,mt6768-musb" that reuses the
 *   upstream-proven mediatek.c glue pattern (musb-hdrc child device via
 *   platform_device_register_full(), 3 clocks main/mcu/univpll, generic
 *   PHY API, usb_role_switch with userspace control, runtime PM) plus
 *   MT6768 specifics: pericfg MAC+PHY reset (bit29 pulse, downstream
 *   usb20_otg_if.c:105-112), MT6768 resource/clock/PHY/role/PM debug
 *   instrumentation (all rate-limited), and an 8-EP FIFO shape identical
 *   to the upstream generic (deliberately NOT widened to the downstream
 *   raw num_eps=16, which is classified UNPROVEN as a musb EP count).
 * - A consumer of drivers/phy/mediatek/phy-mtk-tphy.c via the generic PHY
 *   API. "mediatek,mt6768-tphy"+"mediatek,generic-tphy-v1" (mt6768.dtsi)
 *   binds generic-tphy-v1 -> V1 pdata in mainline; no new PHY driver, no
 *   tune@ tables, no invented HS values here.
 *
 * What this file is NOT:
 * - NOT a second MUSB core: musb_core/host/gadget/dma/debugfs are reused
 *   untouched (musb_interrupt(), musbhs DMA, configfs UDC all as-is).
 * - NOT a copy of downstream usb20/usb20_phy/usb20_otg_if: VBUS/drvvbus
 *   GPIOs, portmode/saving/cmode procfs hooks, UART-switch and BC1.2
 *   charger code are downstream-specific and absent here. cmode writes
 *   from init.mt6768.usb.rc:801-803 have no mainline equivalent and are
 *   documented only.
 * - NOT wired into Kconfig/Makefile by this change (out of scope for the
 *   Track C write allowance). Wiring is a 2-line addition documented in
 *   docs/MUSB-UDC-IMPLEMENTATION.md; until then the generic
 *   "mediatek,mtk-musb" fallback in mediatek.c keeps binding working and
 *   this driver simply does not compete (it is not built in-tree).
 *
 * Evidence chain: see docs/MUSB-UDC-IMPLEMENTATION.md and mt6768.h.
 * Hardware status: OFFLINE-VALIDATED only; physical enumeration is
 * HARDWARE-UNPROVEN (no flash/reboot/hardware contact performed).
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/usb/role.h>
#include <linux/usb/usb_phy_generic.h>

#include "mt6768.h"
#include "musb_core.h"
#include "musb_dma.h"

/* Mirror of the upstream register block used for debug + ISR descoping. */
#define MT6768_USB_L1INTS		0x00a0
#define MT6768_USB_L1INTM		0x00a4
#define MT6768_MUSB_TXFUNCADDR		0x0480

#define MT6768_RXTOG			0x80
#define MT6768_RXTOGEN			0x82
#define MT6768_TXTOG			0x84
#define MT6768_TXTOGEN			0x86
#define MT6768_TOGGLE_EN		GENMASK(15, 0)

#define MT6768_TX_INT			BIT(0)
#define MT6768_RX_INT			BIT(1)
#define MT6768_USBCOM_INT		BIT(2)
#define MT6768_DMA_INT			BIT(3)

#define MT6768_DMA_STATUS_MSK		GENMASK(7, 0)
#define MT6768_DMA_UNMASK_SET_MSK	GENMASK(31, 24)

static int debug_level;
module_param(debug_level, int, 0444);
MODULE_PARM_DESC(debug_level, "extra MT6768 MUSB register dumps (default 0)");

struct mt6768_glue {
	struct device *dev;
	struct musb *musb;
	struct platform_device *musb_pdev;
	struct platform_device *usb_phy;
	struct phy *phy;
	struct usb_phy *xceiv;
	enum phy_mode phy_mode;
	struct clk_bulk_data clks[MT6768_MUSB_CLKS_NUM];
	enum usb_role role;
	struct usb_role_switch *role_sw;
	struct regmap *pericfg;
};

/* ---- pericfg MAC+PHY reset (downstream usb20_otg_if.c:105-112) ----
 * Pulse bit29 of the pericfg word: set, then clear. Resolved at runtime
 * via syscon so no DT binding change is needed.
 */
static int mt6768_pericfg_reset(struct mt6768_glue *glue)
{
	unsigned int val;
	int ret;

	glue->pericfg = syscon_regmap_lookup_by_compatible(MT6768_PERICFG_COMPAT);
	if (IS_ERR(glue->pericfg)) {
		mt6768_dbg(glue->dev, "pericfg syscon not present, skip MAC+PHY reset (%ld)\n",
			   PTR_ERR(glue->pericfg));
		glue->pericfg = NULL;
		return 0; /* optional on boards without exposed pericfg */
	}

	ret = regmap_read(glue->pericfg, 0, &val);
	if (ret) {
		mt6768_dbg(glue->dev, "pericfg read failed: %d\n", ret);
		return ret;
	}
	mt6768_dbg(glue->dev, "pericfg pre-reset word=0x%08x, pulsing bit%u\n",
		   val, MT6768_PERICFG_USB_RST_BIT);

	ret = regmap_update_bits(glue->pericfg, 0,
				 BIT(MT6768_PERICFG_USB_RST_BIT),
				 BIT(MT6768_PERICFG_USB_RST_BIT));
	if (ret)
		return ret;
	ret = regmap_update_bits(glue->pericfg, 0,
				 BIT(MT6768_PERICFG_USB_RST_BIT), 0);
	if (ret)
		return ret;

	mt6768_dbg(glue->dev, "pericfg MAC+PHY reset pulse done\n");
	return 0;
}

static int mt6768_clks_get(struct mt6768_glue *glue)
{
	glue->clks[0].id = MT6768_MUSB_CLK_MAIN;
	glue->clks[1].id = MT6768_MUSB_CLK_MCU;
	glue->clks[2].id = MT6768_MUSB_CLK_UNIVPLL;

	return devm_clk_bulk_get(glue->dev, MT6768_MUSB_CLKS_NUM, glue->clks);
}

static int mt6768_otg_switch_set(struct mt6768_glue *glue, enum usb_role role)
{
	struct musb *musb = glue->musb;
	u8 devctl;
	enum usb_role new_role;
	int next;

	/* Shared pure-logic gate, unit-tested on host (mt6768.h). */
	next = mt6768_role_next(glue->role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				glue->role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				MT6768_ROLE_NONE,
				role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				role == USB_ROLE_NONE ? MT6768_ROLE_NONE : 99,
				musb->port_mode == MUSB_OTG);
	if (next == MT6768_EINVAL_CUSTOM)
		return -EINVAL;
	if ((enum usb_role)next == glue->role && musb->port_mode == MUSB_OTG)
		return 0;

	devctl = readb(musb->mregs + MUSB_DEVCTL);

	switch (role) {
	case USB_ROLE_HOST:
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
		glue->phy_mode = PHY_MODE_USB_HOST;
		new_role = USB_ROLE_HOST;
		if (glue->role == USB_ROLE_NONE)
			phy_power_on(glue->phy);
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		MUSB_HST_MODE(musb);
		break;
	case USB_ROLE_DEVICE:
		musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		glue->phy_mode = PHY_MODE_USB_DEVICE;
		new_role = USB_ROLE_DEVICE;
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		if (glue->role == USB_ROLE_NONE)
			phy_power_on(glue->phy);
		MUSB_DEV_MODE(musb);
		break;
	case USB_ROLE_NONE:
		glue->phy_mode = PHY_MODE_USB_OTG;
		new_role = USB_ROLE_NONE;
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
		if (glue->role != USB_ROLE_NONE)
			phy_power_off(glue->phy);
		break;
	default:
		dev_err(glue->dev, "invalid role request\n");
		return -EINVAL;
	}

	glue->role = new_role;
	phy_set_mode(glue->phy, glue->phy_mode);
	mt6768_dbg(glue->dev, "role -> %s (phy_mode=%d)\n",
		   mt6768_role_name(next), glue->phy_mode);
	return 0;
}

static int mt6768_role_sw_set(struct usb_role_switch *sw, enum usb_role role)
{
	return mt6768_otg_switch_set(usb_role_switch_get_drvdata(sw), role);
}

static enum usb_role mt6768_role_sw_get(struct usb_role_switch *sw)
{
	struct mt6768_glue *glue = usb_role_switch_get_drvdata(sw);

	return glue->role;
}

static int mt6768_otg_switch_init(struct mt6768_glue *glue)
{
	struct usb_role_switch_desc desc = { 0 };

	desc.set = mt6768_role_sw_set;
	desc.get = mt6768_role_sw_get;
	desc.allow_userspace_control = true;
	desc.fwnode = dev_fwnode(glue->dev);
	desc.driver_data = glue;
	glue->role_sw = usb_role_switch_register(glue->dev, &desc);

	return PTR_ERR_OR_ZERO(glue->role_sw);
}

static void mt6768_otg_switch_exit(struct mt6768_glue *glue)
{
	usb_role_switch_unregister(glue->role_sw);
}

static irqreturn_t mt6768_generic_interrupt(int irq, void *__hci)
{
	unsigned long flags;
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);
	musb->int_usb = musb_clearb(musb->mregs, MUSB_INTRUSB);
	musb->int_rx = musb_clearw(musb->mregs, MUSB_INTRRX);
	musb->int_tx = musb_clearw(musb->mregs, MUSB_INTRTX);

	if ((musb->int_usb & MUSB_INTR_RESET) && !is_host_active(musb)) {
		musb_ep_select(musb->mregs, 0);
		musb_writeb(musb->mregs, MUSB_FADDR, 0);
	}

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static irqreturn_t mt6768_interrupt(int irq, void *dev_id)
{
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = dev_id;
	u32 l1_ints;

	l1_ints = musb_readl(musb->mregs, MT6768_USB_L1INTS) &
		  musb_readl(musb->mregs, MT6768_USB_L1INTM);

	if (l1_ints & (MT6768_TX_INT | MT6768_RX_INT | MT6768_USBCOM_INT))
		retval = mt6768_generic_interrupt(irq, musb);

#if defined(CONFIG_USB_INVENTRA_DMA)
	if (l1_ints & MT6768_DMA_INT)
		retval = dma_controller_irq(irq, musb->dma_controller);
#endif
	return retval;
}

static u32 mt6768_busctl_offset(u8 epnum, u16 offset)
{
	return MT6768_MUSB_TXFUNCADDR + offset + 8 * epnum;
}

static u8 mt6768_clearb(void __iomem *addr, unsigned int offset)
{
	u8 data = musb_readb(addr, offset);

	musb_writeb(addr, offset, data); /* W1C */
	return data;
}

static u16 mt6768_clearw(void __iomem *addr, unsigned int offset)
{
	u16 data = musb_readw(addr, offset);

	musb_writew(addr, offset, data); /* W1C */
	return data;
}

static int mt6768_set_mode(struct musb *musb, u8 mode)
{
	struct device *dev = musb->controller;
	struct mt6768_glue *glue = dev_get_drvdata(dev->parent);
	enum phy_mode new_mode;
	enum usb_role new_role;

	switch (mode) {
	case MUSB_HOST:
		new_mode = PHY_MODE_USB_HOST;
		new_role = USB_ROLE_HOST;
		break;
	case MUSB_PERIPHERAL:
		new_mode = PHY_MODE_USB_DEVICE;
		new_role = USB_ROLE_DEVICE;
		break;
	case MUSB_OTG:
		new_mode = PHY_MODE_USB_OTG;
		new_role = USB_ROLE_NONE;
		break;
	default:
		dev_err(glue->dev, "invalid mode request\n");
		return -EINVAL;
	}

	if (glue->phy_mode == new_mode)
		return 0;

	if (musb->port_mode != MUSB_OTG) {
		dev_err(glue->dev, "mode change not supported on fixed port\n");
		return -EINVAL;
	}

	return mt6768_otg_switch_set(glue, new_role);
}

static void mt6768_debug_regs(struct mt6768_glue *glue)
{
	struct musb *musb = glue->musb;

	if (!debug_level || !musb || !musb->mregs)
		return;
	dev_dbg(glue->dev, "regs DEVCTL=0x%02x L1INTS=0x%08x L1INTM=0x%08x role=%s\n",
		musb_readb(musb->mregs, MUSB_DEVCTL),
		musb_readl(musb->mregs, MT6768_USB_L1INTS),
		musb_readl(musb->mregs, MT6768_USB_L1INTM),
		mt6768_role_name(glue->role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				 glue->role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				 MT6768_ROLE_NONE));
}

static int mt6768_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct mt6768_glue *glue = dev_get_drvdata(dev->parent);
	int ret;

	glue->musb = musb;
	musb->phy = glue->phy;
	musb->xceiv = glue->xceiv;
	musb->is_host = false;
	musb->isr = mt6768_interrupt;

	musb_writew(musb->mregs, MT6768_TXTOGEN, MT6768_TOGGLE_EN);
	musb_writew(musb->mregs, MT6768_RXTOGEN, MT6768_TOGGLE_EN);

	if (musb->port_mode == MUSB_OTG) {
		ret = mt6768_otg_switch_init(glue);
		if (ret)
			return ret;
	}

	ret = phy_init(glue->phy);
	if (ret)
		goto err_phy_init;

	ret = phy_power_on(glue->phy);
	if (ret)
		goto err_phy_power_on;

	phy_set_mode(glue->phy, glue->phy_mode);
	mt6768_dbg(glue->dev, "PHY init/power-on done, phy_mode=%d (generic-tphy-v1 owns tuning)\n",
		   glue->phy_mode);

#if defined(CONFIG_USB_INVENTRA_DMA)
	musb_writel(musb->mregs, MUSB_HSDMA_INTR,
		    MT6768_DMA_STATUS_MSK | MT6768_DMA_UNMASK_SET_MSK);
#endif
	musb_writel(musb->mregs, MT6768_USB_L1INTM,
		    MT6768_TX_INT | MT6768_RX_INT |
		    MT6768_USBCOM_INT | MT6768_DMA_INT);
	mt6768_debug_regs(glue);
	mt6768_dbg(glue->dev, "UDC register path ready, role=%s (bind via configfs UDC)\n",
		   mt6768_role_name(glue->role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				    glue->role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				    MT6768_ROLE_NONE));
	return 0;

err_phy_power_on:
	phy_exit(glue->phy);
err_phy_init:
	if (musb->port_mode == MUSB_OTG)
		mt6768_otg_switch_exit(glue);
	return ret;
}

static u16 mt6768_get_toggle(struct musb_qh *qh, int is_out)
{
	struct musb *musb = qh->hw_ep->musb;
	u8 epnum = qh->hw_ep->epnum;
	u16 toggle = musb_readw(musb->mregs, is_out ? MT6768_TXTOG : MT6768_RXTOG);

	return toggle & (1 << epnum);
}

static u16 mt6768_set_toggle(struct musb_qh *qh, int is_out, struct urb *urb)
{
	struct musb *musb = qh->hw_ep->musb;
	u8 epnum = qh->hw_ep->epnum;
	u16 value, toggle = usb_gettoggle(urb->dev, qh->epnum, is_out);

	if (is_out) {
		value = musb_readw(musb->mregs, MT6768_TXTOG);
		value |= toggle << epnum;
		musb_writew(musb->mregs, MT6768_TXTOG, value);
	} else {
		value = musb_readw(musb->mregs, MT6768_RXTOG);
		value |= toggle << epnum;
		musb_writew(musb->mregs, MT6768_RXTOG, value);
	}

	return 0;
}

static int mt6768_musb_exit(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct mt6768_glue *glue = dev_get_drvdata(dev->parent);

	mt6768_dbg(glue->dev, "exit, role=%s\n",
		   mt6768_role_name(glue->role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				    glue->role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				    MT6768_ROLE_NONE));
	mt6768_otg_switch_exit(glue);
	phy_power_off(glue->phy);
	phy_exit(glue->phy);
	clk_bulk_disable_unprepare(MT6768_MUSB_CLKS_NUM, glue->clks);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return 0;
}

static const struct musb_platform_ops mt6768_musb_ops = {
	.quirks = MUSB_DMA_INVENTRA,
	.init = mt6768_musb_init,
	.get_toggle = mt6768_get_toggle,
	.set_toggle = mt6768_set_toggle,
	.exit = mt6768_musb_exit,
#ifdef CONFIG_USB_INVENTRA_DMA
	.dma_init = musbhs_dma_controller_create_noirq,
	.dma_exit = musbhs_dma_controller_destroy,
#endif
	.clearb = mt6768_clearb,
	.clearw = mt6768_clearw,
	.busctl_offset = mt6768_busctl_offset,
	.set_mode = mt6768_set_mode,
};

/*
 * FIFO shape: upstream-proven 8-EP layout (identical geometry to
 * mediatek.c). Downstream raw num_eps=16 is NOT mapped here (UNPROVEN as
 * a musb EP count; musb_core MUSB_C_NUM_EPS and dyn_fifo sizing for 6768
 * are unverified without hardware, so no invented 16-EP table).
 */
#define MT6768_MUSB_MAX_EP_NUM	MT6768_MUSB_EPS_GENERIC_PROVEN
#define MT6768_MUSB_RAM_BITS	MT6768_MUSB_RAM_BITS_GENERIC

static const struct musb_fifo_cfg mt6768_musb_mode_cfg[] = {
	{ .hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 1, .style = FIFO_RX, .maxpacket = 512, },
	{ .hw_ep_num = 2, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 2, .style = FIFO_RX, .maxpacket = 512, },
	{ .hw_ep_num = 3, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 3, .style = FIFO_RX, .maxpacket = 512, },
	{ .hw_ep_num = 4, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 4, .style = FIFO_RX, .maxpacket = 512, },
	{ .hw_ep_num = 5, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 5, .style = FIFO_RX, .maxpacket = 512, },
	{ .hw_ep_num = 6, .style = FIFO_TX, .maxpacket = 1024, },
	{ .hw_ep_num = 6, .style = FIFO_RX, .maxpacket = 1024, },
	{ .hw_ep_num = 7, .style = FIFO_TX, .maxpacket = 512, },
	{ .hw_ep_num = 7, .style = FIFO_RX, .maxpacket = 64, },
};

static const struct musb_hdrc_config mt6768_musb_hdrc_config = {
	.fifo_cfg = mt6768_musb_mode_cfg,
	.fifo_cfg_size = ARRAY_SIZE(mt6768_musb_mode_cfg),
	.multipoint = true,
	.dyn_fifo = true,
	.num_eps = MT6768_MUSB_MAX_EP_NUM,
	.ram_bits = MT6768_MUSB_RAM_BITS,
};

static const struct platform_device_info mt6768_dev_info = {
	.name = "musb-hdrc",
	.id = PLATFORM_DEVID_AUTO,
	.dma_mask = DMA_BIT_MASK(32),
};

static int mt6768_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data *pdata;
	struct mt6768_glue *glue;
	struct platform_device_info pinfo;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	mt6768_dbg(dev, "probe start (res/irq/clk/PHY/reset/role/PM/UDC instrumentation active)\n");

	glue = devm_kzalloc(dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->dev = dev;
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create child devices\n");
	mt6768_dbg(dev, "DT children populated (t-phy handled by generic-tphy-v1)\n");

	/* MT6768-specific: MAC+PHY reset before clocks (downstream order). */
	ret = mt6768_pericfg_reset(glue);
	if (ret)
		return dev_err_probe(dev, ret, "pericfg reset failed\n");

	ret = mt6768_clks_get(glue);
	if (ret)
		return ret;
	mt6768_dbg(dev, "clocks acquired: main/mcu/univpll\n");

	pdata->config = &mt6768_musb_hdrc_config;
	pdata->platform_ops = &mt6768_musb_ops;
	pdata->mode = usb_get_dr_mode(dev);

	if (IS_ENABLED(CONFIG_USB_MUSB_HOST))
		pdata->mode = USB_DR_MODE_HOST;
	else if (IS_ENABLED(CONFIG_USB_MUSB_GADGET))
		pdata->mode = USB_DR_MODE_PERIPHERAL;

	switch (pdata->mode) {
	case USB_DR_MODE_HOST:
		glue->phy_mode = PHY_MODE_USB_HOST;
		glue->role = USB_ROLE_HOST;
		break;
	case USB_DR_MODE_PERIPHERAL:
		glue->phy_mode = PHY_MODE_USB_DEVICE;
		glue->role = USB_ROLE_DEVICE;
		break;
	case USB_DR_MODE_OTG:
		glue->phy_mode = PHY_MODE_USB_OTG;
		glue->role = USB_ROLE_NONE;
		break;
	default:
		return dev_err_probe(dev, -EINVAL, "bad 'dr_mode' property\n");
	}
	mt6768_dbg(dev, "dr_mode=%d role=%s\n", pdata->mode,
		   mt6768_role_name(glue->role == USB_ROLE_HOST ? MT6768_ROLE_HOST :
				    glue->role == USB_ROLE_DEVICE ? MT6768_ROLE_DEVICE :
				    MT6768_ROLE_NONE));

	glue->phy = devm_of_phy_get_by_index(dev, np, 0);
	if (IS_ERR(glue->phy))
		return dev_err_probe(dev, PTR_ERR(glue->phy), "failed getting PHY\n");
	mt6768_dbg(dev, "PHY acquired (generic-tphy-v1, no invented tuning)\n");

	glue->usb_phy = usb_phy_generic_register();
	if (IS_ERR(glue->usb_phy))
		return dev_err_probe(dev, PTR_ERR(glue->usb_phy),
				     "failed registering usb-phy\n");

	glue->xceiv = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(glue->xceiv)) {
		ret = PTR_ERR(glue->xceiv);
		dev_err(dev, "failed getting usb-phy %d\n", ret);
		goto err_unregister_usb_phy;
	}

	platform_set_drvdata(pdev, glue);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	mt6768_dbg(dev, "runtime PM enabled (evidence-supported: mirrors generic glue)\n");

	ret = clk_bulk_prepare_enable(MT6768_MUSB_CLKS_NUM, glue->clks);
	if (ret)
		goto err_enable_clk;
	mt6768_dbg(dev, "clocks enabled\n");

	pinfo = mt6768_dev_info;
	pinfo.parent = dev;
	pinfo.res = pdev->resource;
	pinfo.num_res = pdev->num_resources;
	pinfo.data = pdata;
	pinfo.size_data = sizeof(*pdata);
	pinfo.fwnode = of_fwnode_handle(np);
	pinfo.of_node_reused = true;

	glue->musb_pdev = platform_device_register_full(&pinfo);
	if (IS_ERR(glue->musb_pdev)) {
		ret = PTR_ERR(glue->musb_pdev);
		dev_err(dev, "failed to register musb device: %d\n", ret);
		goto err_device_register;
	}
	mt6768_dbg(dev, "musb-hdrc child registered; UDC class entry expected under /sys/class/udc/\n");

	return 0;

err_device_register:
	clk_bulk_disable_unprepare(MT6768_MUSB_CLKS_NUM, glue->clks);
err_enable_clk:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
err_unregister_usb_phy:
	usb_phy_generic_unregister(glue->usb_phy);
	return ret;
}

static void mt6768_musb_remove(struct platform_device *pdev)
{
	struct mt6768_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb_pdev);
	usb_phy_generic_unregister(glue->usb_phy);
}

#ifdef CONFIG_OF
static const struct of_device_id mt6768_musb_match[] = {
	{ .compatible = "mediatek,mt6768-musb", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6768_musb_match);
#endif

static struct platform_driver mt6768_musb_driver = {
	.probe = mt6768_musb_probe,
	.remove = mt6768_musb_remove,
	.driver = {
		.name = "musb-mt6768",
		.of_match_table = of_match_ptr(mt6768_musb_match),
	},
};

module_platform_driver(mt6768_musb_driver);

MODULE_DESCRIPTION("MediaTek MT6768 MUSB integration glue (Track C)");
MODULE_AUTHOR("A32 Mainline Bring-up");
MODULE_LICENSE("GPL v2");
