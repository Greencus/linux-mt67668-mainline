// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6768 CCCI platform driver: DT binding, resource acquisition, lifecycle.
 *
 * Evidence (downstream, read-only):
 * - mt6768.dts mdcldma@10014000 ("mediatek,mdcldma"): 4 regs (CLDMA_AO
 *   0x10014000, CLDMA_PDN 0x1021b000, AP_CCIF 0x10209000, MD_CCIF
 *   0x1020a000), 4 IRQs (SPI 180 CLDMA, SPI 164 CCIF0, SPI 165 CCIF1,
 *   SPI 75 WDT), mediatek,md_id = <0>, mediatek,cldma_capability = <6>.
 * - md_sys1_platform.c:195-231 irq_of_parse_and_map() idx0 = CLDMA,
 *   1 = CCIF0, 2 = CCIF1, 3 = WDT, all with IRQF_TRIGGER_NONE ("Device
 *   tree using none flag ... sensitivity has set at irq_of_parse").
 * - md_sys1_platform.c:54-60 clk_table[] names: infra-cldma-bclk,
 *   infra-ccif-ap/md, infra-ccif1-ap/md, infra-ccif2-ap/md (plus
 *   scp-sys-md1-main); missing clocks are tolerated (NULL, non-fatal).
 * - md_sys1_platform.c:77-112 CLDMA reset runs through the infracfg AO
 *   domain (INFRA_RST0/1_REG_AO/PD + CLDMA_CTRL).
 *
 * Mainline form: all addresses/IRQs/clocks come from DT (no hard-coded
 * physical addresses). The CCIF reset path and the modem boot-vector
 * enable are acquired here but NOT executed: starting the modem is a
 * HARDWARE gate (no firmware in this tree), so probe stops after
 * resource acquisition with the FSM in BOOT_WAITING_FOR_HS1.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "mtk_ccci.h"
#include "mtk_ccci_internal.h"

#define MTK_CCCI_DRV_NAME	"mtk-ccci"

static const char *const mtk_ccci_clk_names[] = {
	"scp-sys-md1-main",
	"infra-cldma-bclk",
	"infra-ccif-ap",
	"infra-ccif-md",
	"infra-ccif1-ap",
	"infra-ccif1-md",
	"infra-ccif2-ap",
	"infra-ccif2-md",
};

struct mtk_ccci_plat {
	struct device *dev;
	struct mtk_ccci_modem *md;
	struct mtk_cldma *cldma;
	struct clk *clks[ARRAY_SIZE(mtk_ccci_clk_names)];
	struct regmap *infracfg_ao;
	int ccif_irq0;
	int ccif_irq1;
	int wdt_irq;
	bool ccmni_up;
	bool tty_up;
};

static int mtk_ccci_plat_get_resources(struct platform_device *pdev,
				       struct mtk_ccci_plat *plat)
{
	struct device *dev = &pdev->dev;
	void __iomem *ao_base, *pdn_base, *ap_ccif, *md_ccif;
	struct resource *res;
	u32 md_id, capability;
	int i, irq;

	/* Regs idx0-3: CLDMA_AO, CLDMA_PDN, AP_CCIF, MD_CCIF. */
	ao_base = devm_platform_ioremap_resource(pdev, 0);
	pdn_base = devm_platform_ioremap_resource(pdev, 1);
	ap_ccif = devm_platform_ioremap_resource(pdev, 2);
	md_ccif = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(ao_base) || IS_ERR(pdn_base) ||
	    IS_ERR(ap_ccif) || IS_ERR(md_ccif))
		return -ENODEV;
	/* CCIF windows are reserved for the future CCIF transport
	 * (TODO-CCIF); silence unused-variable warnings by asserting
	 * the mapping succeeded, without touching the hardware.
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res || !ap_ccif || !md_ccif)
		return -ENODEV;

	if (of_property_read_u32(dev->of_node, "mediatek,md_id", &md_id) ||
	    md_id != MTK_CCCI_MD_SYS1) {
		dev_err(dev, "unsupported mediatek,md_id (only 0)\n");
		return -EINVAL;
	}
	if (!of_property_read_u32(dev->of_node, "mediatek,cldma_capability",
				  &capability) && capability != 6)
		dev_warn(dev, "unexpected cldma_capability %u (want 6)\n",
			 capability);

	/* IRQs idx0-3: CLDMA, CCIF0, CCIF1, WDT. The trigger is programmed
	 * by GIC forwarding; the driver requests with TRIGGER_NONE exactly
	 * like downstream. Only the CLDMA IRQ is serviced yet; CCIF/WDT
	 * numbers are stored for the future transports (TODO-CCIF).
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	plat->ccif_irq0 = platform_get_irq(pdev, 1);
	plat->ccif_irq1 = platform_get_irq(pdev, 2);
	plat->wdt_irq = platform_get_irq(pdev, 3);

	plat->md = mtk_ccci_modem_alloc(dev, md_id);
	if (!plat->md)
		return -ENOMEM;

	plat->cldma = mtk_cldma_create(dev, ao_base, pdn_base, irq);
	if (IS_ERR(plat->cldma)) {
		int ret = PTR_ERR(plat->cldma);

		plat->cldma = NULL;
		mtk_ccci_modem_free(plat->md);
		plat->md = NULL;
		return ret;
	}

	/* Clocks: downstream tolerates missing ones (NULL, non-fatal), so
	 * do the same; the scp clock has no mainline provider yet
	 * (TODO-SCP) and is expected to be absent.
	 */
	for (i = 0; i < (int)ARRAY_SIZE(mtk_ccci_clk_names); i++) {
		plat->clks[i] = devm_clk_get_optional(dev,
						      mtk_ccci_clk_names[i]);
		if (IS_ERR(plat->clks[i])) {
			dev_warn(dev, "clk %s unavailable: %pe\n",
				 mtk_ccci_clk_names[i], plat->clks[i]);
			plat->clks[i] = NULL;
		}
	}

	/* Infracfg AO syscon for the future CLDMA/CCIF reset path
	 * (TODO-CCIF-RESET): acquired, never written here.
	 */
	plat->infracfg_ao = syscon_regmap_lookup_by_phandle(dev->of_node,
							    "mediatek,infracfg_ao");
	if (IS_ERR(plat->infracfg_ao)) {
		dev_notice(dev, "no mediatek,infracfg_ao: reset path gated\n");
		plat->infracfg_ao = NULL;
	}

	return 0;
}

static int mtk_ccci_plat_probe(struct platform_device *pdev)
{
	struct mtk_ccci_plat *plat;
	int ret;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return -ENOMEM;
	plat->dev = &pdev->dev;
	platform_set_drvdata(pdev, plat);

	ret = mtk_ccci_plat_get_resources(pdev, plat);
	if (ret)
		return ret;

	ret = mtk_ccci_modem_register(plat->md);
	if (ret)
		goto err_cldma;

	ret = mtk_ccmni_register_all(plat->md->md_id);
	if (ret)
		goto err_modem;
	plat->ccmni_up = true;

	ret = mtk_ccci_tty_register();
	if (ret)
		goto err_ccmni;
	plat->tty_up = true;

	/* The modem is deliberately NOT started here: no firmware is
	 * present in this tree, so the FSM stays BOOT_WAITING_FOR_HS1
	 * (HARDWARE gate, see docs). Remove-time order mirrors init.
	 */
	dev_info(&pdev->dev, "MT6768 CCCI ready, modem held pre-HS1\n");
	return 0;

err_ccmni:
	if (plat->ccmni_up) {
		mtk_ccmni_unregister_all(plat->md->md_id);
		plat->ccmni_up = false;
	}
err_modem:
	mtk_ccci_modem_unregister(plat->md);
err_cldma:
	mtk_cldma_destroy(plat->cldma);
	plat->cldma = NULL;
	mtk_ccci_modem_free(plat->md);
	plat->md = NULL;
	return ret;
}

static void mtk_ccci_plat_remove(struct platform_device *pdev)
{
	struct mtk_ccci_plat *plat = platform_get_drvdata(pdev);

	if (plat->tty_up) {
		mtk_ccci_tty_unregister();
		plat->tty_up = false;
	}
	if (plat->ccmni_up) {
		mtk_ccmni_unregister_all(plat->md->md_id);
		plat->ccmni_up = false;
	}
	mtk_ccci_modem_unregister(plat->md);
	mtk_cldma_destroy(plat->cldma);
	plat->cldma = NULL;
	mtk_ccci_modem_free(plat->md);
	plat->md = NULL;
}

static const struct of_device_id mtk_ccci_of_match[] = {
	{ .compatible = "mediatek,mt6768-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_ccci_of_match);

static struct platform_driver mtk_ccci_driver = {
	.probe = mtk_ccci_plat_probe,
	.remove = mtk_ccci_plat_remove,
	.driver = {
		.name = MTK_CCCI_DRV_NAME,
		.of_match_table = mtk_ccci_of_match,
	},
};

static int __init mtk_ccci_mod_init(void)
{
	int ret;

	ret = mtk_ccci_core_init();
	if (ret)
		return ret;
	ret = platform_driver_register(&mtk_ccci_driver);
	if (ret)
		mtk_ccci_core_exit();
	return ret;
}
module_init(mtk_ccci_mod_init);

static void __exit mtk_ccci_mod_exit(void)
{
	platform_driver_unregister(&mtk_ccci_driver);
	mtk_ccci_core_exit();
}
module_exit(mtk_ccci_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("A32 mainline bring-up");
MODULE_DESCRIPTION("MediaTek MT6768 CCCI modem transport (clean-room port)");
