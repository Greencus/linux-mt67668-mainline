// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 early TOPRGU WDT quiesce — DIAGNOSTIC ONLY, never production.
 *
 * Diagnostic-3: stop the MediaTek AP watchdog from an early_initcall, i.e.
 * earlier than any device probe (in particular long before the mtk-wdt
 * platform probe), so the Phase-7 read-out can discriminate "an early path
 * hangs and the armed WDT fires ~30 s later" from "Linux itself drives the
 * reset". Emits pr_emerg A32-WDT-DIAG lines with the pre/post WDT_MODE
 * value, proving both the earliest-observed state and that the quiesce
 * executed.
 *
 * Quiesce sequence (adapted, not invented): the downstream proven-safe
 * stop from kernel/downstream/drivers/watchdog/mediatek/wdt/common/wdt_v2/
 *   - mtk_wdt.h:49  MTK_WDT_MODE_KEY (0x22000000): every MODE write must
 *                   carry the key or the hardware ignores it.
 *   - mtk_wdt.h:62  MTK_WDT_MODE_ENABLE (0x0001): the EN bit.
 *   - mtk_wdt_v2.c:419-432 mtk_wdt_enable(WK_WDT_DIS): read MODE, OR the
 *                   KEY, clear ENABLE, sync-write back.
 *   - mtk_wdt_v2.c:267-275 mtk_wdt_mode_config(..., wdt_en == false):
 *                   same read-modify-write clearing-EN idiom.
 * Only WDT_MODE (offset 0x00) is written. No reset register (SWRST,
 * SWSYSRST), no REQ_MODE/REQ_IRQ_EN, no IRQ/LATCH/DEBUG_CTL register is
 * touched, so no reset is asserted and no reset-source routing changes.
 *
 * The whole file is compiled only when CONFIG_A32_EARLY_WDT_DIAG is set
 * (see drivers/watchdog/Makefile), and the body below is additionally
 * wrapped in #ifdef so the translation unit is empty in every other
 * configuration. That config is enabled solely by the Diagnostic-3
 * fragment into a throwaway O directory; production configs never set it.
 */

#ifdef CONFIG_A32_EARLY_WDT_DIAG

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

/* TOPRGU WDT_MODE register layout (see header comment for provenance). */
#define A32_WDT_MODE_OFF	0x00
#define A32_WDT_MODE_KEY	0x22000000
#define A32_WDT_MODE_ENABLE	0x0001

static int __init a32_early_wdt_quiesce(void)
{
	struct device_node *np;
	void __iomem *base;
	u32 pre, post;

	/* Same TOPRGU node the mtk-wdt probe binds to (reg 0x10007000). */
	np = of_find_compatible_node(NULL, NULL, "mediatek,mt6768-wdt");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "mediatek,mt6589-wdt");
	if (!np) {
		pr_emerg("A32-WDT-DIAG: early-quiesce: no toprgu DT node\n");
		return 0;
	}
	base = of_iomap(np, 0);
	of_node_put(np);
	if (!base) {
		pr_emerg("A32-WDT-DIAG: early-quiesce: of_iomap failed\n");
		return 0;
	}

	/* Earliest-observed state, before this kernel touches anything. */
	pre = readl(base + A32_WDT_MODE_OFF);
	pr_emerg("A32-WDT-DIAG: early-quiesce pre MODE=0x%08x\n", pre);

	/* Downstream-proven stop: keyed MODE write clearing EN. */
	post = (pre | A32_WDT_MODE_KEY) & ~A32_WDT_MODE_ENABLE;
	writel(post, base + A32_WDT_MODE_OFF);

	pr_emerg("A32-WDT-DIAG: early-quiesce post MODE=0x%08x\n",
		 readl(base + A32_WDT_MODE_OFF));
	pr_emerg("A32-WDT-DIAG: early-quiesce done\n");
	return 0;
}

/* Runs at early_initcall level: strictly earlier than the mtk-wdt device
 * probe (and earlier than downstream's own core_initcall iomap helper),
 * so the pre MODE value is the state handed off by LK/preloader code.
 */
early_initcall(a32_early_wdt_quiesce);

#endif /* CONFIG_A32_EARLY_WDT_DIAG */
