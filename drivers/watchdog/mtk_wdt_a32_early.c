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
 * SET-TIMEOUT-N LENGTH encoding (adapted, not invented): downstream
 *   - mtk_wdt_v2.c:218-238 mtk_wdt_set_time_out_value(value): ticks =
 *                   value * (1 << 6) (1 s = 64 ticks of 512xT32K), clamp
 *                   0x7FF, then LENGTH = (ticks << 5) | MTK_WDT_LENGTH_KEY.
 *   - mtk_wdt.h:88  MTK_WDT_LENGTH_KEY (0x0008): low-5-bit key field.
 *                   Mirrored by in-tree mtk_wdt_set_timeout()
 *                   (drivers/watchdog/mtk_wdt.c:288-290):
 *                   WDT_LENGTH_TIMEOUT((timeout-pretimeout) << 6) |
 *                   WDT_LENGTH_KEY, with WDT_LENGTH_TIMEOUT(n) = (n) << 5.
 * Only WDT_MODE (offset 0x00, STOP path) or WDT_LENGTH (offset 0x04,
 * EXTEND path) is ever written. No reset register (SWRST 0x14, SWSYSRST
 * 0x18, RESTART 0x08 reload), no REQ_MODE/REQ_IRQ_EN, no STATUS/NONRST
 * scratch bits are written, so no reset is asserted and no reset-source
 * routing changes. STATUS (0x0C) is read-only diagnostic readout.
 *
 * Two entry points, one shared core (no semantic drift between them):
 *   - a32_early_wdt_setup_arch_quiesce(): called from setup_arch() in
 *     arch/arm64/kernel/setup.c immediately after setup_machine_fdt(),
 *     i.e. after early_ioremap_init() and before ANY initcall level
 *     (early_initcall included). OF/of_iomap is NOT usable this early
 *     (unflatten_device_tree() runs much later in setup_arch), so MMIO
 *     uses early_ioremap() of the fixed TOPRGU base 0x10007000, one page
 *     (covers MODE 0x00/LENGTH 0x04/STATUS 0x0C; NONRST 0x20/0x24 not
 *     needed here). Base corroboration: minimal DT watchdog@10007000 reg
 *     <0 0x10007000 0 0x100> (mt6768.dtsi:1153-1157, compat
 *     mediatek,mt6768-wdt), pmOS DT toprgu@10007000 reg 0x10007000/0x1000,
 *     downstream MTK_WDT_BASE=toprgu_base of_iomap'd from the same DT reg
 *     (mtk_wdt_v2.c:452-459). Mapping is released with early_iounmap()
 *     before return, strictly before setup_arch's early_ioremap_reset().
 *   - a32_early_wdt_quiesce() early_initcall fallback: kept verbatim in
 *     behavior (of_iomap + legacy A32-WDT-DIAG pre/post lines), now routed
 *     through the same core so STOP/EXTEND semantics cannot diverge. No
 *     conflict: setup_arch runs first (start_kernel -> setup_arch long
 *     before do_initcalls), and the fallback's keyed EN-clear is
 *     idempotent if STOP already ran.
 *
 * The whole file is compiled only when CONFIG_A32_EARLY_WDT_DIAG is set
 * (see drivers/watchdog/Makefile), and the body below is additionally
 * wrapped in #ifdef so the translation unit is empty in every other
 * configuration. That config is enabled solely by the Diagnostic-3
 * fragment into a throwaway O directory; production configs never set it.
 *
 * DIAGNOSTIC-ONLY shx-latch addition: when CONFIG_A32_EARLY_WDT_RESTART_PING
 * is set (default n; shx-latch throwaway O-dir only), the EXTEND branch
 * above additionally issues one WDT_RESTART reload (offset 0x08, key 0x1971;
 * provenance at A32_WDT_RESTART_KEY) between dsb sy barriers, plus a
 * distinct EXTEND+RESTART printk. STOP is untouched and the option unset
 * leaves the EXTEND branch exactly as before.
 */

#ifdef CONFIG_A32_EARLY_WDT_DIAG

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/barrier.h>
#include <asm/early_ioremap.h>

/* TOPRGU WDT_MODE register layout (see header comment for provenance). */
#define A32_WDT_MODE_OFF	0x00
#define A32_WDT_LENGTH_OFF	0x04
/* WDT_RESTART reload offset/key (shx-latch EXTEND+RESTART ping ONLY).
 * Sourced, not invented:
 *   - kernel/mainline/drivers/watchdog/mtk_wdt.c:42 WDT_RST 0x08,
 *     mtk_wdt.c:43 WDT_RST_RELOAD 0x1971, mtk_wdt.c:259-264
 *     mtk_wdt_ping() iowrite32(WDT_RST_RELOAD, wdt_base + WDT_RST);
 *   - references/linux/mt6768-mainline-u-boot/drivers/watchdog/
 *     mtk_wdt.c:17 MTK_WDT_RESTART 0x08, :34 WDT_RESTART_KEY 0x1971,
 *     :46/:65 writel(WDT_RESTART_KEY, priv->base + MTK_WDT_RESTART);
 *   - kernel/downstream/drivers/watchdog/mediatek/wdt/common/wdt_v2/
 *     mtk_wdt.h:22 MTK_WDT_RESTART (MTK_WDT_BASE+0x0008),
 *     mtk_wdt.h:91-92 MTK_WDT_RESTART_KEY (0x1971).
 * Never used unless CONFIG_A32_EARLY_WDT_RESTART_PING is set (shx-latch
 * O-dir only). No MODE/SWRST/SWSYSRST/REQ_MODE/STATUS/NONRST write exists
 * anywhere in this file. */
#define A32_WDT_RESTART_OFF	0x08
#define A32_WDT_RESTART_KEY	0x1971
#define A32_WDT_STATUS_OFF	0x0C
#define A32_WDT_MODE_KEY	0x22000000
#define A32_WDT_MODE_ENABLE	0x0001
#define A32_WDT_LENGTH_KEY	0x8

/* Fixed TOPRGU base for the setup_arch entry (see header comment). */
#define A32_WDT_TOPRGU_PHYS	0x10007000
#define A32_WDT_MAP_SIZE	0x1000

/* Shared core: STOP when CONFIG_A32_EARLY_WDT_TIMEOUT == 0, else extend
 * LENGTH to N seconds. MODE/STATUS are never written here. */
static void __init a32_early_wdt_core(void __iomem *base, const char *via)
{
	u32 mode, len, status;

	pr_emerg("A32-EARLY-WDT: ENTER via %s (timeout=%d)\n",
		 via, CONFIG_A32_EARLY_WDT_TIMEOUT);

	/* Earliest-observed state, before this kernel touches anything. */
	mode = readl(base + A32_WDT_MODE_OFF);
	len = readl(base + A32_WDT_LENGTH_OFF);
	status = readl(base + A32_WDT_STATUS_OFF);
	pr_emerg("A32-EARLY-WDT: TOPRGU MODE=0x%08x LENGTH=0x%08x STATUS=0x%08x\n",
		 mode, len, status);

	if (CONFIG_A32_EARLY_WDT_TIMEOUT == 0) {
		u32 post;

		/* Downstream-proven stop: keyed MODE write clearing EN. */
		post = (mode | A32_WDT_MODE_KEY) & ~A32_WDT_MODE_ENABLE;
		writel(post, base + A32_WDT_MODE_OFF);

		pr_emerg("A32-EARLY-WDT: STOP COMPLETE MODE=0x%08x\n",
			 readl(base + A32_WDT_MODE_OFF));
	} else {
		unsigned int t = CONFIG_A32_EARLY_WDT_TIMEOUT;
		u32 ticks, length;

		/* Downstream-proven LENGTH encoding (see header comment).
		 * Clamp to the driver-valid window [2, 31]. */
		if (t < 2)
			t = 2;
		if (t > 31)
			t = 31;
		ticks = t * (1 << 6);
		if (ticks > 0x7FF)
			ticks = 0x7FF;
		length = (ticks << 5) | A32_WDT_LENGTH_KEY;
		writel(length, base + A32_WDT_LENGTH_OFF);
#ifdef CONFIG_A32_EARLY_WDT_RESTART_PING
		/* DIAGNOSTIC-ONLY shx-latch: one RESTART reload so the new
		 * LENGTH latches now instead of at the next ping (see the
		 * A32_WDT_RESTART_KEY provenance above). EXTEND-branch only;
		 * compiled out in every other config, so STOP and plain
		 * EXTEND behavior are unchanged there. */
		dsb(sy);
		writel(A32_WDT_RESTART_KEY, base + A32_WDT_RESTART_OFF);
		dsb(sy);

		pr_emerg("A32-EARLY-WDT: EXTEND+RESTART COMPLETE LENGTH=0x%08x MODE=0x%08x\n",
			 readl(base + A32_WDT_LENGTH_OFF),
			 readl(base + A32_WDT_MODE_OFF));
#endif

		pr_emerg("A32-EARLY-WDT: EXTEND COMPLETE LENGTH=0x%08x MODE=0x%08x\n",
			 readl(base + A32_WDT_LENGTH_OFF),
			 readl(base + A32_WDT_MODE_OFF));
	}
}

/* setup_arch entry: earliest-source-proven-safe neutralization, called
 * after setup_machine_fdt() (early_ioremap usable, no OF yet). */
void __init a32_early_wdt_setup_arch_quiesce(void)
{
	void __iomem *base;

	base = early_ioremap(A32_WDT_TOPRGU_PHYS, A32_WDT_MAP_SIZE);
	if (!base) {
		pr_emerg("A32-EARLY-WDT: ENTER via setup_arch (timeout=%d)\n",
			 CONFIG_A32_EARLY_WDT_TIMEOUT);
		pr_emerg("A32-EARLY-WDT: early_ioremap failed\n");
		return;
	}
	a32_early_wdt_core(base, "setup_arch");
	early_iounmap(base, A32_WDT_MAP_SIZE);
}

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

	/* Shared core (idempotent if the setup_arch entry already ran). */
	a32_early_wdt_core(base, "early_initcall");

	post = readl(base + A32_WDT_MODE_OFF);
	pr_emerg("A32-WDT-DIAG: early-quiesce post MODE=0x%08x\n", post);
	pr_emerg("A32-WDT-DIAG: early-quiesce done\n");
	iounmap(base);
	return 0;
}

/* Runs at early_initcall level: strictly earlier than the mtk-wdt device
 * probe (and earlier than downstream's own core_initcall iomap helper),
 * so the pre MODE value is the state handed off by LK/preloader code.
 */
early_initcall(a32_early_wdt_quiesce);

#endif /* CONFIG_A32_EARLY_WDT_DIAG */
