// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 diagnostic reboot proof, LEVEL 4 DIRECT (BOOT-only, diagnostic-only,
 * console-free).
 *
 * A subsys_initcall that performs a short timer-free settle and then issues
 * a PSCI SYSTEM_RESET SMC DIRECTLY, bypassing the kernel restart-notifier
 * chain entirely. If the kernel reaches the subsys initcall level, the
 * phone reboots within seconds WITHOUT any userspace, timers, UART, or
 * pstore involvement — a console-free, handler-independent execution proof
 * that initcall completion reached level 4.
 *
 * Why direct-SMC instead of emergency_restart() (verified in-tree):
 *   emergency_restart() [kernel/reboot.c]
 *     -> machine_emergency_restart() -> machine_restart()
 *       -> do_kernel_restart() -> atomic_notifier chain [kernel/reboot.c]
 *   do_kernel_restart() explicitly "does nothing" when zero handlers are
 *   registered, and the sole in-tree reset provider (psci_sys_reset,
 *   registered by psci_0_2_set_functions() from psci_probe()) cannot be
 *   assumed present before L4/L6. A silent L4/L6 emergency_restart() image
 *   is therefore UNINTERPRETABLE (reached-without-handler vs never-reached
 *   look identical). This driver kills that ambiguity: the reset primitive
 *   below depends on NO notifier registration, NO probe ordering, NO DT
 *   parsing — only on firmware that is proven present (LK booted us, and
 *   the pinned DTB carries psci { compatible = "arm,psci-1.0";
 *   method = "smc"; }).
 *
 * Primitive feasibility (read from kernel/mainline sources, NOT assumed):
 *   - invoke_psci_fn: "static psci_fn *invoke_psci_fn" in
 *     drivers/firmware/psci/psci.c — file-static, no header declaration,
 *     no EXPORT_SYMBOL. NOT callable from drivers/misc.
 *   - psci_ops.system_reset: does NOT exist — struct psci_operations
 *     (include/linux/psci.h) carries only get_version/cpu_suspend/cpu_off/
 *     cpu_on/migrate/affinity_info/migrate_info_type. NOT callable.
 *   - arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET): FEASIBLE and used here.
 *     Public macro (include/linux/arm-smccc.h, always present on arm64
 *     with CONFIG_HAVE_ARM_SMCCC=y), function ID from
 *     include/uapi/linux/psci.h, byte-identical to what psci_sys_reset()
 *     itself issues on the SMC conduit. No PSCI driver state required,
 *     callable from any initcall context.
 *
 * If the SMC ever returns (firmware ignored it — not expected), the CPU
 * spins forever instead of returning to the initcall machinery, so a
 * silent board can never be misread as "died before L4" after L4 was in
 * fact reached: fail-closed, no false localizations.
 *
 * Compiled ONLY when CONFIG_A32_DIAG_REBOOT_PROOF_L4D=y (default n).
 * Never enable in production: the board reboots unconditionally at boot.
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/processor.h>

#include <uapi/linux/psci.h>

static const char a32_diag_reboot_proof_l4d_marker[] __initconst =
	"A32-DIAG-REBOOT-PROOF-L4D";

/*
 * Short settle with NO timer dependency: a plain volatile spin loop.
 * No mdelay/udelay/msleep/schedule — the reboot itself must not depend
 * on clocks, jiffies, or timers (a dead timer subsystem must not mask
 * the proof). The loop only lets boot console/DMA state drain for
 * a few milliseconds before the synchronous reset.
 */
static void __init a32_diag_reboot_proof_l4d_settle(void)
{
	volatile unsigned long i = 5000000UL;

	while (i > 0)
		i--;
}

static int __init a32_diag_reboot_proof_l4d_init(void)
{
	struct arm_smccc_res res;

	a32_diag_reboot_proof_l4d_settle();
	/* Harmless if unseen (no UART hw on this lane); pstore may catch it. */
	pr_emerg("%s: subsys_initcall reached, direct PSCI SYSTEM_RESET SMC now\n",
		 a32_diag_reboot_proof_l4d_marker);
	/* Direct firmware reset: bypasses the restart-notifier chain. */
	arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0, 0, 0, 0, 0, &res);
	/*
	 * Unreachable on working firmware (SYSTEM_RESET does not return).
	 * Fail closed: never return to initcall machinery claiming success.
	 * (The return keeps initcall form and silences -Wreturn-type; it
	 * never executes because the loop above cannot exit.)
	 */
	while (1)
		cpu_relax();
	return 0;
}
subsys_initcall(a32_diag_reboot_proof_l4d_init);
