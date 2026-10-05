// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 diagnostic reboot proof (BOOT-only, diagnostic-only, console-free).
 *
 * A late_initcall that performs a short timer-free settle and then calls
 * emergency_restart() synchronously. If the kernel reaches the late
 * initcall level, the phone reboots within seconds WITHOUT any userspace,
 * timers, UART, or pstore involvement — a console-free execution proof
 * that initcall completion was reached.
 *
 * Reboot path (verified in-tree, no DT dependency):
 *   emergency_restart() [kernel/reboot.c]
 *     -> machine_emergency_restart() == machine_restart(NULL)
 *          [include/asm-generic/emergency-restart.h]
 *       -> do_kernel_restart() -> atomic_notifier chain [kernel/reboot.c]
 *         -> psci_sys_reset() -> PSCI SYSTEM_RESET / SYSTEM_RESET2 SMC
 *              [drivers/firmware/psci/psci.c]
 * The PSCI restart handler is registered by psci_0_2_set_functions()
 * from psci_dt_init(), which runs in setup_arch()
 * [arch/arm64/kernel/setup.c] — i.e. in start_kernel BEFORE rest_init
 * and therefore before every initcall level (early..late). At
 * late_initcall (level 7, the last level) it is guaranteed registered.
 *
 * Compiled ONLY when CONFIG_A32_DIAG_REBOOT_PROOF=y (default n).
 * Never enable in production: the board reboots unconditionally at boot.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/reboot.h>

static const char a32_diag_reboot_proof_marker[] __initconst =
	"A32-DIAG-REBOOT-PROOF";

/*
 * Short settle with NO timer dependency: a plain volatile spin loop.
 * No mdelay/udelay/msleep/schedule — the reboot itself must not depend
 * on clocks, jiffies, or timers (a dead timer subsystem must not mask
 * the proof). The loop only lets late-boot console/DMA state drain for
 * a few milliseconds before the synchronous restart.
 */
static void __init a32_diag_reboot_proof_settle(void)
{
	volatile unsigned long i = 5000000UL;

	while (i > 0)
		i--;
}

static int __init a32_diag_reboot_proof_init(void)
{
	a32_diag_reboot_proof_settle();
	/* Harmless if unseen (no UART hw on this lane); pstore may catch it. */
	pr_emerg("%s: late_initcall reached, emergency_restart() now\n",
		 a32_diag_reboot_proof_marker);
	emergency_restart();
	/* Unreachable on a working restart path; return keeps initcall form. */
	return 0;
}
late_initcall(a32_diag_reboot_proof_init);
