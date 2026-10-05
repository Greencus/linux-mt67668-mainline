// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 diagnostic reboot proof, LEVEL 6 (BOOT-only, diagnostic-only,
 * console-free).
 *
 * A device_initcall that performs a short timer-free settle and then calls
 * emergency_restart() synchronously. If the kernel reaches the device
 * initcall level, the phone reboots within seconds WITHOUT any userspace,
 * timers, UART, or pstore involvement — a console-free execution proof
 * that initcall completion reached level 6.
 *
 * Binary-search DOWN one level from the late_initcall (level 7) proof
 * (kernel/mainline/drivers/misc/diag-reboot-proof.c): if the L7 image is
 * silent, this L6 image splits the window — a reboot here means the hang
 * is between device_initcalls and late_init (L6-L7 window); silence here
 * means the hang is at or before L6 (shift to subsys L4 next, etc.).
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
 * device_initcall (level 6) it is guaranteed registered.
 *
 * Compiled ONLY when CONFIG_A32_DIAG_REBOOT_PROOF_L6=y (default n).
 * Never enable in production: the board reboots unconditionally at boot.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/reboot.h>

static const char a32_diag_reboot_proof_l6_marker[] __initconst =
	"A32-DIAG-REBOOT-PROOF-L6";

/*
 * Short settle with NO timer dependency: a plain volatile spin loop.
 * No mdelay/udelay/msleep/schedule — the reboot itself must not depend
 * on clocks, jiffies, or timers (a dead timer subsystem must not mask
 * the proof). The loop only lets boot console/DMA state drain for
 * a few milliseconds before the synchronous restart.
 */
static void __init a32_diag_reboot_proof_l6_settle(void)
{
	volatile unsigned long i = 5000000UL;

	while (i > 0)
		i--;
}

static int __init a32_diag_reboot_proof_l6_init(void)
{
	a32_diag_reboot_proof_l6_settle();
	/* Harmless if unseen (no UART hw on this lane); pstore may catch it. */
	pr_emerg("%s: device_initcall reached, emergency_restart() now\n",
		 a32_diag_reboot_proof_l6_marker);
	emergency_restart();
	/* Unreachable on a working restart path; return keeps initcall form. */
	return 0;
}
device_initcall(a32_diag_reboot_proof_l6_init);
