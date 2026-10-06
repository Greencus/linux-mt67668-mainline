// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 diagnostic setup_arch checkpoints R0-R6 (BOOT-only, diagnostic-only,
 * console-free).
 *
 * Execution-boundary probes INSIDE setup_arch() for the ramcon-DTB lane
 * (bare 64K reserve kills deterministically; initcall-level proofs cannot
 * localize inside setup_arch). Each checkpoint is a single guarded call
 * site in arch/arm64/kernel/setup.c that issues a PSCI SYSTEM_RESET SMC
 * DIRECTLY and never returns: a reboot within seconds of boot proves the
 * kernel executed setup_arch() at least up to that call site, with NO
 * userspace, timers, UART, printk, or pstore involvement.
 *
 * Same direct-SMC primitive pattern as the proven
 * drivers/misc/diag-reboot-proof-l4d.c lane (read first):
 *   - arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET): public macro
 *     (include/linux/arm-smccc.h, always present on arm64 with
 *     CONFIG_HAVE_ARM_SMCCC=y), function ID from
 *     include/uapi/linux/psci.h, byte-identical to what psci_sys_reset()
 *     itself issues on the SMC conduit. No PSCI driver state required,
 *     no notifier registration, no probe ordering, no DT parsing — only
 *     firmware that is proven present (LK booted us, and the pinned DTB
 *     carries psci { compatible = "arm,psci-1.0"; method = "smc"; }).
 *   - Fail-closed spin: if the SMC ever returns (firmware ignored it —
 *     not expected), the CPU spins forever via cpu_relax() instead of
 *     returning into setup_arch(), so a silent board can never be
 *     misread as "died before Rn" after Rn was in fact reached.
 *   - No-timers/no-printk discipline: NO mdelay/udelay/msleep/schedule,
 *     NO pr_* (printk/logbuf/console are NOT usable at R0-R2; even at
 *     R3-R6 printk would add a confounding dependency). The ONLY
 *     observable is the reboot itself. A volatile touch of the marker
 *     keeps the A32-DIAG-CKPT-SETUPARCH string in the binary for the
 *     marker-in-binary gate without emitting anything at runtime.
 *
 * Each image enables EXACTLY ONE of CONFIG_A32_CKPT_R0..R6 (default n);
 * every other call site compiles out. Binary-layout caveat: the seven
 * images share the same code shape, but the active checkpoint's position
 * shifts surrounding layout by a few bytes, so Image hashes legitimately
 * differ R-to-R even before any source change.
 *
 * Compiled ONLY when at least one CONFIG_A32_CKPT_Rn=y (default n, see
 * drivers/misc/Kconfig + Makefile, exactly-one-y invariant documented
 * there). Never enable in production: the board reboots unconditionally
 * at boot.
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/processor.h>

#include <uapi/linux/psci.h>

static const char a32_ckpt_setuparch_marker[] __initconst =
	"A32-DIAG-CKPT-SETUPARCH";

void __init a32_ckpt(int id)
{
	struct arm_smccc_res res;

	/*
	 * Keep the marker in the binary without any runtime output: the
	 * compiler must materialize the marker's address for the asm
	 * input operand, so the string survives into .init.rodata for
	 * the marker-in-binary gate. No printk, no timers, no MMIO.
	 */
	asm volatile("" : : "r" (a32_ckpt_setuparch_marker + id) : "memory");
	/* Direct firmware reset: bypasses the restart-notifier chain. */
	arm_smccc_smc(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0, 0, 0, 0, 0, &res);
	/*
	 * Unreachable on working firmware (SYSTEM_RESET does not return).
	 * Fail closed: never return into setup_arch() claiming success.
	 */
	while (1)
		cpu_relax();
}
