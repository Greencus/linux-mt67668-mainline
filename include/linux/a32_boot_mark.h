// SPDX-License-Identifier: GPL-2.0-only
/*
 * A32 diagnostic boot breadcrumb marker (Phase-3B tape, K0-K2 + K4-K5).
 *
 * DIAGNOSTIC-ONLY. Reuses the 95ac096db tape mechanism verbatim in style:
 * a single printk line with the "A32-BREADCRUMB:" prefix, extended with
 * the "A32-BOOT K%d" milestone token. K3 (watchdog probe) and K6 (/init)
 * belong to parallel lanes and are intentionally NOT emitted here.
 *
 * Survival path (no new code needed): printk writes to the kernel logbuf;
 * once the ramoops backend probes (compatible "ramoops" @ 0x4d010000,
 * size 0xe0000, console-size 0x40000; diagnostic kernel has
 * PSTORE_CONSOLE=y via scripts/diag-pstore.fragment), console records
 * persist across reset in reserved RAM and are chunk-dumped on the
 * FOLLOWING boot. Markers emitted before the ramoops probe (K0/K1/K4,
 * K2-entry) are replayed from logbuf into the console zone at probe
 * time, so this helper deliberately performs NO direct pstore dereference.
 *
 * Safety (soft-fail by construction, can NEVER hang boot):
 * - printk only: no locks, no waiting, no sleeping, no I/O, no MMIO.
 * - the label pointer is null-checked; worst case the line prints "?".
 * - safe with interrupts disabled and before console_init() (logbuf
 *   buffering; proven precedent: kaslr_init() pr_info in setup_arch).
 * - no control-flow change at any call site; callers ignore any result.
 */

#ifndef _LINUX_A32_BOOT_MARK_H
#define _LINUX_A32_BOOT_MARK_H

#include <linux/printk.h>

static inline void a32_boot_mark(int k, const char *s)
{
	if (!s)
		s = "?";
	pr_emerg("A32-BREADCRUMB: A32-BOOT K%d %s\n", k, s);
}

#endif /* _LINUX_A32_BOOT_MARK_H */
