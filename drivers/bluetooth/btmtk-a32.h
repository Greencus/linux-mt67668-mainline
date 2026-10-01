/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * btmtk-a32.h -- A32 (MT6768/MT6631-class CONNSYS) Bluetooth HCI transport
 * definitions shared between the kernel driver (btmtk-a32.c) and the
 * userspace offline unit tests (scripts/tests/bt/).
 *
 * Dual-mode header: compiles under the kernel (__KERNEL__ defined by
 * kbuild) and under a plain userspace gcc (stdint.h-based fallbacks).
 * Contains ONLY pure packet/framing/name helpers -- no kernel APIs, no
 * libc calls -- so the offline tests validate the exact logic the
 * driver ships.
 *
 * Downstream evidence (all read-only, kernel/downstream/):
 * - WMT-over-HCI header 01 6F FC:
 *     drivers/misc/mediatek/connectivity/bt/mt66xx/connac2/btmtk_mt66xx.c:98
 *     (WMT_OVER_HCI_CMD_HDR[] = { 0x01, 0x6F, 0xFC, 0x00 }) and concrete
 *     commands at :1222, :1233, :1258.
 * - H4 rx demux table (ACL/SCO/EVENT):
 *     connac2/btmtk_main.c:177-179 (mtk_recv_pkts[]:
 *       H4_RECV_ACL -> btmtk_recv_acl,
 *       H4_RECV_SCO -> hci_recv_frame,
 *       H4_RECV_EVENT -> btmtk_recv_event),
 *     reassembly helper h4_recv_buf at btmtk_main.c:43,
 *     delivery via hci_recv_frame at btmtk_main.c:416,510,529.
 * - Firmware loader request_firmware(&fw_entry, bin_name, dev) with
 *   retry: connac2/btmtk_main.c:598-613 (btmtk_load_code_from_bin).
 * - A32 firmware set (short basenames, no mediatek/ prefix): vendor
 *   manifest references/android/vendor/vendor_samsung_a32/a32-vendor.mk
 *   :590-:593.
 * - BTIF is the COMPILE-TIME default transport:
 *     connac2/Makefile:28 (-DCHIP_IF_BTIF),
 *     connac2/Makefile.ce:23, selector connac2/btmtk_chip_if.h:25.
 * - Wakeup-IRQ shape (§3):
 *     connac2/btmtk_irq.c:39-40 (bt_irq_ctrl table: "BTIF_WAKEUP_IRQ" /
 *     "BGF_SW_IRQ"), :198-199 (wakeup = FW-has-data, SW = fw assert/log),
 *     :211-226 (handler: match irq_num, disable_irq, set rx_ind /
 *     bgf2ap_ind + clear sleep_flag, wake_up tx_waitq, else IRQ_NONE),
 *     :239-290 (bt_request_irq: DTS-resolved via irq_of_parse_and_map on
 *     compatible "mediatek,bt", index 0 = wakeup / 1 = SW,
 *     IRQF_TRIGGER_HIGH|IRQF_SHARED), :303-352 (enable/disable with
 *     spinlock + active flag, disable_irq_nosync on the disable leg);
 *     request-then-mask at connac2/btmtk_mt66xx.c:1139-1149 (both IRQs
 *     requested, both disabled immediately), freed at :1161-1164/:1190;
 *     numeric IDs MT_BGF2AP_BTIF_WAKEUP_IRQ_ID 312 / MT_BGF2AP_SW_IRQ_ID
 *     271 are marked temp-only (connac2/btmtk_btif.h:61-62) and are
 *     NOT authoritative; PSM sleep/wake at connac2/btmtk_btif_main.c:
 *     :1306-1347 (SLEEP wakeup: disable IRQ, FW-own clear, -> NORMAL_TR),
 *     :1356-1360 (rx_ind consumed, "wakeup by BTIF_WAKEUP_IRQ"),
 *     :1420-1438 (NORMAL sleep entry: FW-own set, enable IRQ, -> SLEEP);
 *     wakeup_source "bt_psm" lock at connac2/btmtk_btif.h:456-467;
 *     BTIF owner "CONSYS_BT" (connac2/btmtk_btif_main.c:46); sibling
 *     DTS-resolved request_irq + enable_irq_wake precedent at
 *     wmt_drv/common_main/platform/wmt_plat_alps.c:619-637.
 */

#ifndef __BTMTK_A32_H
#define __BTMTK_A32_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/errno.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#endif

/* H4 packet indicators on the host<->controller stream. These are the
 * same H4 type bytes the downstream h4_recv_buf demuxes on
 * (btmtk_main.c:177-179). SCO is included ONLY because the downstream
 * demux table has an explicit H4_RECV_SCO entry (:178).
 */
#define BTMTK_A32_H4_CMD	0x01
#define BTMTK_A32_H4_ACL	0x02
#define BTMTK_A32_H4_SCO	0x03
#define BTMTK_A32_H4_EVT	0x04

/* HCI packet types as seen by the kernel HCI core (hci.h). */
#define BTMTK_A32_PKT_CMD	HCI_COMMAND_PKT		/* 0x01 */
#define BTMTK_A32_PKT_ACL	HCI_ACLDATA_PKT		/* 0x02 */
#define BTMTK_A32_PKT_SCO	HCI_SCODATA_PKT		/* 0x03 */
#define BTMTK_A32_PKT_EVT	HCI_EVENT_PKT		/* 0x04 */
#ifndef __KERNEL__
/* Userspace fallback: keep the header self-contained when hci.h is
 * unavailable. Values match include/net/bluetooth/hci.h.
 */
#ifndef HCI_COMMAND_PKT
#define HCI_COMMAND_PKT		0x01
#define HCI_ACLDATA_PKT		0x02
#define HCI_SCODATA_PKT		0x03
#define HCI_EVENT_PKT		0x04
#endif
#endif

/* WMT-over-HCI vendor command header (btmtk_mt66xx.c:98):
 * H4 CMD indicator, OGF=0x3F/OCF=0x36 vendor opcode 0xFC6F (LE: 6F FC),
 * followed by one total-parameter-length byte, then the WMT payload.
 */
#define BTMTK_A32_WMT_HDR_LEN	4
#define BTMTK_A32_WMT_OP_LSB	0x6F
#define BTMTK_A32_WMT_OP_MSB	0xFC

/* HCI header sizes (excluding the H4 type byte). */
#define BTMTK_A32_CMD_HDR_LEN	3	/* opcode LE16 + plen u8 */
#define BTMTK_A32_ACL_HDR_LEN	4	/* handle+flags LE16 + dlen LE16 */
#define BTMTK_A32_SCO_HDR_LEN	3	/* handle+flags LE16 + dlen u8 */
#define BTMTK_A32_EVT_HDR_LEN	2	/* evt u8 + plen u8 */

/* Firmware basenames requested from the loader. SHORT names with NO
 * mediatek/ prefix -- per the downstream loader which passes the plain
 * bin_name to request_firmware() (btmtk_main.c:605), and per the A32
 * vendor manifest (a32-vendor.mk:590-593). The BT_FW.cfg replay list
 * is post-HCI (autobt) and is NOT request_firmware() material; it is
 * listed here for inventory completeness only.
 */
#define BTMTK_A32_FW_BT_RAM	"soc1_0_ram_bt_1a_1_hdr.bin"
#define BTMTK_A32_FW_PATCH_MCU	"soc1_0_patch_mcu_1a_1_hdr.bin"
#define BTMTK_A32_FW_RAM_MCU	"soc1_0_ram_mcu_1a_1_hdr.bin"
#define BTMTK_A32_FW_CFG	"BT_FW.cfg"
#define BTMTK_A32_FW_COUNT	3	/* request_firmware() set */
#define BTMTK_A32_FW_RETRY	10	/* mirror downstream retry budget */

/* ------------------------------------------------------------------
 * §3 wakeup-IRQ contract (pure part: indices, DT names, PSM states).
 *
 * Downstream resolves both IRQs from DT, never from drum-tight
 * constants: bt_request_irq() maps index 0 (wakeup) / 1 (SW) of the
 * compatible "mediatek,bt" node (btmtk_irq.c:249-269). This driver
 * keeps that contract via platform_get_irq_byname() on the
 * interrupt-names below; the DT fragment carries the names (+
 * wakeup-source) while the numeric line stays TODO-HWIRQ until the
 * actual A32 CONNSYS->GIC/sysirq routing is confirmed on hardware
 * (the 312/271 IDs are downstream-marked "temp" and MUST NOT be used).
 * ------------------------------------------------------------------
 */

/* Downstream DTS indices (btmtk_irq.c:249-269: index 0 = BTIF_WAKEUP,
 * index 1 = SW). Order only; no GIC numbers are implied.
 */
#define BTMTK_A32_IRQ_WAKEUP	0
#define BTMTK_A32_IRQ_SW	1
#define BTMTK_A32_IRQ_COUNT	2

/* DT interrupt-names for this driver's binding. The request_irq() devname
 * args keep the downstream .name strings (btmtk_irq.c:39-40) so traces
 * match: "BTIF_WAKEUP_IRQ" / "BGF_SW_IRQ".
 */
#define BTMTK_A32_IRQ_NAME_WAKEUP	"wakeup"
#define BTMTK_A32_IRQ_NAME_SW		"sw"
#define BTMTK_A32_IRQ_DEVNAME_WAKEUP	"BTIF_WAKEUP_IRQ"
#define BTMTK_A32_IRQ_DEVNAME_SW	"BGF_SW_IRQ"

/* Wakeup-source hold (ms) per FW-wakeup event. Downstream holds the
 * "bt_psm" wake lock across the whole TX/wakeup window (__pm_stay_awake
 * + qos, btmtk_btif.h:426-454); the mainline equivalent here is a bounded
 * __pm_wakeup_event() per IRQ-thread drain.
 */
#define BTMTK_A32_WAKE_HOLD_MS	100

/* Power-save states mirroring the downstream PSM (btmtk_btif_main.c
 * PSM_ST_SLEEP / PSM_ST_NORMAL_TR).
 */
enum btmtk_a32_psm {
	BTMTK_A32_PSM_SLEEP = 0,
	BTMTK_A32_PSM_NORMAL = 1,
};

/* PSM input events. */
enum btmtk_a32_psm_ev {
	BTMTK_A32_EV_WAKE_IRQ = 0,	/* FW wakeup IRQ (has data) */
	BTMTK_A32_EV_SLEEP_REQ,		/* host asks to enter sleep */
	BTMTK_A32_EV_WAKE_REQ,		/* host forces awake */
	BTMTK_A32_EV_FW_OWN_FAIL,	/* FW-own clear failed mid-wakeup */
};

/**
 * btmtk_a32_psm_next - PSM transition, mirroring downstream semantics.
 * @st: current state.
 * @ev: input event.
 *
 * - WAKE_IRQ always ends awake: the handler clears sleep_flag even when
 *   it fires mid-SLEEP-wakeup (btmtk_irq.c:212-218) and the thread's
 *   NORMAL_TR leg consumes rx_ind without leaving NORMAL
 *   (btmtk_btif_main.c:1356-1360).
 * - SLEEP_REQ enters sleep from NORMAL only (btmtk_btif_main.c:1420-1438:
 *   FW-own set + enable IRQ + -> SLEEP); already asleep is a no-op.
 * - WAKE_REQ always ends awake (btmtk_set_wakeup analogue).
 * - FW_OWN_FAIL leaves the state UNCHANGED: the SLEEP wakeup leg
 *   re-enables the IRQ and breaks out without changing psm->state
 *   (btmtk_btif_main.c:1323-1342).
 *
 * Return: the next state.
 */
static inline enum btmtk_a32_psm
btmtk_a32_psm_next(enum btmtk_a32_psm st, enum btmtk_a32_psm_ev ev)
{
	switch (ev) {
	case BTMTK_A32_EV_WAKE_IRQ:
		return BTMTK_A32_PSM_NORMAL;
	case BTMTK_A32_EV_SLEEP_REQ:
		return st == BTMTK_A32_PSM_NORMAL ?
			BTMTK_A32_PSM_SLEEP : st;
	case BTMTK_A32_EV_WAKE_REQ:
		return BTMTK_A32_PSM_NORMAL;
	case BTMTK_A32_EV_FW_OWN_FAIL:
		return st;
	}
	return st;
}

/**
 * btmtk_a32_is_wmt_cmd - test for the WMT-over-HCI vendor header.
 * @buf: H4 stream bytes starting at a packet boundary.
 * @len: bytes available in @buf.
 *
 * Return: 1 when @buf holds 01 6F FC (btmtk_mt66xx.c:98), else 0.
 */
static inline int btmtk_a32_is_wmt_cmd(const u8 *buf, unsigned int len)
{
	if (!buf || len < (unsigned int)BTMTK_A32_WMT_HDR_LEN)
		return 0;
	return buf[0] == (u8)BTMTK_A32_H4_CMD &&
	       buf[1] == (u8)BTMTK_A32_WMT_OP_LSB &&
	       buf[2] == (u8)BTMTK_A32_WMT_OP_MSB;
}

/**
 * btmtk_a32_cmd_frame_len - total H4 length of an HCI CMD frame.
 * @buf: bytes starting at the H4 type byte; @len available.
 *
 * Return: full frame length (1 + 3 + plen), or 0 when the header is
 * incomplete (need more bytes) or not a CMD packet (-1 via 0/-1
 * convention: 0 = need more data).
 */
static inline int btmtk_a32_cmd_frame_len(const u8 *buf, unsigned int len)
{
	if (!buf || len < 1u + (unsigned int)BTMTK_A32_CMD_HDR_LEN)
		return 0;
	if (buf[0] != (u8)BTMTK_A32_H4_CMD)
		return -EINVAL;
	return 1 + BTMTK_A32_CMD_HDR_LEN + buf[3];
}

/**
 * btmtk_a32_acl_frame_len - total H4 length of an HCI ACL frame.
 */
static inline int btmtk_a32_acl_frame_len(const u8 *buf, unsigned int len)
{
	unsigned int dlen;

	if (!buf || len < 1u + (unsigned int)BTMTK_A32_ACL_HDR_LEN)
		return 0;
	if (buf[0] != (u8)BTMTK_A32_H4_ACL)
		return -EINVAL;
	dlen = (unsigned int)buf[3] | ((unsigned int)buf[4] << 8);
	return 1 + BTMTK_A32_ACL_HDR_LEN + (int)dlen;
}

/**
 * btmtk_a32_sco_frame_len - total H4 length of an HCI SCO frame.
 * (Downstream demux evidences SCO on this stream: btmtk_main.c:178.)
 */
static inline int btmtk_a32_sco_frame_len(const u8 *buf, unsigned int len)
{
	if (!buf || len < 1u + (unsigned int)BTMTK_A32_SCO_HDR_LEN)
		return 0;
	if (buf[0] != (u8)BTMTK_A32_H4_SCO)
		return -EINVAL;
	return 1 + BTMTK_A32_SCO_HDR_LEN + buf[3];
}

/**
 * btmtk_a32_evt_frame_len - total H4 length of an HCI EVENT frame.
 */
static inline int btmtk_a32_evt_frame_len(const u8 *buf, unsigned int len)
{
	if (!buf || len < 1u + (unsigned int)BTMTK_A32_EVT_HDR_LEN)
		return 0;
	if (buf[0] != (u8)BTMTK_A32_H4_EVT)
		return -EINVAL;
	return 1 + BTMTK_A32_EVT_HDR_LEN + buf[2];
}

/**
 * btmtk_a32_rx_frame_len - demux dispatch: total H4 length for the
 * packet at @buf, mirroring the downstream mtk_recv_pkts[] table
 * (btmtk_main.c:177-179: ACL / SCO / EVENT).
 *
 * Return: >0 full length, 0 need-more-data, -EINVAL unknown type.
 */
static inline int btmtk_a32_rx_frame_len(const u8 *buf, unsigned int len)
{
	if (!buf || len < 1u)
		return 0;
	switch (buf[0]) {
	case (u8)BTMTK_A32_H4_ACL:
		return btmtk_a32_acl_frame_len(buf, len);
	case (u8)BTMTK_A32_H4_SCO:
		return btmtk_a32_sco_frame_len(buf, len);
	case (u8)BTMTK_A32_H4_EVT:
		return btmtk_a32_evt_frame_len(buf, len);
	default:
		return -EINVAL;
	}
}

/**
 * btmtk_a32_encode_cmd - build an H4 HCI CMD frame.
 * @opcode: HCI opcode (stored little-endian).
 * @params: command parameters (may be NULL when @plen is 0).
 * @plen: parameter length.
 * @out: destination buffer; @outlen its size.
 *
 * Return: bytes written (1 + 3 + plen), or -EINVAL/-ENOSPC on error.
 */
static inline int btmtk_a32_encode_cmd(u16 opcode, const u8 *params, u8 plen,
				       u8 *out, unsigned int outlen)
{
	unsigned int need, i;

	need = 1u + (unsigned int)BTMTK_A32_CMD_HDR_LEN + plen;
	if (!out || outlen < need)
		return -ENOSPC;
	if (plen && !params)
		return -EINVAL;
	out[0] = (u8)BTMTK_A32_H4_CMD;
	out[1] = (u8)(opcode & 0xffu);
	out[2] = (u8)((opcode >> 8) & 0xffu);
	out[3] = plen;
	for (i = 0; i < plen; i++)
		out[4 + i] = params[i];
	return (int)need;
}

/**
 * btmtk_a32_encode_wmt_cmd - build a WMT-over-HCI vendor command frame
 * (01 6F FC <dlen> <payload...>) per btmtk_mt66xx.c:98,1222.
 * @payload: WMT payload AFTER the length byte (e.g. 01 03 01 00 04).
 * @payload_len: payload length (fits in one length byte).
 *
 * Return: bytes written, or -EINVAL/-ENOSPC on error.
 */
static inline int btmtk_a32_encode_wmt_cmd(const u8 *payload, u8 payload_len,
					  u8 *out, unsigned int outlen)
{
	unsigned int need, i;

	need = (unsigned int)BTMTK_A32_WMT_HDR_LEN + payload_len;
	if (!out || outlen < need)
		return -ENOSPC;
	if (payload_len && !payload)
		return -EINVAL;
	out[0] = (u8)BTMTK_A32_H4_CMD;
	out[1] = (u8)BTMTK_A32_WMT_OP_LSB;
	out[2] = (u8)BTMTK_A32_WMT_OP_MSB;
	out[3] = payload_len;
	for (i = 0; i < payload_len; i++)
		out[4 + i] = payload[i];
	return (int)need;
}

/**
 * btmtk_a32_fw_name - firmware basename for download slot @index.
 * @index: 0..BTMTK_A32_FW_COUNT-1 in downstream request order.
 * @buf: destination; @buflen its size (min 32 recommended).
 *
 * The driver passes the returned SHORT basename straight to
 * request_firmware() (downstream pattern btmtk_main.c:605) -- no
 * mediatek/ prefix is ever added here.
 *
 * Return: 0 on success, -EINVAL on bad index/buffer.
 */
static inline int btmtk_a32_fw_name(unsigned int index, char *buf,
				   unsigned int buflen)
{
	const char *names[BTMTK_A32_FW_COUNT] = {
		BTMTK_A32_FW_BT_RAM,
		BTMTK_A32_FW_PATCH_MCU,
		BTMTK_A32_FW_RAM_MCU,
	};
	unsigned int i;

	if (index >= (unsigned int)BTMTK_A32_FW_COUNT || !buf || buflen == 0)
		return -EINVAL;
	for (i = 0; names[index][i] != '\0'; i++) {
		if (i + 1u >= buflen)
			return -ENOSPC;
		buf[i] = names[index][i];
	}
	buf[i] = '\0';
	return 0;
}

#endif /* __BTMTK_A32_H */
