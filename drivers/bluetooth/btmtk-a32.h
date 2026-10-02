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
 * §3 REAL firmware-download contract (EMI, not HCI-chunked).
 *
 * Downstream 66xx/CONNAC2 path (the A32/MT6631-class path -- NOT the
 * 766x/79xx USB/SDIO path) downloads firmware by copying patch bodies
 * into EMI, never by HCI-chunked WMT upload:
 *
 * - Entry: bt_hw_and_mcu_on() -> btmtk_load_rom_patch() ->
 *   is_mt66xx() -> btmtk_load_rom_patch_66xx() -> bgfsys_bt_patch_dl()
 *   (connac2/btmtk_mt66xx.c:1115-1125, kernel/downstream/).
 * - Order is FIXED: bgfsys_mcu_rom_patch_dl() FIRST (g_fwp_names[0],
 *   the MCU ROM patch), then bgfsys_bt_ram_code_dl() (g_fwp_names[1],
 *   the BT RAM code). Each leg is __download_patch_to_emi()
 *   (btmtk_mt66xx.c:929-...): btmtk_load_code_from_bin() with retry 10
 *   (btmtk_main.c:599-615: request_firmware + msleep(100)), reject when
 *   smaller than the 48-byte EMI header, parse struct fw_patch_emi_hdr,
 *   strip the header, copy the body to EMI (conninfra-supplied base +
 *   header emi_addr offset via ioremap + memcpy_toio).
 * - A32 name composition follows compose_fw_name()
 *   (platform_base.h:220-249): "<BIN>[_<flavor>]_1_hdr.bin" with the
 *   flavor char from the "mediatek,bt" flavor_bin DT property; A32's
 *   manifest pins flavor 'a': soc1_0_patch_mcu_1a_1_hdr.bin (MCU slot,
 *   sent FIRST) and soc1_0_ram_bt_1a_1_hdr.bin (BT slot, sent SECOND).
 *   The manifest's third blob, soc1_0_ram_mcu_1a_1_hdr.bin, has NO
 *   downstream 66xx send-order evidence (PATCH_FILE_NUM is 2); it is
 *   staged THIRD in manifest order with its position marked
 *   HARDWARE-UNPROVEN.
 * - There is NO HCI chunk/phase/ACK loop on this path: PATCH_PHASE1/2/3,
 *   UPLOAD_PATCH_UNIT (2048), PATCH_INFO_SIZE (30) and the 04 E4 05
 *   LD_PATCH_EVT belong to the 766x/79xx btmtk_load_fw_patch_using_wmt_cmd
 *   path (btmtk_main.c:1368-1440) and MUST NOT be replayed here.
 * - BT_FW.cfg is NOT a patch blob: btmtk_intcmd_wmt_send_antenna_cmd()
 *   (btmtk_mt66xx.c:1043-...) loads it as TEXT, finds the
 *   "[driver_antenna][<chipid>]" tag and sends ONE WMT command
 *   (01 6F FC ... 01 55 antenna-efem); the remaining VENDOR_CMD lines are
 *   post-HCI autobt replay (TODO-AUTOBT). It is never checksummed or
 *   EMI-copied; the driver only inventories the name.
 *
 * Pure (testable) part of that contract lives here; the request_firmware
 * + EMI-copy half lives in btmtk-a32.c and fails probe LOUDLY when any
 * blob is absent or fails verification.
 * ------------------------------------------------------------------
 */

/* EMI patch header, byte-identical to downstream struct fw_patch_emi_hdr
 * (connac2/btmtk_mt66xx.c:46-55, kernel/downstream/): 16+4+2+2+4+4+14+2
 * = 48 bytes. Field ORDER is the contract (emi_addr middle bytes select
 * the EMI offset; crc covers the body AFTER this header).
 */
#define BTMTK_A32_EMI_HDR_LEN	48
#define BTMTK_A32_EMI_DT_LEN	16

struct btmtk_a32_emi_hdr {
	u8 date_time[BTMTK_A32_EMI_DT_LEN];
	u8 plat[4];
	u16 hw_ver;
	u16 sw_ver;
	u8 emi_addr[4];
	u32 subsys_id;
	u8 reserved[14];
	u16 crc;
};

/* 48-byte EMI header layout guard (field order = downstream contract). */
_Static_assert(sizeof(struct btmtk_a32_emi_hdr) == BTMTK_A32_EMI_HDR_LEN,
	       "EMI header must be exactly 48 bytes");

/**
 * btmtk_a32_fw_send_order - download sequence position -> firmware slot.
 * @pos: 0..BTMTK_A32_FW_COUNT-1 in on-the-wire send order.
 *
 * Downstream bgfsys_bt_patch_dl() sends the MCU ROM patch FIRST and the
 * BT RAM code SECOND; the A32 manifest's third blob (RAM MCU) has no
 * downstream order evidence and goes last (HARDWARE-UNPROVEN position).
 * Slots are the btmtk_a32_fw_name() indices (0 = BT RAM, 1 = patch MCU,
 * 2 = RAM MCU), kept stable so the name contract never shifts.
 *
 * Return: slot index, or -EINVAL on out-of-range @pos.
 */
static inline int btmtk_a32_fw_send_order(unsigned int pos)
{
	static const u8 order[BTMTK_A32_FW_COUNT] = { 1, 0, 2 };

	if (pos >= (unsigned int)BTMTK_A32_FW_COUNT)
		return -EINVAL;
	return order[pos];
}

/**
 * btmtk_a32_fw_crc - EMI patch body checksum (fwp_checksume16 algorithm).
 * @data: body bytes (AFTER the 48-byte EMI header).
 * @len: body length.
 *
 * Exact downstream fwp_checksume16 (connac2/btmtk_mt66xx.c:114-137):
 * 16-bit ones'-complement sum over the body, trailing odd byte added
 * raw, result bitwise-inverted. Compared against hdr->crc by
 * fwp_check_patch (:166) / __download_patch_to_emi.
 *
 * Return: checksum to compare against the header crc field.
 */
static inline u16 btmtk_a32_fw_crc(const u8 *data, unsigned int len)
{
	u32 sum = 0;

	if (!data)
		return 0;
	while (len > 1) {
		sum += (u32)data[0] | ((u32)data[1] << 8);
		data += 2;
		if (sum & 0x80000000u)
			sum = (sum & 0xffffu) + (sum >> 16);
		len -= 2;
	}
	if (len)
		sum += *data;
	while (sum >> 16)
		sum = (sum & 0xffffu) + (sum >> 16);
	return (u16)~sum;
}

/**
 * btmtk_a32_emi_offset - EMI offset selected by a patch header.
 * @hdr: parsed 48-byte EMI header.
 *
 * Downstream __download_patch_to_emi: the header emi_addr is the FW view
 * 0xFXXXXXXX; the middle two bytes are the offset added to the
 * conninfra-supplied EMI AP physical base
 * (emi_addr[2] << 16 | emi_addr[1] << 8).
 *
 * Return: offset, or -EINVAL on NULL @hdr.
 */
static inline int btmtk_a32_emi_offset(const struct btmtk_a32_emi_hdr *hdr)
{
	if (!hdr)
		return -EINVAL;
	return ((int)hdr->emi_addr[2] << 16) | ((int)hdr->emi_addr[1] << 8);
}

/* ------------------------------------------------------------------
 * §1 BTIF PIO block contract (pure part).
 *
 * The AP-side BTIF block is driven in PIO mode (no DMA engine): TX
 * pushes bytes to THR, RX pulls bytes from RBR, both under the LSR/IER/
 * IIR handshake. Register map + bits from the downstream BTIF HAL
 * (btif/common/plat_inc/btif_priv.h, kernel/downstream/ -- 16550-shaped):
 * RBR/THR +0x0, IER +0x4 (RXFEN bit0, TXEEN bit1), IIR +0x8 (NINT bit0,
 * TX_EMPTY bit1, RX bit2, RX_TIMEOUT 0x44), FIFOCTRL +0x8 (CLR_RX bit1,
 * CLR_TX bit2), FAKELCR +0xC (normal 0), LSR +0x14 (DR bit0, THRE bit5,
 * TEMT bit6), SLEEP_EN +0x48, DMA_EN +0x4C (RX bit0, TX bit1, AUTORST
 * bit2), TRI_LVL +0x60 (TX[3:0], RX[6:4], LOOP bit7), WAK +0x64 (WAK
 * bit0), HANDSHAKE +0x6C.
 *
 * Sequences (downstream btif/common/, kernel/downstream/):
 * - hw_init (hal_btif_hw_init): FAKELCR normal, handshake on, FIFOCLR
 *   RX then TX, TRI_LVL = TX(8)|RX(1)|LOOP_DIS, loopback off, DMA TX+RX
 *   off (= PIO), AUTORST on, TXEEN off, RXFEN on.
 * - TX PIO (_btif_pio_write + hal_btif_send_data, legacy non-kfifo leg):
 *   LSR TEMT -> 16 bytes room, THRE -> 16-8 = 8 bytes room, else 0;
 *   writeb() to THR; retry budget 10.
 * - RX IRQ (hal_btif_irq_handler + btif_rx_irq_handler): read IIR; while
 *   RX|RX_TIMEOUT: readb() RBR into the rx_cb; TX_EMPTY tops up THR.
 *   The IIR/RBR reads ARE the ack (16550 semantics) -- no masking.
 * - Wake pulse (hal_btif_raise_wak_sig): CLR WAK bit, usleep 128-160
 *   (> 1/32k period + margin), SET WAK bit -- AP->CONSYS wakeup.
 * - FIFOs: TX 16 / RX 8 (BTIF_TX_FIFO_SIZE / BTIF_RX_FIFO_SIZE);
 *   thresholds TX 8 / RX 1.
 * ------------------------------------------------------------------
 */

/* BTIF PIO register offsets (relative to the BTIF window base). */
#define BTMTK_A32_BTIF_RBR_OFF		0x00
#define BTMTK_A32_BTIF_THR_OFF		0x00
#define BTMTK_A32_BTIF_IER_OFF		0x04
#define BTMTK_A32_BTIF_IIR_OFF		0x08
#define BTMTK_A32_BTIF_FIFOCTRL_OFF	0x08
#define BTMTK_A32_BTIF_FAKELCR_OFF	0x0C
#define BTMTK_A32_BTIF_LSR_OFF		0x14
#define BTMTK_A32_BTIF_SLEEP_EN_OFF	0x48
#define BTMTK_A32_BTIF_DMA_EN_OFF	0x4C
#define BTMTK_A32_BTIF_TRI_LVL_OFF	0x60
#define BTMTK_A32_BTIF_WAK_OFF		0x64
#define BTMTK_A32_BTIF_HANDSHAKE_OFF	0x6C

/* IER / IIR / LSR bits. */
#define BTMTK_A32_BTIF_IER_RXFEN	(1u << 0)
#define BTMTK_A32_BTIF_IER_TXEEN	(1u << 1)
#define BTMTK_A32_BTIF_IIR_NINT		(1u << 0)
#define BTMTK_A32_BTIF_IIR_TX_EMPTY	(1u << 1)
#define BTMTK_A32_BTIF_IIR_RX		(1u << 2)
#define BTMTK_A32_BTIF_IIR_RX_TIMEOUT	(0x11u << 2)
#define BTMTK_A32_BTIF_LSR_DR		(1u << 0)
#define BTMTK_A32_BTIF_LSR_THRE		(1u << 5)
#define BTMTK_A32_BTIF_LSR_TEMT		(1u << 6)

/* FIFOCTRL / DMA_EN / TRI_LVL / WAK bits and thresholds. */
#define BTMTK_A32_BTIF_FIFOCTRL_CLR_RX	(1u << 1)
#define BTMTK_A32_BTIF_FIFOCTRL_CLR_TX	(1u << 2)
#define BTMTK_A32_BTIF_DMA_EN_RX	(1u << 0)
#define BTMTK_A32_BTIF_DMA_EN_TX	(1u << 1)
#define BTMTK_A32_BTIF_DMA_EN_AUTORST	(1u << 2)
#define BTMTK_A32_BTIF_TRI_TX_LVL	8u
#define BTMTK_A32_BTIF_TRI_RX_LVL	1u
#define BTMTK_A32_BTIF_TRI_LOOP_DIS	(0u << 7)
#define BTMTK_A32_BTIF_WAK_BIT		(1u << 0)
#define BTMTK_A32_BTIF_HANDSHAKE_ON	1u
#define BTMTK_A32_BTIF_TX_FIFO_SIZE	16u
#define BTMTK_A32_BTIF_RX_FIFO_SIZE	8u

/* PIO TX retry budget (_btif_pio_write max_tx_retry). */
#define BTMTK_A32_BTIF_PIO_TX_RETRY	10

/* BTIF-block IRQ (downstream mt6768.dts btif@1100c000: "btif irq",
 * GIC_SPI 133 LEVEL_LOW). TX/RX DMA IRQs (115/141) are NOT wired:
 * DMA stays TODO (TODO-DMA) and unrequested lines must not be claimed.
 */
#define BTMTK_A32_BTIF_IRQ_NAME		"btif"

/**
 * btmtk_a32_btif_tx_room - PIO TX room from an LSR value.
 * @lsr: BTIF_LSR register value.
 *
 * hal_btif_send_data room rule: TEMT -> full FIFO (16), else THRE ->
 * FIFO minus threshold (16-8 = 8), else 0 (FIFO above threshold).
 *
 * Return: bytes that may be pushed to THR now.
 */
static inline unsigned int btmtk_a32_btif_tx_room(u32 lsr)
{
	if (lsr & (u32)BTMTK_A32_BTIF_LSR_TEMT)
		return BTMTK_A32_BTIF_TX_FIFO_SIZE;
	if (lsr & (u32)BTMTK_A32_BTIF_LSR_THRE)
		return BTMTK_A32_BTIF_TX_FIFO_SIZE - BTMTK_A32_BTIF_TRI_TX_LVL;
	return 0;
}

/**
 * btmtk_a32_btif_tri_lvl - TRI_LVL register value for PIO bring-up.
 *
 * hal_btif_hw_init: TX threshold | RX threshold | LOOP_DIS.
 *
 * Return: the register value.
 */
static inline u32 btmtk_a32_btif_tri_lvl(void)
{
	return ((u32)BTMTK_A32_BTIF_TRI_TX_LVL & 0xfu) |
	       (((u32)BTMTK_A32_BTIF_TRI_RX_LVL & 0x7u) << 4) |
	       (u32)BTMTK_A32_BTIF_TRI_LOOP_DIS;
}

/**
 * btmtk_a32_fw_fit - bounds-check one staged blob against the EMI pool.
 * @pool_size: EMI pool size in bytes.
 * @off: blob EMI offset (from btmtk_a32_emi_offset).
 * @len: blob body length.
 *
 * The __download_patch_to_emi() range gate (elided downstream, enforced
 * here): the copy must land strictly inside the pool.
 *
 * Return: 0 fits, -ERANGE otherwise.
 */
static inline int btmtk_a32_fw_fit(unsigned int pool_size, u32 off,
				   unsigned int len)
{
	if (len == 0)
		return -ERANGE;
	if (off >= pool_size || len > pool_size - off)
		return -ERANGE;
	return 0;
}

/* ------------------------------------------------------------------
 * §3 BTIF transport + CONNSYS register contract (pure part).
 *
 * BTIF open/rx/TX/close shape (connac2/btmtk_btif_main.c,
 * kernel/downstream/):
 * - open: mtk_wcn_btif_open("CONSYS_BT", &id) (:735, owner string at :46)
 *   THEN mtk_wcn_btif_rx_cb_register(id, bt_receive_data_cb) (:752) --
 *   registration order is load-bearing (no RX source before open);
 *   dpidle-idle workqueue arming is power policy, not transport.
 * - RX: bt_receive_data_cb (:669-678) clears the PSM sleep_flag and calls
 *   btmtk_recv() (h4_recv_buf demux) -- this driver's btmtk_a32_recv()
 *   is that callback.
 * - TX: btmtk_btif_send_cmd (:1053-1095) partial-write loop over
 *   mtk_wcn_btif_write() with retry budget + usleep_range() backoff
 *   between attempts; returns -1 when the BTIF id is NULL (never
 *   silently drops).
 * - close: mtk_wcn_btif_close(id) + id = 0 (:724-...).
 * There is NO STP task/channel multiplexing on the BTIF path: the
 * ENABLESTP/mtk_stp_split branch in btmtk_recv() is compiled out on
 * CHIP_IF_BTIF (connac2/Makefile:28 -DCHIP_IF_BTIF); WMT-vs-BT channel
 * selection is the 01 6F FC opcode match inside the single BTIF stream.
 *
 * CONNSYS register contract (connac2/btmtk_mt66xx_reg.h,
 * kernel/downstream/ -- connac2-COMMON offsets/bits; the absolute bases
 * BGF_REG_BASE_ADDR (BGFSYS 0x18800000 window) and CONN_HOST_CSR_TOP are
 * conninfra-mapped at runtime and are NEVER hardcoded here):
 * - BGF_SW_IRQ_STATUS = BGF_BASE + 0x0150, RESET_ADDR = BGF_BASE + 0x014C;
 *   BGF_WHOLE_CHIP_RESET = BIT(26), BGF_SUBSYS_CHIP_RESET = BIT(25),
 *   BGF_FW_LOG_NOTIFY = BIT(24) (:253-257).
 * - FW-own handshake: BGF_LPCTL = CSR_BASE + 0x0030, HOST_SET_FW_OWN =
 *   BIT(0), HOST_CLR_FW_OWN = BIT(1), OWNER_STATE_SYNC = BIT(2);
 *   BGF_IRQ_STAT = CSR_BASE + 0x0034, FW_OWN_CLR = BIT(0);
 *   BGF_IRQ_STAT2 = CSR_BASE + 0x003C, FW_OWN_SET = BIT(0) (:226-239).
 * - Polling budget LPCR_POLLING_RTY_LMT = 4096 with ~0.5ms waits
 *   (btmtk_btif_main.c:48, :290-330): wakeup polls OWNER_STATE_SYNC
 *   clear after HOST_CLR then write-1-clears IRQ_STAT; sleep polls
 *   IRQ_STAT2 FW_OWN_SET after HOST_SET (HW asserts OWNER_STATE_SYNC
 *   without FW ack, so it must NOT be used as the sleep-done test).
 * ------------------------------------------------------------------
 */

/* BGF software-IRQ status/clear offsets (relative to the conninfra-
 * provided BGF register base) and dispatch bits. Values from
 * connac2/btmtk_mt66xx_reg.h:253-257 (kernel/downstream/).
 */
#define BTMTK_A32_BGF_SW_IRQ_STATUS_OFF	0x0150
#define BTMTK_A32_BGF_SW_IRQ_RESET_OFF	0x014C
#define BTMTK_A32_BGF_WHOLE_CHIP_RESET	(1u << 26)
#define BTMTK_A32_BGF_SUBSYS_CHIP_RESET	(1u << 25)
#define BTMTK_A32_BGF_FW_LOG_NOTIFY	(1u << 24)

/* FW-own/host-own handshake offsets (relative to the conninfra-provided
 * CONN_HOST_CSR base) and bits (connac2/btmtk_mt66xx_reg.h:226-239).
 */
#define BTMTK_A32_BGF_LPCTL_OFF		0x0030
#define BTMTK_A32_BGF_HOST_SET_FW_OWN	(1u << 0)
#define BTMTK_A32_BGF_HOST_CLR_FW_OWN	(1u << 1)
#define BTMTK_A32_BGF_OWNER_STATE_SYNC	(1u << 2)
#define BTMTK_A32_BGF_IRQ_STAT_OFF	0x0034
#define BTMTK_A32_BGF_IRQ_FW_OWN_CLR	(1u << 0)
#define BTMTK_A32_BGF_IRQ_STAT2_OFF	0x003C
#define BTMTK_A32_BGF_IRQ_FW_OWN_SET	(1u << 0)

/* FW-own polling budget: 4096 x ~0.5ms (btmtk_btif_main.c:48). */
#define BTMTK_A32_FW_OWN_RETRY		4096

/* BTIF owner name claimed at open (btmtk_btif_main.c:46). */
#define BTMTK_A32_BTIF_OWNER		"CONSYS_BT"

/* BTIF TX partial-write retry budget (btmtk_btif_send_cmd shape). */
#define BTMTK_A32_BTIF_TX_RETRY		5

/* SW-IRQ dispatch outcome for the threaded handler (mirrors the
 * bt_bgf2ap_irq_handler() branches, connac2/btmtk_irq.c:128-...).
 */
enum btmtk_a32_sw_evt {
	BTMTK_A32_SW_NONE = 0,		/* status clear / nothing to do */
	BTMTK_A32_SW_SUBSYS_RESET,	/* SUBSYS_CHIP_RESET: schedule reset */
	BTMTK_A32_SW_FW_LOG,		/* FW_LOG_NOTIFY: drain FW log */
	BTMTK_A32_SW_WHOLE_RESET,	/* WHOLE_CHIP_RESET: whole-chip reset */
	BTMTK_A32_SW_BUS_HANG,		/* 0xDEADFEED / unreadable: dump+reset */
};

/**
 * btmtk_a32_sw_dispatch - classify a BGF_SW_IRQ_STATUS value.
 * @status: raw BGF_SW_IRQ_STATUS register value.
 *
 * Priority mirrors bt_bgf2ap_irq_handler(): SUBSYS first (ack + reset
 * path), then FW_LOG, then WHOLE_CHIP, else none. 0xDEADFEED (bus
 * timeout sentinel) maps to BUS_HANG.
 *
 * Return: the dispatch outcome.
 */
static inline enum btmtk_a32_sw_evt
btmtk_a32_sw_dispatch(u32 status)
{
	if (status == 0xDEADFEEDu)
		return BTMTK_A32_SW_BUS_HANG;
	if (status & (u32)BTMTK_A32_BGF_SUBSYS_CHIP_RESET)
		return BTMTK_A32_SW_SUBSYS_RESET;
	if (status & (u32)BTMTK_A32_BGF_FW_LOG_NOTIFY)
		return BTMTK_A32_SW_FW_LOG;
	if (status & (u32)BTMTK_A32_BGF_WHOLE_CHIP_RESET)
		return BTMTK_A32_SW_WHOLE_RESET;
	return BTMTK_A32_SW_NONE;
}

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
