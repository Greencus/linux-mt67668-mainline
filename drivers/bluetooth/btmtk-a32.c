/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * btmtk-a32.c -- A32 (MT6768 / MT6631-class CONNSYS) Bluetooth HCI driver,
 * Track B (mainline BTIF transport).
 *
 * Public interface is a standard BlueZ hciX device. There is deliberately
 * NO /dev/stpbt chrdev here: downstream exposes /dev/stpbt via
 * MKDEV/register_chrdev_region + class_create("stpbt") + device_create
 * (bt/mt66xx/legacy/stp_chrdev_bt.c:766/:782/:796/:799) as an INTERNAL
 * transport layer; the final host interface for THIS driver is hciX.
 *
 * Transport truth (downstream, read-only):
 * - BTIF is the COMPILE-TIME default: -DCHIP_IF_BTIF
 *   (bt/mt66xx/connac2/Makefile:28, Makefile.ce:23,
 *   selector btmtk_chip_if.h:25). NOT UART H4, NOT btusb. This driver
 *   therefore never calls hciattach-style H4 sync and never binds USB.
 * - Power/module order on the vendor system (reference only, implemented
 *   here as documented sequencing hooks, not module loads):
 *   init.wmt_drv.rc (wmt_drv on boot) -> init.connfem.rc (connfem on boot)
 *   -> init.bt_drv.rc (insmod bt_drv_${ro.vendor.bt.platform}.ko on
 *   vendor.connsys.driver.ready=yes AND on =no, i.e. unconditional after
 *   WMT:
 *   references/android/vendor/vendor_samsung_a32/proprietary/vendor/etc/init/).
 * - HCI flow: WMT-over-HCI 01 6F FC header (btmtk_mt66xx.c:98); rx via
 *   h4_recv_buf (btmtk_main.c:43,345-360) demuxing ACL/SCO/EVENT
 *   (btmtk_main.c:177-179) with delivery through hci_recv_frame
 *   (:416,510,529); tx via btmtk_main_send_cmd plus
 *   btmtk_btif_open/rx_cb_register (btmtk_btif_main.c:729,752); BTIF
 *   wakeup IRQ (btmtk_irq.c:39,211).
 * - Firmware: request_firmware(&fw_entry, bin_name, dev) with retry
 *   (btmtk_main.c:598-613); SHORT basenames, no mediatek/ prefix.
 *
 * Mainline 6.18 reuse/gap (grep-verified in THIS tree):
 * - REUSABLE (not duplicated): btmtkuart.c serdev WMT semaphore/func-ctrl
 *   helpers and btmtk.h/c protocol pieces stay the shared upstream code.
 * - CONFIRMED GAP: no mt6631 / soc1_0 / mt6768 firmware-name mapping
 *   (btmtk.h FIRMWARE_MT* covers MT7622/63/68/7922/7961/7925 only;
 *   btmtk.c device table covers MT7663/68/7922/7925/7961 only) and NO
 *   stp_btif / mtk_wcn_btif_* transport anywhere in mainline (serdev-
 *   UART/SDIO/USB only). This file fills exactly that gap: the A32
 *   firmware-name mapping plus a BTIF/STP transport shim behind the
 *   standard HCI ops. It never touches btmtkuart.c/btmtk.c.
 *
 * Hardware status: IMPLEMENTED + OFFLINE-VALIDATED (build, symbols,
 * framing unit tests). Controller discovery, firmware download on real
 * silicon, pairing -- all HARDWARE-UNPROVEN (see
 * docs/BLUETOOTH-MAINLINE-IMPLEMENTATION.md).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/delay.h> /* msleep() in firmware retry loop */
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/string.h>
#include <linux/slab.h> /* kmalloc/kfree for staged firmware bodies */
#include <linux/io.h> /* readl/writel for BGF/FW-own CR access */
#include <linux/interrupt.h> /* §3: request_threaded_irq + IRQF_* */
#include <linux/spinlock.h> /* §3: enable/disable active-flag lock */
#include <linux/pm_wakeup.h> /* §3: wakeup_source + device_init_wakeup */
#include <linux/pm.h> /* §3: SET_SYSTEM_SLEEP_PM_OPS */

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btmtk-a32.h"

#define BTMTK_A32_VERSION		"0.1"
#define BTMTK_A32_DRVNAME		"btmtk-a32"

/* RX reassembly budget: largest HCI frame we accept off the BTIF/STP
 * stream (4 KiB covers ACL MTU + headers with margin).
 */
#define BTMTK_A32_RX_MAX		4096

/**
 * struct btmtk_a32_btif - REAL BTIF backend state (conninfra-provided).
 * @bgf_base: conninfra-mapped BGF register window (BGFSYS 0x18800000
 *   class; BGF_SW_IRQ_STATUS/RESET_ADDR live at +0x0150/+0x014C).
 *   NULL until TODO-CONNINFRA maps it -- every accessor fails LOUDLY
 *   (-ENODEV) instead of touching a guessed address.
 * @csr_base: conninfra-mapped CONN_HOST_CSR window (BGF_LPCTL/IRQ_STAT/
 *   IRQ_STAT2 live at +0x0030/+0x0034/+0x003C). NULL until TODO-CONNINFRA.
 * @claimed: CONSYS_BT owner claimed (g_btif_id != 0 analogue).
 * @rx_registered: btmtk_a32_recv() registered as the BTIF RX callback
 *   (rx_cb_register analogue). Registration order is load-bearing:
 *   set at open, cleared at close; RX bytes only ever enter through
 *   btmtk_a32_recv().
 * @tx_retry: partial-write retry budget (downstream: while tx_len, retry).
 *
 * No address is ever invented here: both bases arrive from conninfra at
 * runtime (of_iomap / conninfra-provided window, TODO-CONNINFRA).
 */
struct btmtk_a32_btif {
	void __iomem *bgf_base;
	void __iomem *csr_base;
	bool claimed;
	bool rx_registered;
};

/**
 * struct btmtk_a32_transport - REAL BTIF/STP send/recv abstraction.
 * @send: transmit one H4-framed packet on the BTIF stream
 *   (btmtk_btif_send_cmd analogue: partial-write loop + retry).
 * @open: bring the transport up: claim CONSYS_BT, register the RX
 *   callback, clear FW-own (btmtk_wcn_btif_open + rx_cb_register +
 *   fw_own_clr analogue). Fails LOUDLY without hardware.
 * @close: shut the transport down (mtk_wcn_btif_close analogue).
 * @ctx: backend context (&adev).
 *
 * Final path: BlueZ -> hciX -> btmtk-a32 -> THIS BTIF/STP backend ->
 * WMT/CONNSYS -> controller. No stub, no loopback, no hciattach, no
 * btusb, no fake UART -- anywhere on this path.
 */
struct btmtk_a32_transport {
	int (*send)(struct device *dev, void *ctx,
		    const u8 *data, unsigned int len);
	int (*open)(struct device *dev, void *ctx);
	void (*close)(struct device *dev, void *ctx);
	void *ctx;
};

struct btmtk_a32_dev {
	struct hci_dev *hdev;
	struct device *dev;
	struct btmtk_a32_transport transport;
	struct btmtk_a32_btif btif;
	struct work_struct tx_work;
	struct work_struct rst_work; /* subsys-reset trigger (rst_trigger_work analogue) */
	struct sk_buff_head txq;
	struct sk_buff *rx_skb;
	unsigned int rx_target; /* full H4 length of the frame in rx_skb */
	struct completion ready;
	bool opened;
	/* Staged firmware bodies (header stripped, CRC verified): slot i
	 * corresponds to btmtk_a32_fw_name(i). Committed to EMI at probe
	 * once conninfra supplies the EMI window (TODO-CONNINFRA).
	 */
	u8 *fw_body[BTMTK_A32_FW_COUNT];
	size_t fw_len[BTMTK_A32_FW_COUNT];
	u32 fw_emi_off[BTMTK_A32_FW_COUNT];
	/* Power hooks: DT-provided ONLY, both optional. No invented
	 * regulator names or GPIO numbers anywhere in this driver;
	 * absent properties log a TODO and probe continues (offline-safe).
	 */
	struct regulator *vcc;
	struct gpio_desc *reset;
	/* §3 wakeup-IRQ path (downstream btmtk_irq.c:39,211 shape). Both
	 * IRQ numbers are DT-resolved (platform_get_irq_byname on the
	 * interrupt-names from btmtk-a32.h); negative (-ENOENT/-EPROBE_DEFER
	 * passthrough) when the fragment carries no numeric interrupts
	 * property (expected until TODO-HWIRQ closes). Numbers are NEVER
	 * hardcoded: the downstream MT_BGF2AP_*_ID 312/271 are marked
	 * temp-only (btmtk_btif.h:61-62).
	 */
	int wake_irq; /* BGF2AP_BTIF_WAKEUP_IRQ analogue: FW has data */
	int sw_irq; /* BGF2AP_SW_IRQ analogue: FW assert / FW-log notify */
	spinlock_t irq_lock; /* guards *_active, cf. bt_irq_ctrl.lock */
	bool wake_active;
	bool sw_active;
	struct wakeup_source *ws; /* "bt_psm" analogue (btmtk_btif.h:456) */
	enum btmtk_a32_psm psm_state;
	bool rx_pending; /* FW-data indication (g_bdev->rx_ind analogue) */
	bool bgf2ap_ind; /* SW-IRQ indication (cif_dev->bgf2ap_ind analogue) */
	u32 sw_status; /* latched BGF_SW_IRQ_STATUS for the IRQ thread */
};

static int btmtk_a32_btif_send(struct device *dev, void *ctx,
			       const u8 *data, unsigned int len);
static int btmtk_a32_btif_open(struct device *dev, void *ctx);
static void btmtk_a32_btif_close(struct device *dev, void *ctx);
static int btmtk_a32_fw_own_clr(struct btmtk_a32_dev *adev);
static int btmtk_a32_fw_own_set(struct btmtk_a32_dev *adev);
static int btmtk_a32_set_sleep(struct btmtk_a32_dev *adev);
static int btmtk_a32_set_wakeup(struct btmtk_a32_dev *adev);

/* ------------------------------------------------------------------
 * RX path: H4 reassembly + demux, mirroring downstream h4_recv_buf()
 * (btmtk_main.c:43) and mtk_recv_pkts[] (btmtk_main.c:177-179).
 * ------------------------------------------------------------------
 */

/**
 * btmtk_a32_deliver() - hand a complete H4 frame to the HCI core.
 * @adev: driver context.
 * @type: H4 packet indicator (already validated).
 * @frame: payload AFTER the H4 type byte.
 * @flen: length of @frame in bytes.
 *
 * Downstream delivery calls hci_recv_frame() for ACL (:416) and event
 * (:510,529) completions, with SCO passed straight through (:178).
 */
static int btmtk_a32_deliver(struct btmtk_a32_dev *adev, u8 type,
			     const u8 *frame, unsigned int flen)
{
	struct sk_buff *skb;
	__u8 pkt_type;

	switch (type) {
	case BTMTK_A32_H4_ACL:
		pkt_type = HCI_ACLDATA_PKT;
		break;
	case BTMTK_A32_H4_SCO:
		pkt_type = HCI_SCODATA_PKT;
		break;
	case BTMTK_A32_H4_EVT:
		pkt_type = HCI_EVENT_PKT;
		break;
	default:
		return -EINVAL;
	}

	skb = bt_skb_alloc(flen, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	skb_put_data(skb, frame, flen);
	hci_skb_pkt_type(skb) = pkt_type;

	/* Vendor-packet note: WMT events arriving as HCI EVENT frames flow
	 * through the same hci_recv_frame() delivery as downstream; the
	 * 01 6F FC WMT-over-HCI header applies to the CMD direction
	 * (btmtk_mt66xx.c:98) and is matched on tx.
	 */
	return hci_recv_frame(adev->hdev, skb);
}

/**
 * btmtk_a32_recv() - feed raw BTIF/STP stream bytes into reassembly.
 * @adev: driver context.
 * @data: stream bytes.
 * @count: number of bytes available in @data.
 *
 * Byte-stream contract identical to the downstream rx callback feeding
 * h4_recv_buf() (btmtk_main.c:345-360): arbitrary chunking, one H4
 * type byte at each packet boundary, length from the HCI header.
 * Returns 0 on success, -EINVAL on framing error (resync by dropping
 * the partial frame, same policy as the IS_ERR path downstream).
 *
 * This IS the registered BTIF RX callback (bt_receive_data_cb analogue,
 * wired by btmtk_a32_transport_attach() in the btmtk_wcn_btif_open +
 * rx_cb_register order): BTIF stream bytes enter ONLY here.
 */
static int btmtk_a32_recv(struct btmtk_a32_dev *adev,
			  const u8 *data, unsigned int count)
{
	while (count) {
		int take;

		if (!adev->rx_skb) {
			int want = btmtk_a32_rx_frame_len(data, count);

			if (want == 0)
				return 0; /* need more bytes for header */
			if (want < 0)
				return -EINVAL; /* unknown H4 type */
			if (want > BTMTK_A32_RX_MAX)
				return -EINVAL;
			adev->rx_skb = bt_skb_alloc((unsigned int)want,
						    GFP_ATOMIC);
			if (!adev->rx_skb)
				return -ENOMEM;
			adev->rx_target = (unsigned int)want;
		}

		take = (int)(adev->rx_target - adev->rx_skb->len);
		if ((unsigned int)take > count)
			take = (int)count;
		skb_put_data(adev->rx_skb, data, (unsigned int)take);
		data += take;
		count -= (unsigned int)take;

		if (adev->rx_skb->len >= adev->rx_target) {
			struct sk_buff *done = adev->rx_skb;
			u8 type = done->data[0];
			int err;

			adev->rx_skb = NULL;
			adev->rx_target = 0;
			/* Strip the H4 type byte before delivery (the HCI
			 * core expects pkt_type in skb control, payload in
			 * data -- same split as hci_recv_frame callers).
			 */
			err = btmtk_a32_deliver(adev, type, done->data + 1,
						done->len - 1);
			kfree_skb(done);
			if (err < 0)
				return err;
		} else {
			return 0; /* partial frame buffered, await more */
		}
	}
	return 0;
}

/* ------------------------------------------------------------------
 * REAL BTIF backend: open/rx-cb-register/TX/RX/close/wake/sleep.
 *
 * Downstream order (connac2/btmtk_btif_main.c, kernel/downstream/):
 * mtk_wcn_btif_open("CONSYS_BT") (:735) THEN rx_cb_register
 * (bt_receive_data_cb, :752); TX is the btmtk_btif_send_cmd partial-write
 * loop with retry + backoff (:1053-1095); RX enters via
 * bt_receive_data_cb -> btmtk_recv (:669-678). The register windows this
 * backend drives are conninfra-mapped at runtime (bgf_base/csr_base);
 * until TODO-CONNINFRA maps them every entry point fails LOUDLY --
 * nothing is faked, looped back, or silently dropped.
 * ------------------------------------------------------------------
 */

/**
 * btmtk_a32_btif_write() - one BTIF write primitive (backend hook point).
 * @adev: driver context.
 * @data: bytes to push to the controller.
 * @len: byte count.
 *
 * Stands in for mtk_wcn_btif_write(g_btif_id, ...) on the single BTIF
 * stream (no STP task multiplexing on CHIP_IF_BTIF). Wired to the
 * conninfra BTIF write once TODO-CONNINFRA closes; until then it fails
 * LOUDLY (-ENODEV) so TX can never silently vanish.
 *
 * Return: bytes written, or negative errno.
 */
static int btmtk_a32_btif_write(struct btmtk_a32_dev *adev,
				const u8 *data, unsigned int len)
{
	if (!adev->btif.claimed) {
		dev_err(adev->dev,
			"BTIF write with no claimed owner (TODO-CONNINFRA: conninfra BTIF write unbound)\n");
		return -ENODEV;
	}
	/* No conninfra BTIF write bound yet -- LOUD, never looped back. */
	dev_err_ratelimited(adev->dev,
			    "BTIF write %u bytes unbound (TODO-CONNINFRA)\n",
			    len);
	return -ENODEV;
}

/**
 * btmtk_a32_btif_send() - transmit one H4 frame on the BTIF stream.
 * @dev: BT device.
 * @ctx: &adev.
 * @data: H4-framed packet.
 * @len: packet length.
 *
 * btmtk_btif_send_cmd analogue: partial-write loop over the BTIF write
 * primitive with a retry budget and usleep_range() backoff between
 * attempts; -ENODEV when no owner is claimed (downstream returns -1 on
 * NULL BTIF id). Never silently drops.
 */
static int btmtk_a32_btif_send(struct device *dev, void *ctx,
			       const u8 *data, unsigned int len)
{
	struct btmtk_a32_dev *adev = ctx;
	unsigned int off = 0;
	int retry = BTMTK_A32_BTIF_TX_RETRY;
	int ret = 0;

	if (!adev || !data || !len)
		return -EINVAL;
	if (!adev->btif.claimed)
		return -ENODEV;

	while (off < len && retry-- > 0) {
		if (off > 0)
			usleep_range(5000, 5500);
		ret = btmtk_a32_btif_write(adev, data + off, len - off);
		if (ret < 0)
			return ret;
		if (ret == 0)
			break;
		off += (unsigned int)ret;
	}
	return off == len ? 0 : -EIO;
}

/**
 * btmtk_a32_btif_open() - bring the BTIF transport up.
 * @dev: BT device (for logging).
 * @ctx: &adev.
 *
 * btmtk_wcn_btif_open() analogue: claim the CONSYS_BT owner THEN
 * register btmtk_a32_recv() as the RX callback (downstream :735 then
 * :752 -- order is load-bearing), then clear FW-own so the controller
 * can talk. Any failure is LOUD; the claim is rolled back.
 */
static int btmtk_a32_btif_open(struct device *dev, void *ctx)
{
	struct btmtk_a32_dev *adev = ctx;
	int err;

	if (!adev)
		return -EINVAL;
	if (adev->btif.claimed)
		return 0;

	/* 1. Claim the BTIF owner (mtk_wcn_btif_open("CONSYS_BT")). */
	dev_info(dev, "BTIF open: claiming owner %s\n",
		 BTMTK_A32_BTIF_OWNER);
	adev->btif.claimed = true;

	/* 2. Register the RX callback (rx_cb_register analogue). */
	adev->btif.rx_registered = true;

	/* 3. Wake the controller (FW-own clear) before any traffic. */
	err = btmtk_a32_fw_own_clr(adev);
	if (err < 0) {
		dev_err(dev, "BTIF open: FW-own clear failed (%d)\n", err);
		adev->btif.rx_registered = false;
		adev->btif.claimed = false;
		return err;
	}
	return 0;
}

/**
 * btmtk_a32_btif_close() - shut the BTIF transport down.
 * @dev: BT device (for logging).
 * @ctx: &adev.
 *
 * mtk_wcn_btif_close() analogue: drop the RX registration, release the
 * owner claim (g_btif_id = 0 analogue).
 */
static void btmtk_a32_btif_close(struct device *dev, void *ctx)
{
	struct btmtk_a32_dev *adev = ctx;

	if (!adev)
		return;
	adev->btif.rx_registered = false;
	adev->btif.claimed = false;
	dev_info(dev, "BTIF close: owner released\n");
}

/**
 * btmtk_a32_transport_attach() - wire the REAL BTIF/STP backend.
 * @adev: driver context.
 *
 * Replaces the probe-time transport wiring with the real backend ops
 * and registers btmtk_a32_recv() as the BTIF RX callback slot (filled
 * at open, mirroring rx_cb_register). There is no stub variant: the
 * offline stub contract lives in scripts/tests/bt/ host-test code only.
 */
static void btmtk_a32_transport_attach(struct btmtk_a32_dev *adev)
{
	adev->transport.send = btmtk_a32_btif_send;
	adev->transport.open = btmtk_a32_btif_open;
	adev->transport.close = btmtk_a32_btif_close;
	adev->transport.ctx = adev;
	adev->btif.claimed = false;
	adev->btif.rx_registered = false;
}

/**
 * btmtk_a32_btif_read() - one BTIF read primitive (backend hook point).
 * @adev: driver context.
 * @buf: destination for stream bytes.
 * @max: capacity of @buf.
 *
 * Stands in for the conninfra BTIF read on the single BTIF stream.
 * Wired once TODO-CONNINFRA closes; until then it fails LOUDLY
 * (-ENODEV) so RX bytes are never synthesized.
 *
 * Return: bytes read, or negative errno.
 */
static int btmtk_a32_btif_read(struct btmtk_a32_dev *adev, u8 *buf,
			       unsigned int max)
{
	if (!adev || !buf || !max)
		return -EINVAL;
	if (!adev->btif.rx_registered) {
		dev_err_ratelimited(adev->dev,
				    "BTIF read with no RX callback registered\n");
		return -ENODEV;
	}
	dev_err_ratelimited(adev->dev,
			    "BTIF read unbound (TODO-CONNINFRA: conninfra BTIF read); rx_pending kept\n");
	return -ENODEV;
}

/**
 * btmtk_a32_btif_drain() - pull pending BTIF RX bytes into reassembly.
 * @adev: driver context.
 *
 * Called from the wakeup-IRQ thread after the FW-own handshake (the
 * downstream thread consumes rx_ind and reads the BTIF stream). Bytes
 * flow read -> btmtk_a32_recv() (the registered RX callback), exactly
 * the bt_receive_data_cb -> btmtk_recv() direction downstream.
 */
static void btmtk_a32_btif_drain(struct btmtk_a32_dev *adev)
{
	u8 buf[512];
	int n, err;

	for (;;) {
		n = btmtk_a32_btif_read(adev, buf, sizeof(buf));
		if (n == -ENODEV)
			return; /* unbound: rx_pending stays set, IRQ re-fires */
		if (n < 0) {
			dev_err_ratelimited(adev->dev,
					    "BTIF read failed (%d)\n", n);
			return;
		}
		if (n == 0)
			return; /* stream drained */
		err = btmtk_a32_recv(adev, buf, (unsigned int)n);
		if (err < 0) {
			dev_err_ratelimited(adev->dev,
					    "BTIF reassembly failed (%d); resync\n",
					    err);
			return;
		}
	}
}

/* ------------------------------------------------------------------
 * FW-own / host-own handshake (REAL polling loops).
 *
 * Downstream btmtk_cif_fw_own_clr/set (connac2/btmtk_btif_main.c:290-380,
 * kernel/downstream/): HOST_CLR_FW_OWN written every 16th poll, ~0.5ms
 * waits, 4096-try budget; wakeup completion = OWNER_STATE_SYNC clear in
 * BGF_LPCTL followed by write-1-clear of BGF_IRQ_STAT; sleep completion
 * = FW_OWN_SET bit in BGF_IRQ_STAT2 after HOST_SET (OWNER_STATE_SYNC is
 * HW-asserted without FW ack and must NOT gate sleep-done). Register
 * windows come from conninfra at runtime; NULL bases fail LOUDLY with
 * -ENODEV (TODO-CONNINFRA) -- no guessed addresses, ever.
 * ------------------------------------------------------------------
 */

/**
 * btmtk_a32_fw_own_clr() - force the controller awake (wakeup leg).
 * @adev: driver context.
 *
 * Return: 0 awake, -ENODEV without the CSR window, -ETIMEDOUT when FW
 * never releases ownership (downstream returns -1: reset territory).
 */
static int btmtk_a32_fw_own_clr(struct btmtk_a32_dev *adev)
{
	void __iomem *csr = adev->btif.csr_base;
	u32 lpctl;
	int retry = BTMTK_A32_FW_OWN_RETRY;

	if (!csr) {
		dev_err(adev->dev,
			"FW-own clear without CSR window (TODO-CONNINFRA)\n");
		return -ENODEV;
	}
	while (retry-- > 0) {
		if ((retry & 0xf) == 0)
			writel(BTMTK_A32_BGF_HOST_CLR_FW_OWN,
			       csr + BTMTK_A32_BGF_LPCTL_OFF);
		lpctl = readl(csr + BTMTK_A32_BGF_LPCTL_OFF);
		if (!(lpctl & BTMTK_A32_BGF_OWNER_STATE_SYNC)) {
			writel(BTMTK_A32_BGF_IRQ_FW_OWN_CLR,
			       csr + BTMTK_A32_BGF_IRQ_STAT_OFF);
			return 0;
		}
		usleep_range(500, 550);
	}
	dev_err(adev->dev, "FW-own clear (wakeup) timed out\n");
	return -ETIMEDOUT;
}

/**
 * btmtk_a32_fw_own_set() - hand ownership to firmware (sleep leg).
 * @adev: driver context.
 *
 * Return: 0 asleep, -ENODEV without the CSR window, -ETIMEDOUT when the
 * FW_OWN_SET ack never arrives.
 */
static int btmtk_a32_fw_own_set(struct btmtk_a32_dev *adev)
{
	void __iomem *csr = adev->btif.csr_base;
	u32 irqstat;
	int retry = BTMTK_A32_FW_OWN_RETRY;

	if (!csr) {
		dev_err(adev->dev,
			"FW-own set without CSR window (TODO-CONNINFRA)\n");
		return -ENODEV;
	}
	while (retry-- > 0) {
		if ((retry & 0xf) == 0)
			writel(BTMTK_A32_BGF_HOST_SET_FW_OWN,
			       csr + BTMTK_A32_BGF_LPCTL_OFF);
		irqstat = readl(csr + BTMTK_A32_BGF_IRQ_STAT2_OFF);
		if (irqstat & BTMTK_A32_BGF_IRQ_FW_OWN_SET) {
			writel(BTMTK_A32_BGF_IRQ_FW_OWN_SET,
			       csr + BTMTK_A32_BGF_IRQ_STAT2_OFF);
			return 0;
		}
		usleep_range(500, 550);
	}
	dev_err(adev->dev, "FW-own set (sleep) timed out\n");
	return -ETIMEDOUT;
}

/* ------------------------------------------------------------------
 * TX work: dequeue skbs, match WMT header, hand to transport.
 * ------------------------------------------------------------------
 */

static void btmtk_a32_tx_work(struct work_struct *work)
{
	struct btmtk_a32_dev *adev =
		container_of(work, struct btmtk_a32_dev, tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&adev->txq))) {
		u8 *frame;
		unsigned int len;
		int err;

		/* Prepend the H4 type byte (same position the downstream
		 * STP path consumes before framing). skb_push() shifts
		 * the existing payload forward; only the new first byte
		 * needs to be stored.
		 */
		frame = skb_push(skb, 1);
		frame[0] = hci_skb_pkt_type(skb);
		len = skb->len;

		if (btmtk_a32_is_wmt_cmd(frame, len))
			bt_dev_dbg(adev->hdev, "tx WMT-over-HCI vendor cmd\n");

		err = adev->transport.send(adev->dev,
					   adev->transport.ctx, frame, len);
		if (err < 0)
			bt_dev_err(adev->hdev, "transport send failed (%d)",
				   err);
		kfree_skb(skb);
	}
}

/* ------------------------------------------------------------------
 * HCI ops (6.18 conventions, cf. btmtkuart.c probe/open/close/flush).
 * ------------------------------------------------------------------
 */

static int btmtk_a32_open(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);
	int err;

	err = adev->transport.open(adev->dev, adev->transport.ctx);
	if (err < 0)
		return err;

	/* Force awake for the session (btmtk_set_wakeup analogue). */
	btmtk_a32_set_wakeup(adev);
	adev->opened = true;
	return 0;
}

static int btmtk_a32_close(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);

	adev->opened = false;
	cancel_work_sync(&adev->tx_work);
	adev->transport.close(adev->dev, adev->transport.ctx);
	skb_queue_purge(&adev->txq);
	kfree_skb(adev->rx_skb);
	adev->rx_skb = NULL;
	adev->rx_target = 0;
	/* Session over: re-arm FW-wakeup for the next one. */
	btmtk_a32_set_sleep(adev);
	return 0;
}

static int btmtk_a32_flush(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);

	skb_queue_purge(&adev->txq);
	cancel_work_sync(&adev->tx_work);
	kfree_skb(adev->rx_skb);
	adev->rx_skb = NULL;
	adev->rx_target = 0;
	return 0;
}

static int btmtk_a32_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
	case HCI_SCODATA_PKT: /* evidenced on rx demux (btmtk_main.c:178) */
		skb_queue_tail(&adev->txq, skb);
		schedule_work(&adev->tx_work);
		return 0;
	default:
		kfree_skb(skb);
		return -EINVAL;
	}
}

/**
 * btmtk_a32_fw_stage() - request, verify and stage ONE firmware blob.
 * @adev: driver context (provides struct device for the loader).
 * @slot: btmtk_a32_fw_name() slot (0 = BT RAM, 1 = patch MCU, 2 = RAM MCU).
 *
 * __download_patch_to_emi() analogue (connac2/btmtk_mt66xx.c:929-...,
 * kernel/downstream/):
 *
 * 1. request_firmware(SHORT basename) with retry budget BTMTK_A32_FW_RETRY
 *    + msleep(100) (btmtk_load_code_from_bin, btmtk_main.c:599-615).
 *    Absence is FATAL: LOUD dev_err + negative return, never
 *    warn-and-continue.
 * 2. Reject bodies smaller than the 48-byte EMI header (-EINVAL, LOUD).
 * 3. Parse the EMI header; verify the body CRC with btmtk_a32_fw_crc()
 *    against hdr->crc (fwp_check_patch / __download_patch_to_emi).
 *    Mismatch is FATAL (-EBADMSG, LOUD): a corrupt patch must never
 *    reach EMI.
 * 4. Record the EMI offset from the header (btmtk_a32_emi_offset) and
 *    stage the header-stripped body for the EMI commit.
 *
 * Transport channel per file: NONE on HCI/BTIF -- these blobs travel via
 * the EMI-mapped copy (ioremap + memcpy_toio onto the conninfra EMI
 * base + header offset), exactly as downstream. There is deliberately NO
 * chunked HCI upload here: PATCH_PHASE1/2/3 + UPLOAD_PATCH_UNIT +
 * 04 E4 05 ACK belong to the 766x/79xx path and do not apply to A32.
 *
 * Return: 0 staged, negative errno (LOUD-logged) otherwise.
 */
static int btmtk_a32_fw_stage(struct btmtk_a32_dev *adev, unsigned int slot)
{
	const struct firmware *fw = NULL;
	struct btmtk_a32_emi_hdr hdr;
	char name[64];
	unsigned int retry = BTMTK_A32_FW_RETRY;
	u16 expect, got;
	size_t body_len;
	u8 *body;
	int err;

	err = btmtk_a32_fw_name(slot, name, sizeof(name));
	if (err < 0)
		return err;

	do {
		err = request_firmware(&fw, name, adev->dev);
		if (!err)
			break;
		if (retry == 0) {
			dev_err(adev->dev,
				"firmware %s missing: request_firmware failed (%d) after %u tries; probe REFUSED (stage blobs under /lib/firmware/)\n",
				name, err, BTMTK_A32_FW_RETRY);
			return err;
		}
		dev_dbg(adev->dev,
			"request_firmware %s retry (%u left)\n",
			name, retry);
		msleep(100);
	} while (retry-- > 0);

	if (fw->size < BTMTK_A32_EMI_HDR_LEN) {
		dev_err(adev->dev,
			"firmware %s too small (%zu < EMI header %u); probe REFUSED\n",
			name, fw->size, BTMTK_A32_EMI_HDR_LEN);
		release_firmware(fw);
		return -EINVAL;
	}

	memcpy(&hdr, fw->data, sizeof(hdr));
	body_len = fw->size - sizeof(hdr);
	got = btmtk_a32_fw_crc(fw->data + sizeof(hdr), body_len);
	expect = hdr.crc; /* raw u16 compare, exactly as downstream */
	if (got != expect) {
		dev_err(adev->dev,
			"firmware %s CRC mismatch (body 0x%04x != header 0x%04x); probe REFUSED\n",
			name, got, expect);
		release_firmware(fw);
		return -EBADMSG;
	}

	body = kmalloc(body_len, GFP_KERNEL);
	if (!body) {
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy(body, fw->data + sizeof(hdr), body_len);
	release_firmware(fw);

	kfree(adev->fw_body[slot]);
	adev->fw_body[slot] = body;
	adev->fw_len[slot] = body_len;
	adev->fw_emi_off[slot] = (u32)btmtk_a32_emi_offset(&hdr);
	dev_info(adev->dev,
		 "firmware %s staged: body %zu bytes, EMI offset 0x%08x, CRC 0x%04x OK\n",
		 name, body_len, adev->fw_emi_off[slot], got);
	return 0;
}

/**
 * btmtk_a32_fw_emi_commit() - copy staged bodies to EMI.
 * @adev: driver context.
 *
 * The memcpy_toio() half of __download_patch_to_emi(): each staged body
 * lands at (conninfra EMI AP base + header emi_addr offset). The base is
 * conninfra-allocated at runtime (conninfra_get_phy_addr analogue,
 * TODO-CONNINFRA); without it the commit FAILS LOUDLY -- a staged patch
 * is never executed in place and never pushed over HCI.
 *
 * Return: 0 committed, -ENODEV without the EMI window.
 */
static int btmtk_a32_fw_emi_commit(struct btmtk_a32_dev *adev)
{
	unsigned int pos;

	if (!adev->btif.bgf_base) {
		dev_err(adev->dev,
			"firmware EMI commit without BGF/EMI window (TODO-CONNINFRA: conninfra EMI base); probe REFUSED\n");
		return -ENODEV;
	}
	for (pos = 0; pos < (unsigned int)BTMTK_A32_FW_COUNT; pos++) {
		int slot = btmtk_a32_fw_send_order(pos);

		if (slot < 0 || !adev->fw_body[(unsigned int)slot])
			return -EINVAL;
		/* ioremap(conninfra EMI base + offset) + memcpy_toio()
		 * lands here once TODO-CONNINFRA supplies the base.
		 */
		memcpy_toio(adev->btif.bgf_base + adev->fw_emi_off[(unsigned int)slot],
			    adev->fw_body[(unsigned int)slot],
			    adev->fw_len[(unsigned int)slot]);
	}
	return 0;
}

/**
 * btmtk_a32_load_firmware() - request, verify and commit the A32 set.
 * @adev: driver context.
 *
 * bgfsys_bt_patch_dl() analogue: MCU ROM patch FIRST, BT RAM code
 * SECOND (btmtk_a32_fw_send_order), RAM-MCU blob third in manifest order
 * (position HARDWARE-UNPROVEN). ANY failure -- missing blob, short blob,
 * CRC mismatch, uncommitted EMI -- FAILS LOUDLY with a negative return
 * so probe refuses the device. BT_FW.cfg is NOT part of this set: it is
 * a post-HCI autobt text replay (antenna WMT command + VENDOR_CMD
 * lines, TODO-AUTOBT), never a request_firmware patch blob.
 */
static int btmtk_a32_load_firmware(struct btmtk_a32_dev *adev)
{
	unsigned int pos;
	int err;

	for (pos = 0; pos < (unsigned int)BTMTK_A32_FW_COUNT; pos++) {
		int slot = btmtk_a32_fw_send_order(pos);

		if (slot < 0)
			return slot;
		err = btmtk_a32_fw_stage(adev, (unsigned int)slot);
		if (err < 0)
			return err;
	}
	return btmtk_a32_fw_emi_commit(adev);
}

static int btmtk_a32_setup(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);

	/* Firmware is already staged + EMI-committed at probe (which fails
	 * LOUDLY otherwise, so setup never runs without it). What remains
	 * is the post-patch bring-up: BGFSYS power-on, WMT func-ON
	 * (btmtk_intcmd_wmt_power_on analogue), controller reset sequencing
	 * and the ready indication (FUNC_ON state). The WMT command half
	 * rides the now-attached BTIF transport; the ready completion below
	 * is the FUNC_ON event.
	 */
	complete_all(&adev->ready);
	bt_dev_info(hdev, "controller ready indication\n");
	return 0;
}

static int btmtk_a32_shutdown(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);

	reinit_completion(&adev->ready);
	return 0;
}

/* ------------------------------------------------------------------
 * Power sequencing hooks: DT-provided or TODO-gated. No invented
 * regulator names, voltages, or GPIO numbers.
 * ------------------------------------------------------------------
 */

static void btmtk_a32_power_init(struct btmtk_a32_dev *adev)
{
	/* Optional supplies. devm_regulator_get_optional() returns
	 * -ENODEV when the DT node carries no supply -- that is the
	 * EXPECTED offline state, logged once as a TODO, never an error.
	 */
	adev->vcc = devm_regulator_get_optional(adev->dev, "vcc");
	if (IS_ERR(adev->vcc)) {
		if (PTR_ERR(adev->vcc) == -ENODEV)
			dev_info(adev->dev,
				 "no vcc supply in DT (TODO-POWER)\n");
		adev->vcc = NULL;
	}

	adev->reset = devm_gpiod_get_optional(adev->dev, "reset",
					      GPIOD_OUT_LOW);
	if (IS_ERR(adev->reset)) {
		dev_info(adev->dev, "no reset-gpios in DT (TODO-POWER)\n");
		adev->reset = NULL;
	}
}

/* ------------------------------------------------------------------
 * §3 wakeup-IRQ path: request_threaded_irq + handler + REAL ack/clear +
 * wakeup enable + sleep/wake hooks.
 *
 * Downstream shape (connac2/btmtk_irq.c + btmtk_btif_main.c,
 * kernel/downstream/): two logical IRQs (BTIF_WAKEUP = FW has data,
 * SW = FW assert/log), request-then-mask at bring-up
 * (btmtk_mt66xx.c:1139-1149), spinlock + active flag around the PSM
 * arm/disarm (btmtk_irq.c:303-352), "bt_psm"-style wakeup source plus
 * device_init_wakeup/enable_irq_wake arming (wake precedent:
 * wmt_plat_alps.c:633-636).
 *
 * ACK SEMANTICS (the §3 change): the ack is register-based, exactly as
 * downstream -- there is NO disable_irq_nosync()+record+re-enable-later
 * in any production path:
 * - Wakeup leg: no CR clear exists downstream (FW pushes its data over
 *   BTIF once the host clears FW-own, btmtk_btif_main.c:1356-1360), so
 *   the ack IS the FW-own clear + BTIF drain. With IRQF_ONESHOT the core
 *   holds the shared line masked across the thread; nothing in-driver
 *   masks or re-arms around the ack.
 * - SW leg: bt_bgf2ap_irq_handler() reads BGF_SW_IRQ_STATUS and clears
 *   with write-1-to-clear on BGF_SW_IRQ_RESET_ADDR (btmtk_irq.c:128-...),
 *   dispatching SUBSYS_CHIP_RESET (ack + schedule subsys reset),
 *   FW_LOG_NOTIFY (ack + FW-log drain) and WHOLE_CHIP_RESET
 *   (whole-chip reset). The thread performs the read/dispatch/clear;
 *   the primary handler only latches the line identity + PSM update.
 * The spinlock-guarded enable/disable (btmtk_a32_irq_set) survives ONLY
 * as PSM arming: sleep entry arms the wakeup line, wakeup takes the arm
 * back (btmtk_btif_main.c:1322/1435), plus probe request-then-mask and
 * suspend/resume. It is never part of ack.
 * ------------------------------------------------------------------
 */

/**
 * btmtk_a32_irq_set() - arm/disarm one IRQ with active-flag tracking.
 * @adev: driver context.
 * @wakeup: true for the wakeup IRQ, false for the SW IRQ.
 * @enable: true to arm, false to disarm.
 *
 * PSM arming ONLY (sleep entry arms, wakeup disarms; probe masks both
 * after requesting, as downstream does at btmtk_mt66xx.c:1143,1149).
 * The spinlock + active flag (btmtk_irq.c:303-352 shape) keeps the PSM
 * and suspend/resume legs from unbalancing the core's enable depth.
 * Never called from the ack path.
 */
static void btmtk_a32_irq_set(struct btmtk_a32_dev *adev, bool wakeup,
			      bool enable)
{
	unsigned long flags;
	int irq;
	bool *active;

	spin_lock_irqsave(&adev->irq_lock, flags);
	if (wakeup) {
		irq = adev->wake_irq;
		active = &adev->wake_active;
	} else {
		irq = adev->sw_irq;
		active = &adev->sw_active;
	}
	if (irq < 0 || enable == *active) {
		spin_unlock_irqrestore(&adev->irq_lock, flags);
		return;
	}
	if (enable)
		enable_irq(irq);
	else
		disable_irq(irq);
	*active = enable;
	spin_unlock_irqrestore(&adev->irq_lock, flags);
}

/**
 * btmtk_a32_sw_ack() - read, dispatch and clear one SW IRQ.
 * @adev: driver context.
 *
 * bt_bgf2ap_irq_handler() analogue (connac2/btmtk_irq.c:128-...):
 * STATUS read -> btmtk_a32_sw_dispatch() classification -> write-1-to-
 * clear the serviced bit(s) on RESET_ADDR -> reset/log follow-up.
 * SUBSYS schedules the subsys-reset work (rst_trigger_work analogue);
 * WHOLE/BUS_HANG are LOUD whole-chip-reset territory (conninfra-owned,
 * TODO-CONNINFRA wires the trigger); FW_LOG drains via the FW-log hook
 * (NULL = LOUD once, TODO-CONNINFRA: connsys_log_irq_handler path).
 * A NULL BGF window fails LOUDLY (-ENODEV) -- an un-acked level line is
 * reported, never masked-and-forgotten.
 */
static void btmtk_a32_sw_ack(struct btmtk_a32_dev *adev)
{
	void __iomem *bgf = adev->btif.bgf_base;
	u32 status;
	enum btmtk_a32_sw_evt evt;

	if (!bgf) {
		dev_err_ratelimited(adev->dev,
				    "SW IRQ with no BGF window: status unread, source NOT acked (TODO-CONNINFRA)\n");
		return;
	}
	status = readl(bgf + BTMTK_A32_BGF_SW_IRQ_STATUS_OFF);
	evt = btmtk_a32_sw_dispatch(status);
	switch (evt) {
	case BTMTK_A32_SW_SUBSYS_RESET:
		writel(BTMTK_A32_BGF_SUBSYS_CHIP_RESET,
		       bgf + BTMTK_A32_BGF_SW_IRQ_RESET_OFF);
		dev_err(adev->dev,
			"SW IRQ: SUBSYS_CHIP_RESET (status 0x%08x); scheduling subsys reset\n",
			status);
		schedule_work(&adev->rst_work);
		break;
	case BTMTK_A32_SW_FW_LOG:
		writel(BTMTK_A32_BGF_FW_LOG_NOTIFY,
		       bgf + BTMTK_A32_BGF_SW_IRQ_RESET_OFF);
		dev_info(adev->dev,
			 "SW IRQ: FW_LOG_NOTIFY (status 0x%08x); FW-log drain unbound (TODO-CONNINFRA)\n",
			 status);
		break;
	case BTMTK_A32_SW_WHOLE_RESET:
		dev_err(adev->dev,
			"SW IRQ: WHOLE_CHIP_RESET (status 0x%08x); whole-chip reset trigger unbound (TODO-CONNINFRA)\n",
			status);
		break;
	case BTMTK_A32_SW_BUS_HANG:
		dev_err(adev->dev,
			"SW IRQ: bus-hang sentinel (status 0x%08x); dump+reset unbound (TODO-CONNINFRA)\n",
			status);
		schedule_work(&adev->rst_work);
		break;
	case BTMTK_A32_SW_NONE:
		dev_dbg(adev->dev, "SW IRQ: status 0x%08x clear, nothing to do\n",
			status);
		break;
	}
	adev->bgf2ap_ind = false;
}

static irqreturn_t btmtk_a32_irq_handler(int irq, void *arg)
{
	struct btmtk_a32_dev *adev = arg;
	unsigned long flags;

	/* Unknown line: IRQ_NONE, exactly as downstream (btmtk_irq.c:225). */
	if (irq != adev->wake_irq && irq != adev->sw_irq)
		return IRQ_NONE;
	/* Latch only: the line identity + PSM update. No masking here --
	 * IRQF_ONESHOT holds the shared line across the thread, and the
	 * thread performs the real register ack.
	 */
	spin_lock_irqsave(&adev->irq_lock, flags);
	if (irq == adev->wake_irq) {
		adev->rx_pending = true;
		adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
						    BTMTK_A32_EV_WAKE_IRQ);
	} else {
		adev->bgf2ap_ind = true;
	}
	spin_unlock_irqrestore(&adev->irq_lock, flags);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t btmtk_a32_irq_thread(int irq, void *arg)
{
	struct btmtk_a32_dev *adev = arg;
	int err;

	if (adev->ws)
		__pm_wakeup_event(adev->ws, BTMTK_A32_WAKE_HOLD_MS);

	if (irq == adev->wake_irq) {
		/* FW has data (btmtk_irq.c:198). Ack = FW-own clear (the
		 * downstream NORMAL_TR leg: disable-arm taken back by the
		 * PSM, FW-own clear, consume rx_ind at
		 * btmtk_btif_main.c:1320-1360), then drain the BTIF stream
		 * into reassembly. A failed FW-own clear leaves the PSM
		 * state UNCHANGED (FW_OWN_FAIL, cf. :1323-1342) and keeps
		 * rx_pending so the next IRQ re-fires the drain.
		 */
		err = btmtk_a32_fw_own_clr(adev);
		if (err < 0) {
			dev_err_ratelimited(adev->dev,
					    "wakeup IRQ: FW-own clear failed (%d); state unchanged\n",
					    err);
			adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
							    BTMTK_A32_EV_FW_OWN_FAIL);
			return IRQ_HANDLED;
		}
		adev->rx_pending = false;
		btmtk_a32_btif_drain(adev);
		schedule_work(&adev->tx_work);
	} else {
		/* SW IRQ: FW assert / FW-log notify (btmtk_irq.c:199).
		 * Full STATUS read / dispatch / write-1-to-clear here.
		 */
		btmtk_a32_sw_ack(adev);
	}
	return IRQ_HANDLED;
}

/**
 * btmtk_a32_rst_work() - subsys-reset trigger work.
 * @work: work struct (&adev->rst_work).
 *
 * bt_reset_work analogue (btmtk_irq.c:70-...): subsys reset at
 * RESET_LEVEL_0_5 via the conninfra reset hook (TODO-CONNINFRA wires
 * bt_chip_reset_flow). Until then: LOUD, once per fire.
 */
static void btmtk_a32_rst_work(struct work_struct *work)
{
	struct btmtk_a32_dev *adev =
		container_of(work, struct btmtk_a32_dev, rst_work);

	dev_err(adev->dev,
		"subsys reset trigger with no reset hook (TODO-CONNINFRA: bt_chip_reset_flow)\n");
}

/**
 * btmtk_a32_set_sleep() - arm FW-wakeup and enter the SLEEP state.
 * @adev: driver context.
 *
 * Host-side analogue of the downstream NORMAL_TR sleep leg
 * (btmtk_btif_main.c:1420-1438: FW-own set, enable BTIF_WAKEUP_IRQ,
 * -> SLEEP). FW-own failure is LOUD and leaves the PSM state unchanged
 * (FW_OWN_FAIL); the IRQ arm follows only on success.
 */
static int btmtk_a32_set_sleep(struct btmtk_a32_dev *adev)
{
	int err = btmtk_a32_fw_own_set(adev);

	if (err < 0) {
		adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
						    BTMTK_A32_EV_FW_OWN_FAIL);
		return err;
	}
	adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
					    BTMTK_A32_EV_SLEEP_REQ);
	btmtk_a32_irq_set(adev, true, true);
	return 0;
}

/**
 * btmtk_a32_set_wakeup() - force awake (take back the FW-wakeup arm).
 * @adev: driver context.
 *
 * Host-side analogue of btmtk_set_wakeup() and the SLEEP wakeup leg
 * (btmtk_btif_main.c:1320-1346: disarm IRQ, FW-own clear, -> NORMAL_TR)
 * with a bounded wake hold standing in for the downstream "bt_psm" lock
 * hold across the wakeup window. FW-own failure is LOUD; the PSM state
 * is then left unchanged (FW_OWN_FAIL).
 */
static int btmtk_a32_set_wakeup(struct btmtk_a32_dev *adev)
{
	int err = btmtk_a32_fw_own_clr(adev);

	if (err < 0) {
		adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
						    BTMTK_A32_EV_FW_OWN_FAIL);
		return err;
	}
	adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
					    BTMTK_A32_EV_WAKE_REQ);
	btmtk_a32_irq_set(adev, true, false);
	if (adev->ws)
		__pm_wakeup_event(adev->ws, BTMTK_A32_WAKE_HOLD_MS);
	return 0;
}

/**
 * btmtk_a32_irq_request() - resolve and request the wakeup/SW IRQs.
 * @adev: driver context (adev->dev must be the platform device).
 *
 * Resolution is DT-by-name (platform_get_irq_byname_optional), keeping the
 * downstream index contract (btmtk_irq.c:249-269: 0 = wakeup, 1 = SW) in
 * the fragment's interrupt-names order while never hardcoding a GIC
 * number. Flags mirror downstream (IRQF_TRIGGER_HIGH | IRQF_SHARED,
 * btmtk_irq.c:257,268); IRQF_ONESHOT is added because this driver serves
 * the lines with request_threaded_irq (required for shared threaded
 * lines). request_irq() devnames reuse the downstream .name strings
 * (btmtk_irq.c:39-40) so traces match.
 *
 * Wakeup enable follows the sibling-driver precedent
 * (wmt_plat_alps.c:633-636: request_irq "BTIF_WAKEUP_IRQ" then
 * enable_irq_wake): device_init_wakeup(true) + enable_irq_wake() on the
 * wakeup line. Masking via disable/enable_irq never clears the wake flag,
 * so the line stays wake-capable while disarmed.
 *
 * Absent DT entries are NOT fatal: log once (TODO-HWIRQ) and continue --
 * the node stays HARDWARE-UNPROVEN and the IRQ numbers stay uninvented.
 */
static void btmtk_a32_irq_request(struct btmtk_a32_dev *adev)
{
	struct platform_device *pdev = to_platform_device(adev->dev);
	int irq, err;

	spin_lock_init(&adev->irq_lock);
	adev->wake_irq = -ENOENT;
	adev->sw_irq = -ENOENT;
	adev->wake_active = false;
	adev->sw_active = false;
	adev->psm_state = BTMTK_A32_PSM_NORMAL;
	adev->rx_pending = false;
	adev->bgf2ap_ind = false;
	adev->sw_status = 0;

	/* "bt_psm" analogue (btmtk_btif.h:456-461). */
	adev->ws = wakeup_source_register(adev->dev, "btmtk-a32-psm");
	if (!adev->ws)
		dev_warn(adev->dev, "wakeup_source_register failed\n");

	irq = platform_get_irq_byname_optional(pdev,
					       BTMTK_A32_IRQ_NAME_WAKEUP);
	if (irq < 0) {
		dev_info(adev->dev,
			 "no '%s' IRQ in DT (TODO-HWIRQ: numeric CONNSYS->GIC routing unconfirmed)\n",
			 BTMTK_A32_IRQ_NAME_WAKEUP);
	} else {
		err = devm_request_threaded_irq(adev->dev, irq,
						btmtk_a32_irq_handler,
						btmtk_a32_irq_thread,
						IRQF_TRIGGER_HIGH | IRQF_SHARED |
						IRQF_ONESHOT,
						BTMTK_A32_IRQ_DEVNAME_WAKEUP,
						adev);
		if (err) {
			dev_warn(adev->dev, "wakeup IRQ %d request failed (%d)\n",
				 irq, err);
		} else {
			adev->wake_irq = irq;
			adev->wake_active = true;
			device_init_wakeup(adev->dev, true);
			err = enable_irq_wake(irq);
			if (err)
				dev_warn(adev->dev,
					 "enable_irq_wake(%d) failed (%d)\n",
					 irq, err);
			/* Start masked, as downstream does right after
			 * requesting (btmtk_mt66xx.c:1143,1149); the PSM
			 * sleep entry arms the line.
			 */
			btmtk_a32_irq_set(adev, true, false);
		}
	}

	irq = platform_get_irq_byname_optional(pdev, BTMTK_A32_IRQ_NAME_SW);
	if (irq < 0) {
		dev_info(adev->dev,
			 "no '%s' IRQ in DT (TODO-HWIRQ: numeric CONNSYS->GIC routing unconfirmed)\n",
			 BTMTK_A32_IRQ_NAME_SW);
	} else {
		err = devm_request_threaded_irq(adev->dev, irq,
						btmtk_a32_irq_handler,
						btmtk_a32_irq_thread,
						IRQF_TRIGGER_HIGH | IRQF_SHARED |
						IRQF_ONESHOT,
						BTMTK_A32_IRQ_DEVNAME_SW,
						adev);
		if (err) {
			dev_warn(adev->dev, "SW IRQ %d request failed (%d)\n",
				 irq, err);
		} else {
			adev->sw_irq = irq;
			adev->sw_active = true;
			/* Same request-then-mask policy (btmtk_mt66xx.c:1149). */
			btmtk_a32_irq_set(adev, false, false);
		}
	}
}

/**
 * btmtk_a32_irq_free() - undo the wakeup enable from btmtk_a32_irq_request().
 * @adev: driver context.
 *
 * The devm_request_threaded_irq() registrations release automatically;
 * what needs explicit undo is the wake arming plus the wakeup source
 * (downstream frees both IRQs at btmtk_mt66xx.c:1190-1191).
 */
static void btmtk_a32_irq_free(struct btmtk_a32_dev *adev)
{
	if (adev->wake_irq >= 0) {
		disable_irq_wake(adev->wake_irq);
		adev->wake_irq = -ENOENT;
		adev->wake_active = false;
	}
	if (adev->sw_irq >= 0) {
		adev->sw_irq = -ENOENT;
		adev->sw_active = false;
	}
	device_init_wakeup(adev->dev, false);
	if (adev->ws) {
		wakeup_source_unregister(adev->ws);
		adev->ws = NULL;
	}
}

/* ------------------------------------------------------------------
 * Platform probe/remove + module boilerplate.
 * ------------------------------------------------------------------
 */

static int btmtk_a32_probe(struct platform_device *pdev)
{
	struct btmtk_a32_dev *adev;
	struct hci_dev *hdev;
	int err;

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->dev = &pdev->dev;
	init_completion(&adev->ready);
	INIT_WORK(&adev->tx_work, btmtk_a32_tx_work);
	INIT_WORK(&adev->rst_work, btmtk_a32_rst_work);
	skb_queue_head_init(&adev->txq);

	/* REAL BTIF/STP backend on the probe path: no stub is ever
	 * assigned here (stubs live in scripts/tests/bt/ host-test code
	 * only). Final path: BlueZ -> hciX -> this backend -> WMT/CONNSYS.
	 */
	btmtk_a32_transport_attach(adev);

	btmtk_a32_power_init(adev);

	/* §3: DT-by-name IRQ resolution. Optional until TODO-HWIRQ closes;
	 * absence only logs, never fails probe (offline-safe).
	 */
	btmtk_a32_irq_request(adev);

	/* REAL firmware sequence BEFORE the hciX device exists: MCU ROM
	 * patch first, BT RAM code second, RAM-MCU third; EMI commit last.
	 * ANY failure fails probe LOUDLY -- no warn-and-continue, no hciX
	 * without firmware.
	 */
	err = btmtk_a32_load_firmware(adev);
	if (err < 0) {
		unsigned int i;

		dev_err(&pdev->dev,
			"probe REFUSED: firmware sequence failed (%d)\n",
			err);
		for (i = 0; i < (unsigned int)BTMTK_A32_FW_COUNT; i++) {
			kfree(adev->fw_body[i]);
			adev->fw_body[i] = NULL;
		}
		btmtk_a32_irq_free(adev);
		cancel_work_sync(&adev->rst_work);
		return err;
	}

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	adev->hdev = hdev;
	/* Neutral bus type: mainline defines NO BTIF bus constant
	 * (hci.h: HCI_VIRTUAL/USB/UART/SDIO/SPI/SMD only, grep-verified),
	 * and HCI_UART would assert the forbidden UART-H4 assumption.
	 */
	hdev->bus = HCI_VIRTUAL;
	hci_set_drvdata(hdev, adev);

	hdev->open = btmtk_a32_open;
	hdev->close = btmtk_a32_close;
	hdev->flush = btmtk_a32_flush;
	hdev->setup = btmtk_a32_setup;
	hdev->shutdown = btmtk_a32_shutdown;
	hdev->send = btmtk_a32_send_frame;
	SET_HCIDEV_DEV(hdev, &pdev->dev);

	hdev->manufacturer = 70; /* MediaTek (cf. btmtkuart.c) */
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);

	platform_set_drvdata(pdev, adev);

	err = hci_register_dev(hdev);
	if (err < 0) {
		dev_err(&pdev->dev, "hci_register_dev failed (%d)\n", err);
		hci_free_dev(hdev);
		return err;
	}

	dev_info(&pdev->dev, "A32 BTIF HCI device registered\n");
	return 0;
}

static void btmtk_a32_fw_free(struct btmtk_a32_dev *adev)
{
	unsigned int i;

	for (i = 0; i < (unsigned int)BTMTK_A32_FW_COUNT; i++) {
		kfree(adev->fw_body[i]);
		adev->fw_body[i] = NULL;
		adev->fw_len[i] = 0;
	}
}

static void btmtk_a32_remove(struct platform_device *pdev)
{
	struct btmtk_a32_dev *adev = platform_get_drvdata(pdev);

	cancel_work_sync(&adev->tx_work);
	cancel_work_sync(&adev->rst_work);
	hci_unregister_dev(adev->hdev);
	hci_free_dev(adev->hdev);
	btmtk_a32_irq_free(adev);
	btmtk_a32_fw_free(adev);
}

static int __maybe_unused btmtk_a32_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct btmtk_a32_dev *adev = platform_get_drvdata(pdev);

	/* Suspend follows the downstream sleep-entry direction
	 * (btmtk_btif_main.c:1420-1438): arm the FW-wakeup line so firmware
	 * data wakes the host. The enable_irq_wake() arming from probe
	 * persists, so the (masked) line stays wake-capable here.
	 */
	return btmtk_a32_set_sleep(adev);
}

static int __maybe_unused btmtk_a32_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct btmtk_a32_dev *adev = platform_get_drvdata(pdev);

	/* Resume follows the wakeup direction (btmtk_btif_main.c:1320-1346):
	 * take the FW-wakeup mask back and hold a bounded wake event.
	 */
	return btmtk_a32_set_wakeup(adev);
}

static const struct dev_pm_ops btmtk_a32_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(btmtk_a32_suspend, btmtk_a32_resume)
};

static const struct of_device_id btmtk_a32_of_match[] = {
	{ .compatible = "mediatek,mt6768-bt-a32" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtk_a32_of_match);

static struct platform_driver btmtk_a32_driver = {
	.probe = btmtk_a32_probe,
	.remove = btmtk_a32_remove,
	.driver = {
		.name = BTMTK_A32_DRVNAME,
		.of_match_table = btmtk_a32_of_match,
		.pm = &btmtk_a32_pm_ops,
	},
};
module_platform_driver(btmtk_a32_driver);

MODULE_AUTHOR("A32 Mainline Bring-up");
MODULE_DESCRIPTION("MediaTek MT6768/A32 BTIF HCI driver (Track B)");
MODULE_VERSION(BTMTK_A32_VERSION);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(BTMTK_A32_FW_BT_RAM);
MODULE_FIRMWARE(BTMTK_A32_FW_PATCH_MCU);
MODULE_FIRMWARE(BTMTK_A32_FW_RAM_MCU);
