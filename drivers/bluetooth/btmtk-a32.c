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
 * struct btmtk_a32_transport - BTIF/STP send/recv abstraction.
 * @send: transmit one H4-framed packet on the transport.
 * @open: bring the transport up (BTIF open analogue).
 * @close: shut the transport down.
 * @ctx: backend context (offline stub: &adev).
 * @tx_frames: stub accounting: frames handed to @send.
 * @tx_bytes: stub accounting: bytes handed to @send.
 *
 * Mirrors the downstream split: btmtk_btif_open() + rx_cb_register()
 * (btmtk_btif_main.c:729,752) on the transport side, h4_recv_buf()
 * demux (btmtk_main.c:43,345-360) on the HCI side. The offline stub
 * wired at probe implements the same call contract with counters so
 * open/close/send/flush paths are exercised without silicon; the real
 * BTIF backend (conninfra/WMT context) attaches through
 * btmtk_a32_transport_attach() once hardware gates close (TODO-BTIF).
 */
struct btmtk_a32_transport {
	int (*send)(struct device *dev, void *ctx,
		    const u8 *data, unsigned int len);
	int (*open)(struct device *dev, void *ctx);
	void (*close)(struct device *dev, void *ctx);
	void *ctx;
	/* Offline-stub accounting (proves the contract is exercised). */
	unsigned long tx_frames;
	unsigned long tx_bytes;
};

struct btmtk_a32_dev {
	struct hci_dev *hdev;
	struct device *dev;
	struct btmtk_a32_transport transport;
	struct work_struct tx_work;
	struct sk_buff_head txq;
	struct sk_buff *rx_skb;
	unsigned int rx_target; /* full H4 length of the frame in rx_skb */
	struct completion ready;
	bool opened;
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
};

static int btmtk_a32_stub_send(struct device *dev, void *ctx,
			       const u8 *data, unsigned int len);
static int btmtk_a32_stub_open(struct device *dev, void *ctx);
static void btmtk_a32_stub_close(struct device *dev, void *ctx);
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
 * __maybe_unused: no in-tree caller until the real BTIF backend
 * registers this as its rx callback (TODO-BTIF; downstream analogue:
 * rx_cb_register at btmtk_btif_main.c:752). The offline stub transport
 * has no RX source.
 */
__maybe_unused static int btmtk_a32_recv(struct btmtk_a32_dev *adev,
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

/* Offline stub transport: implements the BTIF-shaped send/open/close
 * contract with accounting so the HCI paths are exercised without
 * silicon. The real BTIF backend (conninfra/WMT context) attaches by
 * replacing these hooks (TODO-BTIF) once hardware gates close.
 */

static int btmtk_a32_stub_send(struct device *dev, void *ctx,
			       const u8 *data, unsigned int len)
{
	struct btmtk_a32_dev *adev = ctx;

	if (!adev || !data || !len)
		return -EINVAL;
	adev->transport.tx_frames++;
	adev->transport.tx_bytes += len;
	dev_dbg(dev, "stub tx frame %lu bytes %u\n",
		adev->transport.tx_frames, len);
	return 0;
}

static int btmtk_a32_stub_open(struct device *dev, void *ctx)
{
	dev_dbg(dev, "stub transport open (TODO-BTIF: bind real BTIF)\n");
	return 0;
}

static void btmtk_a32_stub_close(struct device *dev, void *ctx)
{
	dev_dbg(dev, "stub transport close\n");
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
 * btmtk_a32_load_firmware() - request the exact A32 firmware set.
 * @adev: driver context (provides struct device for the loader).
 *
 * Downstream pattern: request_firmware(&fw_entry, bin_name, dev) with
 * retry (btmtk_main.c:598-613), SHORT bin_name, no mediatek/ prefix.
 * Order and names from btmtk-a32.h (vendor manifest a32-vendor.mk
 * :590-:593). BT_FW.cfg is NOT fetched here: it is a post-HCI
 * autobt replay script, not a loader blob (see docs; TODO-AUTOBT).
 */
static int btmtk_a32_load_firmware(struct btmtk_a32_dev *adev)
{
	unsigned int i;

	for (i = 0; i < (unsigned int)BTMTK_A32_FW_COUNT; i++) {
		const struct firmware *fw = NULL;
		char name[64];
		unsigned int retry = BTMTK_A32_FW_RETRY;
		int err;

		err = btmtk_a32_fw_name(i, name, sizeof(name));
		if (err < 0)
			return err;

		do {
			err = request_firmware(&fw, name, adev->dev);
			if (!err)
				break;
			if (retry == 0) {
				bt_dev_err(adev->hdev,
					   "request_firmware %s failed (%d)",
					   name, err);
				return err;
			}
			bt_dev_dbg(adev->hdev,
				   "request_firmware %s retry (%u left)",
				   name, retry);
			msleep(100);
		} while (retry-- > 0);

		bt_dev_info(adev->hdev, "firmware %s size %zu\n",
			    name, fw->size);
		release_firmware(fw);
	}
	return 0;
}

static int btmtk_a32_setup(struct hci_dev *hdev)
{
	struct btmtk_a32_dev *adev = hci_get_drvdata(hdev);
	int err;

	/* Controller reset + firmware + ready indication. On silicon this
	 * is where the WMT semaphore / patch-download / func-ctrl sequence
	 * (upstream btmtkuart_setup() pattern) runs over the BTIF shim;
	 * offline the loader step is what validates the name contract.
	 * Firmware absence (-ENOENT) is NOT fatal here: it keeps the hciX
	 * interface present for inspection while flagging the missing
	 * blob (HARDWARE-UNPROVEN until staged under /lib/firmware/).
	 */
	err = btmtk_a32_load_firmware(adev);
	if (err < 0)
		bt_dev_warn(hdev, "firmware load deferred (%d); TODO-FW\n",
			    err);

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
 * §3 wakeup-IRQ path: request_threaded_irq + handler + ack/clear +
 * wakeup enable + sleep/wake hooks.
 *
 * Downstream shape preserved (btmtk_irq.c:39,211 + btmtk_mt66xx.c:1139):
 * two logical IRQs (BTIF_WAKEUP = FW has data, SW = FW assert/log),
 * request-then-mask at bring-up, disable-first handler that defers the
 * real work, spinlock + active flag around enable/disable
 * (btmtk_irq.c:303-352), and a "bt_psm"-style wakeup source plus
 * device_init_wakeup/enable_irq_wake arming (wake precedent:
 * wmt_plat_alps.c:633-636).
 * ------------------------------------------------------------------
 */

/**
 * btmtk_a32_irq_set() - mask/unmask one IRQ with active-flag tracking.
 * @adev: driver context.
 * @wakeup: true for the wakeup IRQ, false for the SW IRQ.
 * @enable: true to enable, false to disable (nosync, handler-safe).
 *
 * Mirrors bt_enable_irq()/bt_disable_irq() (btmtk_irq.c:303-352): the
 * spinlock + active flag make enable/disable idempotent so the handler
 * (disable leg), the IRQ thread (re-arm leg) and the PSM hooks (arm on
 * sleep entry at btmtk_btif_main.c:1435, mask on wakeup at :1322) cannot
 * unbalance the core's enable depth.
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
		disable_irq_nosync(irq);
	*active = enable;
	spin_unlock_irqrestore(&adev->irq_lock, flags);
}

/**
 * btmtk_a32_irq_ack() - acknowledge/clear a fired IRQ (mainline analogue).
 * @adev: driver context.
 * @irq: the fired IRQ number.
 *
 * Downstream ack/clear is CONNSYS-register based and has NO mainline
 * analogue yet: bt_bgf2ap_irq_handler() reads BGF_SW_IRQ_STATUS and
 * clears bits with SET_BIT(BGF_SW_IRQ_RESET_ADDR, ...) (btmtk_irq.c:151-
 * 169); the BTIF-wakeup leg needs no CR clear because FW pushes its data
 * over BTIF once the host clears FW-own (btmtk_btif_main.c:1356-1360).
 * Those control registers live behind conninfra (TODO-CONNINFRA), so the
 * mainline ack is mask-and-defer: disable_irq_nosync() FIRST -- the same
 * call downstream makes first in its handler (btmtk_irq.c:213,220) --
 * record the indication, clear the sleep flag via the PSM helper, and let
 * the IRQ thread re-arm once the drain completes.
 */
static void btmtk_a32_irq_ack(struct btmtk_a32_dev *adev, int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&adev->irq_lock, flags);
	if (irq == adev->wake_irq && adev->wake_irq >= 0) {
		if (adev->wake_active) {
			disable_irq_nosync(adev->wake_irq);
			adev->wake_active = false;
		}
		adev->rx_pending = true;
		adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
						    BTMTK_A32_EV_WAKE_IRQ);
	} else if (irq == adev->sw_irq && adev->sw_irq >= 0) {
		if (adev->sw_active) {
			disable_irq_nosync(adev->sw_irq);
			adev->sw_active = false;
		}
	}
	spin_unlock_irqrestore(&adev->irq_lock, flags);
}

static irqreturn_t btmtk_a32_irq_handler(int irq, void *arg)
{
	struct btmtk_a32_dev *adev = arg;

	/* Unknown line: IRQ_NONE, exactly as downstream (btmtk_irq.c:225). */
	if (irq != adev->wake_irq && irq != adev->sw_irq)
		return IRQ_NONE;
	btmtk_a32_irq_ack(adev, irq);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t btmtk_a32_irq_thread(int irq, void *arg)
{
	struct btmtk_a32_dev *adev = arg;

	if (adev->ws)
		__pm_wakeup_event(adev->ws, BTMTK_A32_WAKE_HOLD_MS);

	if (irq == adev->wake_irq) {
		/* FW has data (btmtk_irq.c:198). The offline stub has no RX
		 * source, so the drain is: clear the indication, run pending
		 * TX work, re-arm (the re-arm is the enable_irq() the
		 * downstream thread path performs at btmtk_btif_main.c:1435
		 * on sleep entry / after the wakeup completes).
		 */
		adev->rx_pending = false;
		schedule_work(&adev->tx_work);
		btmtk_a32_irq_set(adev, true, true);
	} else {
		/* SW IRQ: FW assert / FW-log notify (btmtk_irq.c:199). Reading
		 * the status CR and driving reset/log handling needs conninfra
		 * register access (TODO-CONNINFRA); offline we re-arm and note
		 * it so no wakeup is ever lost-silent.
		 */
		dev_warn_ratelimited(adev->dev,
				     "SW IRQ fired; status-CR handling deferred (TODO-CONNINFRA)\n");
		btmtk_a32_irq_set(adev, false, true);
	}
	return IRQ_HANDLED;
}

/**
 * btmtk_a32_set_sleep() - arm FW-wakeup and enter the SLEEP state.
 * @adev: driver context.
 *
 * Host-side analogue of the downstream NORMAL_TR sleep leg
 * (btmtk_btif_main.c:1420-1438: FW-own set, enable BTIF_WAKEUP_IRQ,
 * -> SLEEP). The FW-own set itself needs the BTIF backend (TODO-BTIF);
 * what this hook honors offline is the transition plus the IRQ arming.
 */
static int btmtk_a32_set_sleep(struct btmtk_a32_dev *adev)
{
	adev->psm_state = btmtk_a32_psm_next(adev->psm_state,
					    BTMTK_A32_EV_SLEEP_REQ);
	btmtk_a32_irq_set(adev, true, true);
	return 0;
}

/**
 * btmtk_a32_set_wakeup() - force awake (mask FW-wakeup, hold a wake event).
 * @adev: driver context.
 *
 * Host-side analogue of btmtk_set_wakeup() (btmtk_mt66xx.c:1891) and the
 * SLEEP wakeup leg (btmtk_btif_main.c:1320-1346: disable IRQ, FW-own
 * clear, -> NORMAL_TR). The FW-own clear needs the BTIF backend
 * (TODO-BTIF); offline this honors the transition, the IRQ mask (cf.
 * btmtk_btif_main.c:1322) and a bounded wake hold standing in for the
 * downstream "bt_psm" lock hold across the wakeup window.
 */
static int btmtk_a32_set_wakeup(struct btmtk_a32_dev *adev)
{
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
	skb_queue_head_init(&adev->txq);

	/* Default transport: offline stub behind the BTIF-shaped contract
	 * (TODO-BTIF swaps in the real conninfra backend).
	 */
	adev->transport.send = btmtk_a32_stub_send;
	adev->transport.open = btmtk_a32_stub_open;
	adev->transport.close = btmtk_a32_stub_close;
	adev->transport.ctx = adev;

	btmtk_a32_power_init(adev);

	/* §3: DT-by-name IRQ resolution. Optional until TODO-HWIRQ closes;
	 * absence only logs, never fails probe (offline-safe).
	 */
	btmtk_a32_irq_request(adev);

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

static void btmtk_a32_remove(struct platform_device *pdev)
{
	struct btmtk_a32_dev *adev = platform_get_drvdata(pdev);

	cancel_work_sync(&adev->tx_work);
	hci_unregister_dev(adev->hdev);
	hci_free_dev(adev->hdev);
	btmtk_a32_irq_free(adev);
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
