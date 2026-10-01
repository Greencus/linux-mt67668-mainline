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
};

static int btmtk_a32_stub_send(struct device *dev, void *ctx,
			       const u8 *data, unsigned int len);
static int btmtk_a32_stub_open(struct device *dev, void *ctx);
static void btmtk_a32_stub_close(struct device *dev, void *ctx);

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
}

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
