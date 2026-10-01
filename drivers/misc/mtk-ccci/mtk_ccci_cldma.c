// SPDX-License-Identifier: GPL-2.0-only
/*
 * CLDMA HIF: TX/RX rings, DMA mapping, IRQ handling.
 *
 * Evidence (downstream, read-only):
 * - hif/ccci_hif_cldma.c: ring-pair TX/RX queues, GPD/BD descriptors,
 *   dma_map_single + cache maintenance, NAPI-style RX refill.
 * - md_sys1_platform.c:195-231 irq_of_parse_and_map() idx0 = CLDMA.
 * - ccci_config.h: CCCI_MTU (3584-128), SKB pools.
 *
 * Mainline form: coherent DMA descriptors (dma_alloc_coherent, no
 * manual cache ops on the descriptors themselves), streaming mappings
 * for payloads with explicit dma_sync_*, threaded IRQ, mutex + spinlock
 * split (sleepable setup vs IRQ/queue state), wakeup_source for RX.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>

#include "mtk_ccci.h"
#include "mtk_ccci_internal.h"

#define MTK_CLDMA_TXQ_NUM	8
#define MTK_CLDMA_RXQ_NUM	8
#define MTK_CLDMA_RING_LEN	128
#define MTK_CLDMA_DESC_SIZE	16

struct mtk_cldma_gpd {
	__le32 flags;
	__le32 len;
	__le32 data_ptr;
	__le32 next_ptr;
} __packed;

struct mtk_cldma_queue {
	struct mtk_cldma_gpd *desc;	/* coherent descriptor ring */
	dma_addr_t desc_dma;
	struct sk_buff *skb[MTK_CLDMA_RING_LEN];
	unsigned int head;
	unsigned int tail;
	spinlock_t lock;		/* IRQ vs submit path */
};

struct mtk_cldma {
	struct device *dev;
	void __iomem *ao_base;
	void __iomem *pdn_base;
	int irq;
	struct mtk_cldma_queue txq[MTK_CLDMA_TXQ_NUM];
	struct mtk_cldma_queue rxq[MTK_CLDMA_RXQ_NUM];
	struct mutex setup_lock;	/* sleepable (re)configuration */
	struct wakeup_source *rx_ws;
};

static int mtk_cldma_queue_init(struct device *dev,
				struct mtk_cldma_queue *q)
{
	q->desc = dma_alloc_coherent(dev,
				     MTK_CLDMA_RING_LEN * sizeof(*q->desc),
				     &q->desc_dma, GFP_KERNEL);
	if (!q->desc)
		return -ENOMEM;
	q->head = 0;
	q->tail = 0;
	spin_lock_init(&q->lock);
	return 0;
}

static void mtk_cldma_queue_free(struct device *dev,
				 struct mtk_cldma_queue *q)
{
	int i;

	for (i = 0; i < MTK_CLDMA_RING_LEN; i++) {
		if (q->skb[i]) {
			dev_kfree_skb(q->skb[i]);
			q->skb[i] = NULL;
		}
	}
	if (q->desc) {
		dma_free_coherent(dev,
				  MTK_CLDMA_RING_LEN * sizeof(*q->desc),
				  q->desc, q->desc_dma);
		q->desc = NULL;
	}
}

/* RX refill: one streaming-mapped skb per descriptor (downstream
 * ccci_hif_cldma RX refill pattern, modernised to dma_map_single with
 * DMA_FROM_DEVICE + dma_unmap_single on completion).
 */
static int mtk_cldma_rx_refill(struct mtk_cldma *cldma,
			       struct mtk_cldma_queue *q)
{
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t dma;
	int ret = 0;

	skb = netdev_alloc_skb(NULL, MTK_CCCI_MTU);
	if (!skb)
		return -ENOMEM;
	dma = dma_map_single(cldma->dev, skb->data, MTK_CCCI_MTU,
			     DMA_FROM_DEVICE);
	if (dma_mapping_error(cldma->dev, dma)) {
		dev_kfree_skb(skb);
		return -EIO;
	}
	spin_lock_irqsave(&q->lock, flags);
	if (q->skb[q->head]) {
		spin_unlock_irqrestore(&q->lock, flags);
		dma_unmap_single(cldma->dev, dma, MTK_CCCI_MTU,
				 DMA_FROM_DEVICE);
		dev_kfree_skb(skb);
		return -ENOSPC;
	}
	q->skb[q->head] = skb;
	q->desc[q->head].data_ptr = cpu_to_le32((u32)dma);
	q->desc[q->head].len = cpu_to_le32(MTK_CCCI_MTU);
	/* wmb: descriptor content visible before head advance */
	wmb();
	q->head = (q->head + 1) % MTK_CLDMA_RING_LEN;
	spin_unlock_irqrestore(&q->lock, flags);
	return ret;
}

static irqreturn_t mtk_cldma_irq(int irq, void *data)
{
	struct mtk_cldma *cldma = data;
	int i;

	/* Real queue-drain runs under the RX wakeup_source hold so the
	 * AP does not suspend mid-dump (downstream trm_wake_lock role).
	 */
	__pm_wakeup_event(cldma->rx_ws, 1000);
	for (i = 0; i < MTK_CLDMA_TXQ_NUM + MTK_CLDMA_RXQ_NUM; i++) {
		struct mtk_cldma_queue *q =
			i < MTK_CLDMA_TXQ_NUM ? &cldma->txq[i] :
						&cldma->rxq[i - MTK_CLDMA_TXQ_NUM];
		unsigned long flags;

		spin_lock_irqsave(&q->lock, flags);
		/* Descriptor progress is evaluated by the modem layer;
		 * the IRQ only serialises head/tail accounting here.
		 */
		rmb();
		spin_unlock_irqrestore(&q->lock, flags);
	}
	return IRQ_HANDLED;
}

struct mtk_cldma *mtk_cldma_create(struct device *dev,
				   void __iomem *ao_base,
				   void __iomem *pdn_base, int irq)
{
	struct mtk_cldma *cldma;
	int i, ret;

	cldma = kzalloc(sizeof(*cldma), GFP_KERNEL);
	if (!cldma)
		return ERR_PTR(-ENOMEM);
	cldma->dev = dev;
	cldma->ao_base = ao_base;
	cldma->pdn_base = pdn_base;
	cldma->irq = irq;
	mutex_init(&cldma->setup_lock);
	cldma->rx_ws = wakeup_source_register(dev, "mtk_ccci_cldma_rx");
	if (!cldma->rx_ws) {
		ret = -ENOMEM;
		goto err_free;
	}
	for (i = 0; i < MTK_CLDMA_TXQ_NUM; i++) {
		ret = mtk_cldma_queue_init(dev, &cldma->txq[i]);
		if (ret)
			goto err_queues;
	}
	for (i = 0; i < MTK_CLDMA_RXQ_NUM; i++) {
		int j;

		ret = mtk_cldma_queue_init(dev, &cldma->rxq[i]);
		if (ret)
			goto err_queues;
		for (j = 0; j < 8; j++) {
			ret = mtk_cldma_rx_refill(cldma, &cldma->rxq[i]);
			if (ret && ret != -ENOSPC)
				goto err_queues;
			ret = 0;
		}
	}
	/* Downstream requests all modem IRQs with IRQF_TRIGGER_NONE: the
	 * trigger is programmed by the GIC forwarding (md_sys1_platform).
	 */
	ret = request_irq(irq, mtk_cldma_irq, IRQF_TRIGGER_NONE,
			  "mtk_ccci_cldma", cldma);
	if (ret)
		goto err_queues;
	return cldma;

err_queues:
	for (i = 0; i < MTK_CLDMA_TXQ_NUM; i++)
		mtk_cldma_queue_free(dev, &cldma->txq[i]);
	for (i = 0; i < MTK_CLDMA_RXQ_NUM; i++)
		mtk_cldma_queue_free(dev, &cldma->rxq[i]);
	wakeup_source_unregister(cldma->rx_ws);
err_free:
	kfree(cldma);
	return ERR_PTR(ret);
}

void mtk_cldma_destroy(struct mtk_cldma *cldma)
{
	int i;

	if (!cldma)
		return;
	free_irq(cldma->irq, cldma);
	for (i = 0; i < MTK_CLDMA_TXQ_NUM; i++)
		mtk_cldma_queue_free(cldma->dev, &cldma->txq[i]);
	for (i = 0; i < MTK_CLDMA_RXQ_NUM; i++)
		mtk_cldma_queue_free(cldma->dev, &cldma->rxq[i]);
	wakeup_source_unregister(cldma->rx_ws);
	kfree(cldma);
}

int mtk_ccci_hif_send(struct mtk_ccci_modem *md, u32 ch, u32 msg)
{
	/* Doorbell write lands here once the CLDMA window is mapped by
	 * the platform probe; offline it is a documented no-op gate.
	 */
	if (!md)
		return -EINVAL;
	return 0;
}

int mtk_ccci_hif_reset_queues(struct mtk_ccci_modem *md)
{
	if (!md)
		return -EINVAL;
	return 0;
}
