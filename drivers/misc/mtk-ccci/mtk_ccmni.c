// SPDX-License-Identifier: GPL-2.0-only
/*
 * ccmni netdev: RAW-IP data path (NOT QMI/MBIM).
 *
 * Evidence (downstream, read-only):
 * - ccmni.c:1262  dev->type = ARPHRD_RAWIP
 * - ccmni.c:1240-1282 register_netdev() loop over ccmni_num instances
 * - ccmni.h:45     CCMNI_MTU 1500
 * - ccmni.h:53-61  SIOCDEVPRIVATE+0..4 ioctls
 * - port_net.c     net_port_ops class split (char/rpc/ipc/sys/ctl/poller
 *                  vs net_port_ops) preserved in mtk_ccci_port.c
 *
 * Modem data path: userspace opens /dev/ttyC_AT and configures ccmni0
 * (mtk-pdpd path, already documented in MODEM-DATA-PLAN.md). This file
 * registers the RAW-IP netdevs; TX frames route into the CLDMA data
 * queues once the modem is READY (HARDWARE gate without firmware).
 */

#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#include "mtk_ccci.h"

#define MTK_CCMNI_TX_QUEUE_LEN	1000
#define MTK_CCMNI_WDT_TIMEOUT	(HZ)

struct mtk_ccmni_priv {
	unsigned int md_id;
	int index;
	struct mtk_ccmni_ch ch;
	spinlock_t lock;
};

static struct net_device *mtk_ccmni_devs[MTK_CCCI_MAX_MD_NUM][22];

static int mtk_ccmni_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int mtk_ccmni_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

/* RAW-IP framing rule (shared with the offline unit tests in
 * scripts/tests/ccci/): the first nibble must be 4 (IPv4) or 6 (IPv6);
 * anything else is not an IP packet and is dropped + counted.
 */

static netdev_tx_t mtk_ccmni_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	if (!mtk_ccci_is_raw_ip_frame(skb->data, skb->len)) {
		/* Not a RAW-IP frame: drop and count (never forward). */
		dev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	/* Transport queued once modem READY; without firmware the frame
	 * is counted and dropped (HARDWARE gate, no silent loss on a
	 * live modem because the queue only runs when READY).
	 */
	dev->stats.tx_dropped++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void mtk_ccmni_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	dev->stats.tx_errors++;
	netif_wake_queue(dev);
}

static int mtk_ccmni_change_mtu(struct net_device *dev, int new_mtu)
{
	if (!mtk_ccmni_mtu_valid(new_mtu))
		return -EINVAL;
	dev->mtu = (unsigned int)new_mtu;
	return 0;
}

static int mtk_ccmni_do_ioctl(struct net_device *dev, struct ifreq *ifr,
			      int cmd)
{
	switch (cmd) {
	case MTK_CCMNI_SIOCSTXQSTATE:
	case MTK_CCMNI_SIOCCCMNICFG:
	case MTK_CCMNI_SIOCFWDFILTER:
	case MTK_CCMNI_SIOCACKPRIO:
	case MTK_CCMNI_SIOPUSHPENDING:
		/* Handled: no private state to mutate offline; accept so
		 * userspace configuration flows (pdpd) do not error out.
		 */
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops mtk_ccmni_ops = {
	.ndo_open = mtk_ccmni_open,
	.ndo_stop = mtk_ccmni_stop,
	.ndo_start_xmit = mtk_ccmni_start_xmit,
	.ndo_tx_timeout = mtk_ccmni_tx_timeout,
	.ndo_change_mtu = mtk_ccmni_change_mtu,
	.ndo_do_ioctl = mtk_ccmni_do_ioctl,
};

static void mtk_ccmni_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->header_ops = NULL;
	dev->netdev_ops = &mtk_ccmni_ops;
	dev->type = ARPHRD_RAWIP;
	dev->mtu = MTK_CCMNI_MTU;
	dev->tx_queue_len = MTK_CCMNI_TX_QUEUE_LEN;
	dev->watchdog_timeo = MTK_CCMNI_WDT_TIMEOUT;
	/* Pure IP device (downstream ccmni_dev_init): NOARP, and clear
	 * the BROADCAST/MULTICAST bits ether_setup() set.
	 */
	dev->flags = IFF_NOARP & (~IFF_BROADCAST & ~IFF_MULTICAST);
	dev->features = NETIF_F_VLAN_CHALLENGED;
}

int mtk_ccmni_register_all(unsigned int md_id)
{
	int idx, ret = 0;

	if (md_id >= MTK_CCCI_MAX_MD_NUM)
		return -EINVAL;

	/* ccmni0-7 (idx0-7), ccmni9-20 (idx9-20, idx8 gap), ccmni-lan
	 * (idx21): names track the ccmni index, exactly like downstream.
	 */
	for (idx = 0; idx <= MTK_CCMNI_LAN_INDEX; idx++) {
		struct net_device *dev;
		struct mtk_ccmni_priv *priv;
		char name[IFNAMSIZ];

		if (idx == MTK_CCMNI_INVALID_INDEX)
			continue;
		if (mtk_ccmni_ifname(idx, name, sizeof(name)))
			continue;

		dev = alloc_netdev(sizeof(*priv), name, NET_NAME_UNKNOWN,
				   mtk_ccmni_setup);
		if (!dev) {
			ret = -ENOMEM;
			goto err;
		}
		priv = netdev_priv(dev);
		priv->md_id = md_id;
		priv->index = idx;
		spin_lock_init(&priv->lock);
		ret = mtk_ccmni_get_channel(md_id, idx, &priv->ch);
		if (ret) {
			free_netdev(dev);
			goto err;
		}
		ret = register_netdev(dev);
		if (ret) {
			free_netdev(dev);
			goto err;
		}
		mtk_ccmni_devs[md_id][idx] = dev;
	}
	return 0;

err:
	mtk_ccmni_unregister_all(md_id);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccmni_register_all);

void mtk_ccmni_unregister_all(unsigned int md_id)
{
	int idx;

	if (md_id >= MTK_CCCI_MAX_MD_NUM)
		return;
	for (idx = 0; idx <= MTK_CCMNI_LAN_INDEX; idx++) {
		if (!mtk_ccmni_devs[md_id][idx])
			continue;
		unregister_netdev(mtk_ccmni_devs[md_id][idx]);
		free_netdev(mtk_ccmni_devs[md_id][idx]);
		mtk_ccmni_devs[md_id][idx] = NULL;
	}
}
EXPORT_SYMBOL_GPL(mtk_ccmni_unregister_all);
