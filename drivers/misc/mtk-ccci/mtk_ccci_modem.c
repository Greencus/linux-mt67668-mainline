// SPDX-License-Identifier: GPL-2.0-only
/*
 * CCCI modem instance (MD_SYS1) + EE handshake dispatch + boot stats.
 *
 * Evidence (downstream, read-only):
 * - modem_sys1.c:249  md_cd_ee_handshake() 4-stage EE sequence
 * - modem_sys3.c:88   md_ccif_ee_handshake() CCIF variant
 * - ccci_modem.c:1568 ccci_md_exception_handshake() dispatch via md->ops
 * - ccci_fsm.c:261    EE_CTRL waits MD_EX / MD_EX_REC_OK / MD_EX_PASS
 * - ccci_fsm.c:389-464 BOOT_WAITING_FOR_HS1 -> runtime data -> HS2 -> READY
 * - modem_reg_base.h:21 MD_BOOT_VECTOR_EN 0x20000024
 * - md_sys1_platform.c:451-453,501-537 boot-stats dump/get helpers
 *
 * HS-STAGE model ported here: no BOOT_READY / BOOT_UP tokens exist in
 * either tree; the handshake is (HS1 -> send runtime data -> HS2),
 * with the EE path running (INIT -> INIT_DONE -> CLEARQ_DONE ->
 * ALLQ_RESET). All modem-side register access is through ioremap'd
 * windows described by DT; without firmware the FSM stays pre-HS1.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>

#include "mtk_ccci.h"
#include "mtk_ccci_internal.h"

/* MD1 boot-vector / boot-stats register offsets (modem_reg_base.h) */
#define MTK_CCCI_MD_BOOT_VECTOR_EN	0x20000024
#define MTK_CCCI_MD1_CFG_BASE		0x1020E300
#define MTK_CCCI_MD1_BOOT_STATS0	(MTK_CCCI_MD1_CFG_BASE + 0x00)
#define MTK_CCCI_MD1_BOOT_STATS1	(MTK_CCCI_MD1_CFG_BASE + 0x04)
#define MTK_CCCI_EE_HANDSHAKE_TIMEOUT_MS 20000

static LIST_HEAD(mtk_ccci_modem_list);
static DEFINE_MUTEX(mtk_ccci_modem_list_lock);

struct mtk_ccci_modem *mtk_ccci_modem_alloc(struct device *dev,
					    unsigned int md_id)
{
	struct mtk_ccci_modem *md;

	if (!dev || md_id >= MTK_CCCI_MAX_MD_NUM)
		return NULL;
	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (!md)
		return NULL;
	INIT_LIST_HEAD(&md->entry);
	md->dev = dev;
	md->md_id = md_id;
	md->state = MTK_CCCI_MD_STATE_BOOT_WAITING_FOR_HS1;
	mutex_init(&md->state_lock);
	md->trk_ws = wakeup_source_register(dev, "mtk_ccci_trm");
	if (!md->trk_ws) {
		kfree(md);
		return NULL;
	}
	return md;
}

void mtk_ccci_modem_free(struct mtk_ccci_modem *md)
{
	if (!md)
		return;
	wakeup_source_unregister(md->trk_ws);
	if (md->boot_stats0)
		iounmap(md->boot_stats0);
	if (md->boot_stats1)
		iounmap(md->boot_stats1);
	if (md->boot_vector_en)
		iounmap(md->boot_vector_en);
	kfree(md);
}

void mtk_ccci_modem_set_state(struct mtk_ccci_modem *md,
			      enum mtk_ccci_md_state state)
{
	if (!md)
		return;
	mutex_lock(&md->state_lock);
	md->state = state;
	mutex_unlock(&md->state_lock);
}

static struct mtk_ccci_modem *mtk_ccci_find_modem(unsigned int md_id)
{
	struct mtk_ccci_modem *md;

	lockdep_assert_held(&mtk_ccci_modem_list_lock);
	list_for_each_entry(md, &mtk_ccci_modem_list, entry) {
		if (md->md_id == md_id)
			return md;
	}
	return NULL;
}

int mtk_ccci_modem_register(struct mtk_ccci_modem *md)
{
	if (!md || md->md_id >= MTK_CCCI_MAX_MD_NUM)
		return -EINVAL;

	mutex_lock(&mtk_ccci_modem_list_lock);
	if (mtk_ccci_find_modem(md->md_id)) {
		mutex_unlock(&mtk_ccci_modem_list_lock);
		return -EEXIST;
	}
	list_add_tail(&md->entry, &mtk_ccci_modem_list);
	mutex_unlock(&mtk_ccci_modem_list_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_ccci_modem_register);

void mtk_ccci_modem_unregister(struct mtk_ccci_modem *md)
{
	if (!md)
		return;
	mutex_lock(&mtk_ccci_modem_list_lock);
	list_del(&md->entry);
	mutex_unlock(&mtk_ccci_modem_list_lock);
}
EXPORT_SYMBOL_GPL(mtk_ccci_modem_unregister);

/* One EE handshake stage: maps 1:1 onto the downstream HIF_EX_* sequence.
 * Each stage is acked through the HIF (CLDMA doorbell or CCIF channel)
 * by mtk_ccci_modem_exception(); errors funnel into the EXCEPTION state.
 */
static int mtk_ccci_ee_stage(struct mtk_ccci_modem *md,
			     enum mtk_ccci_hif_ex_stage stage)
{
	int ret;

	ret = mtk_ccci_modem_exception(md, stage);
	if (ret) {
		mutex_lock(&md->state_lock);
		md->state = MTK_CCCI_MD_STATE_EXCEPTION;
		mutex_unlock(&md->state_lock);
	}
	return ret;
}

int mtk_ccci_modem_ee_handshake(struct mtk_ccci_modem *md, int timeout_ms)
{
	int ret;

	if (!md)
		return -EINVAL;
	if (timeout_ms <= 0)
		timeout_ms = MTK_CCCI_EE_HANDSHAKE_TIMEOUT_MS;

	/* Downstream polls INIT_DONE / CLEARQ_DONE / ALLQ_RESET between
	 * stages (modem_sys1.c polling_ready). Here each stage completes
	 * synchronously through the HIF; the wakeup_source hold mirrors
	 * the downstream __pm_wakeup_event(20*HZ) guards.
	 */
	__pm_stay_awake(md->trk_ws);

	ret = mtk_ccci_ee_stage(md, MTK_CCCI_HIF_EX_INIT);
	if (ret)
		goto out;
	ret = mtk_ccci_ee_stage(md, MTK_CCCI_HIF_EX_INIT_DONE);
	if (ret)
		goto out;
	ret = mtk_ccci_ee_stage(md, MTK_CCCI_HIF_EX_CLEARQ_DONE);
	if (ret)
		goto out;
	ret = mtk_ccci_ee_stage(md, MTK_CCCI_HIF_EX_ALLQ_RESET);
out:
	__pm_relax(md->trk_ws);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccci_modem_ee_handshake);

int mtk_ccci_modem_get_boot_stats(struct mtk_ccci_modem *md,
				  u32 *stats0, u32 *stats1)
{
	u32 s0, s1;

	if (!md || !stats0 || !stats1)
		return -EINVAL;
	if (!md->boot_stats0 || !md->boot_stats1)
		return -ENODEV;

	/* Downstream does two dummy reads to cover the AP/MD interface
	 * delay, then buys the third value (md_sys1_platform.c:501-537).
	 */
	readl(md->boot_stats0);
	readl(md->boot_stats0);
	s0 = readl(md->boot_stats0);
	readl(md->boot_stats1);
	readl(md->boot_stats1);
	s1 = readl(md->boot_stats1);
	*stats0 = s0;
	*stats1 = s1;
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_ccci_modem_get_boot_stats);

/* One HIF exception stage (downstream ccci_modem.c:1568 dispatch: the
 * modem layer fans ccci_md_exception_handshake() out to the per-HIF
 * ee_handshake/md_*_exception pokes). INIT-class stages ring the
 * control doorbell; ALLQ_RESET drains through the queue reset. Both
 * land in the HIF glue, which is an offline no-op gate until the
 * transport windows are mapped by the platform probe (HARDWARE gate).
 */
int mtk_ccci_modem_exception(struct mtk_ccci_modem *md,
			     enum mtk_ccci_hif_ex_stage stage)
{
	if (!md)
		return -EINVAL;

	switch (stage) {
	case MTK_CCCI_HIF_EX_INIT:
	case MTK_CCCI_HIF_EX_INIT_DONE:
	case MTK_CCCI_HIF_EX_CLEARQ_DONE:
		return mtk_ccci_hif_send(md, MTK_CCCI_CONTROL_TX,
					 (u32)stage);
	case MTK_CCCI_HIF_EX_ALLQ_RESET:
		return mtk_ccci_hif_reset_queues(md);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(mtk_ccci_modem_exception);
