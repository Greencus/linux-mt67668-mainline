// SPDX-License-Identifier: GPL-2.0-only
/*
 * Internal (non-UAPI) structures shared between the mtk-ccci objects.
 * Not installed, not part of the kernel ABI.
 */

#ifndef _MTK_CCCI_INTERNAL_H_
#define _MTK_CCCI_INTERNAL_H_

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "mtk_ccci.h"

struct mtk_ccci_modem {
	struct list_head entry;
	struct device *dev;
	unsigned int md_id;
	enum mtk_ccci_md_state state;
	struct wakeup_source *trk_ws;
	struct mutex state_lock;
	void __iomem *boot_stats0;
	void __iomem *boot_stats1;
	void __iomem *boot_vector_en;
};

struct mtk_ccci_modem *mtk_ccci_modem_alloc(struct device *dev,
					    unsigned int md_id);
void mtk_ccci_modem_free(struct mtk_ccci_modem *md);
void mtk_ccci_modem_set_state(struct mtk_ccci_modem *md,
			      enum mtk_ccci_md_state state);

/* HIF glue (implemented per transport) */
int mtk_ccci_hif_send(struct mtk_ccci_modem *md, u32 ch, u32 msg);
int mtk_ccci_hif_reset_queues(struct mtk_ccci_modem *md);

/* CLDMA transport instance (implemented in mtk_ccci_cldma.c) */
struct mtk_cldma;
struct mtk_cldma *mtk_cldma_create(struct device *dev,
				   void __iomem *ao_base,
				   void __iomem *pdn_base, int irq);
void mtk_cldma_destroy(struct mtk_cldma *cldma);

#endif /* _MTK_CCCI_INTERNAL_H_ */
