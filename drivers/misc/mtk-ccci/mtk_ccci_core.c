// SPDX-License-Identifier: GPL-2.0-only
/*
 * CCCI core: class + sysfs + boot attribute.
 *
 * Evidence (downstream, read-only):
 *   kernel/downstream/drivers/misc/mediatek/eccci/ccci_core.c:53-72
 *     boot_md_show()/boot_md_store() formatting ("md%d:%d", -EACCES store)
 *   ccci_core.c:120-124  class_create("ccci_node")
 *   ccci_core.c:157      /sys/kernel/ccci/boot registration path
 *
 * Mainline form: a "ccci_node" class plus a /sys/kernel/ccci/boot
 * attribute owned by a kobject. boot show prints "md1:<state>" for each
 * enabled modem; store rejects with -EACCES exactly like downstream.
 */

#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include "mtk_ccci.h"

static struct class *mtk_ccci_class;
static struct kobject *mtk_ccci_kobj;

static bool mtk_ccci_modem_enabled[MTK_CCCI_MAX_MD_NUM] = {
	[MTK_CCCI_MD_SYS1] = true,
};

bool mtk_ccci_md_is_enabled(unsigned int md_id)
{
	if (md_id >= MTK_CCCI_MAX_MD_NUM)
		return false;
	return mtk_ccci_modem_enabled[md_id];
}
EXPORT_SYMBOL_GPL(mtk_ccci_md_is_enabled);

int mtk_ccci_get_md_state(unsigned int md_id)
{
	if (md_id >= MTK_CCCI_MAX_MD_NUM)
		return MTK_CCCI_MD_STATE_INVALID;
	/* No firmware is loaded in an offline tree: report the pre-HS1
	 * state. The READY transition only happens after a real HS1/HS2
	 * exchange with modem hardware (a HARDWARE gate, see docs).
	 */
	return MTK_CCCI_MD_STATE_BOOT_WAITING_FOR_HS1;
}
EXPORT_SYMBOL_GPL(mtk_ccci_get_md_state);

static ssize_t boot_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int curr = 0;
	unsigned int md_id;

	for (md_id = 0; md_id < MTK_CCCI_MAX_MD_NUM; md_id++) {
		if (mtk_ccci_md_is_enabled(md_id)) {
			curr += scnprintf(buf + curr, PAGE_SIZE - (size_t)curr,
					  "md%u:%d", md_id + 1,
					  mtk_ccci_get_md_state(md_id));
		}
	}
	curr += scnprintf(buf + curr, PAGE_SIZE - (size_t)curr, "\n");
	return curr;
}

static ssize_t boot_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t count)
{
	/* Downstream boot_md_store() returns -EACCES unconditionally;
	 * the boot attribute is read-only by design.
	 */
	return -EACCES;
}

static struct kobj_attribute mtk_ccci_boot_attr = __ATTR_RW(boot);

int mtk_ccci_core_init(void)
{
	int ret;

	mtk_ccci_class = class_create("ccci_node");
	if (IS_ERR(mtk_ccci_class))
		return PTR_ERR(mtk_ccci_class);

	mtk_ccci_kobj = kobject_create_and_add("ccci", kernel_kobj);
	if (!mtk_ccci_kobj) {
		ret = -ENOMEM;
		goto err_class;
	}

	ret = sysfs_create_file(mtk_ccci_kobj, &mtk_ccci_boot_attr.attr);
	if (ret)
		goto err_kobj;

	return 0;

err_kobj:
	kobject_put(mtk_ccci_kobj);
	mtk_ccci_kobj = NULL;
err_class:
	class_destroy(mtk_ccci_class);
	mtk_ccci_class = NULL;
	return ret;
}

void mtk_ccci_core_exit(void)
{
	if (mtk_ccci_kobj) {
		sysfs_remove_file(mtk_ccci_kobj, &mtk_ccci_boot_attr.attr);
		kobject_put(mtk_ccci_kobj);
		mtk_ccci_kobj = NULL;
	}
	if (mtk_ccci_class) {
		class_destroy(mtk_ccci_class);
		mtk_ccci_class = NULL;
	}
}
