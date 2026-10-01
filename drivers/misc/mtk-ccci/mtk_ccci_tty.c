// SPDX-License-Identifier: GPL-2.0-only
/*
 * CCCI TTY registration: real ttyC* devices via the TTY layer.
 *
 * Downstream port minors (port_cfg.c, preserved verbatim):
 *   ttyC0 m3, ttyC2 m5, ttyC3 m6, ttyC1 m7, ttyC5 m31, ttyC6 m32,
 *   ttyC_AT m44.
 * Userspace must NEVER create these nodes itself; they are registered
 * here with tty_register_driver()/tty_port_register_device().
 */

#include <linux/console.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include "mtk_ccci.h"

#define MTK_CCCI_TTY_NAME	"ttyC"
#define MTK_CCCI_TTY_NUM	64	/* indices 0..63; used: 3,5,6,7,31,32,44 */

struct mtk_ccci_tty {
	struct tty_port port;
	unsigned int index;
};

static struct tty_driver *mtk_ccci_tty_drv;
static struct mtk_ccci_tty mtk_ccci_ttys[MTK_CCCI_TTY_NUM];

/* TTY indices that back real downstream ports (minor == index).
 * The valid set lives in mtk_ccci.h (single source of truth, shared
 * with the offline unit tests); index 44 is the "ttyC_AT" AT channel.
 */
static const unsigned int mtk_ccci_tty_indices[MTK_CCCI_TTY_COUNT] = {
	3, 5, 6, 7, 31, 32, 44,
};

static int mtk_ccci_tty_open(struct tty_struct *tty, struct file *filp)
{
	if (!mtk_ccci_tty_index_valid(tty->index))
		return -ENODEV;
	return tty_port_open(&mtk_ccci_ttys[tty->index].port, tty, filp);
}

static void mtk_ccci_tty_close(struct tty_struct *tty, struct file *filp)
{
	if (!mtk_ccci_tty_index_valid(tty->index))
		return;
	tty_port_close(&mtk_ccci_ttys[tty->index].port, tty, filp);
}

static ssize_t mtk_ccci_tty_write(struct tty_struct *tty, const u8 *buf,
				  size_t count)
{
	/* Transport path gates on modem READY (a HARDWARE gate without
	 * firmware). Report the bytes as unwritable-for-now rather than
	 * silently dropping them.
	 */
	return -EIO;
}

static unsigned int mtk_ccci_tty_write_room(struct tty_struct *tty)
{
	return 0;
}

static const struct tty_operations mtk_ccci_tty_ops = {
	.open = mtk_ccci_tty_open,
	.close = mtk_ccci_tty_close,
	.write = mtk_ccci_tty_write,
	.write_room = mtk_ccci_tty_write_room,
};

int mtk_ccci_tty_register(void)
{
	unsigned int i;
	int ret;

	mtk_ccci_tty_drv = tty_alloc_driver(MTK_CCCI_TTY_NUM,
					    TTY_DRIVER_REAL_RAW |
					    TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(mtk_ccci_tty_drv))
		return PTR_ERR(mtk_ccci_tty_drv);

	mtk_ccci_tty_drv->driver_name = "mtk_ccci";
	mtk_ccci_tty_drv->name = MTK_CCCI_TTY_NAME;
	mtk_ccci_tty_drv->major = 0;
	mtk_ccci_tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	mtk_ccci_tty_drv->subtype = SERIAL_TYPE_NORMAL;
	mtk_ccci_tty_drv->init_termios = tty_std_termios;
	mtk_ccci_tty_drv->init_termios.c_cflag = B115200 | CS8 | CREAD;
	tty_set_operations(mtk_ccci_tty_drv, &mtk_ccci_tty_ops);

	ret = tty_register_driver(mtk_ccci_tty_drv);
	if (ret) {
		tty_driver_kref_put(mtk_ccci_tty_drv);
		mtk_ccci_tty_drv = NULL;
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(mtk_ccci_tty_indices); i++) {
		unsigned int idx = mtk_ccci_tty_indices[i];
		struct mtk_ccci_tty *ct = &mtk_ccci_ttys[idx];

		/* Registration table must stay consistent with the
		 * header's valid-index set (single source of truth).
		 */
		if (idx >= MTK_CCCI_TTY_NUM ||
		    !mtk_ccci_tty_index_valid(idx)) {
			ret = -EINVAL;
			goto err_ports;
		}
		ct->index = idx;
		tty_port_init(&ct->port);
		/* No port callbacks: the modem transport gates on READY
		 * (HARDWARE gate); default helpers suffice offline.
		 */
		ct->port.ops = NULL;
		tty_port_register_device(&ct->port, mtk_ccci_tty_drv, idx,
					 NULL);
	}
	return 0;

err_ports:
	while (i-- > 0) {
		tty_unregister_device(mtk_ccci_tty_drv,
				      mtk_ccci_tty_indices[i]);
		tty_port_destroy(&mtk_ccci_ttys[mtk_ccci_tty_indices[i]].port);
	}
	tty_unregister_driver(mtk_ccci_tty_drv);
	tty_driver_kref_put(mtk_ccci_tty_drv);
	mtk_ccci_tty_drv = NULL;
	return ret;
}

void mtk_ccci_tty_unregister(void)
{
	unsigned int i;

	if (!mtk_ccci_tty_drv)
		return;
	for (i = 0; i < ARRAY_SIZE(mtk_ccci_tty_indices); i++) {
		unsigned int idx = mtk_ccci_tty_indices[i];

		tty_unregister_device(mtk_ccci_tty_drv, idx);
		tty_port_destroy(&mtk_ccci_ttys[idx].port);
	}
	tty_unregister_driver(mtk_ccci_tty_drv);
	tty_driver_kref_put(mtk_ccci_tty_drv);
	mtk_ccci_tty_drv = NULL;
}
