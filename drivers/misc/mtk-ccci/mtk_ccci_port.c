// SPDX-License-Identifier: GPL-2.0-only
/*
 * CCCI port layer: MD1 port table, minor/channel lookup, ccmni channels.
 *
 * Evidence (downstream, read-only):
 * - port_cfg.c: md1 table (all rows below transcribed with identical
 *   tx/rx channels, queues, flags-class, minor and name, including the
 *   quirks: ccmni idx8 absent, ccmni-lan last at idx21, tty minors
 *   3/5/6/7/31/32/44, ccci_fs m4, ioctl0-4 m12-16, skipped ipc minor 1,
 *   IPC minor+100 rule, SMEM minor base 150).
 *   MD_GENERATION scoping (mt6768 ccci_config.h: 6293): the port_cfg.c
 *   "#if MD_GENERATION >= 6297" rows (wifi_proxy/vts/0_200/0_202/0_204
 *   and first ikeraw) and the CONFIG_MTK_SRIL_SUPPORT rows are absent;
 *   DATA_TCHE is 2 (< 6295).
 * - port_cfg.c:131-132 + ccci_fsm_monitor.c:17 "ccci_monitor must be
 *   first char port for get_port_by_minor() implement".
 * - port_net.c:43-230 ccci_get_ccmni_channel() (idx8 = INVALID replica).
 * - port_smem.c:392,742 smem minor = user_id + CCCI_SMEM_MINOR_BASE.
 * - port_ipc.c:226,439 ipc minor = task_id + CCCI_IPC_MINOR_BASE.
 * - ccci_config.h:44-48 AP/MD runtime sizes 0x800 each.
 *
 * Monitor-first rule: minor 0 of the char space is the FSM monitor
 * conduit ("ccci_monitor"); every char lookup indexes from it, so the
 * table below starts the char class with the monitor entry and the
 * downstream minors (1..44) follow unchanged.
 */

#include <linux/module.h>
#include <linux/sprintf.h>
#include <linux/string.h>

#include "mtk_ccci.h"

#define NQ	MTK_CCCI_DATA_TX_Q	/* 0: data queue selector */
#define EXP	MTK_CCCI_EXP_CTRL_Q	/* 6: expedited control queue */
#define FF	0xFF
#define HIF_N	MTK_CCCI_MD1_NORMAL_HIF
#define HIF_NET	MTK_CCCI_MD1_NET_HIF
#define HIF_CCIF MTK_CCCI_CCIF_HIF_ID
#define F_CHAR	MTK_CCCI_PORT_F_WITH_CHAR_NODE
#define F_UH	MTK_CCCI_PORT_F_USER_HEADER
#define OP_CHAR	MTK_CCCI_OPS_CHAR
#define OP_NET	MTK_CCCI_OPS_NET
#define OP_RPC	MTK_CCCI_OPS_RPC
#define OP_IPC	MTK_CCCI_OPS_IPC
#define OP_SYS	MTK_CCCI_OPS_SYS
#define OP_CTL	MTK_CCCI_OPS_CTL
#define OP_POLL	MTK_CCCI_OPS_POLLER
#define OP_SMEM	MTK_CCCI_OPS_SMEM
#define OP_MISC	MTK_CCCI_OPS_MISC
#define SQ	MTK_CCCI_SMEM_Q		/* 6: smem/CCB queue selector */

#define C_TX(ch) MTK_CCCI_##ch##_TX
#define C_RX(ch) MTK_CCCI_##ch##_RX

/* Net ports first for performance (downstream comment, port_cfg.c). */
static const struct mtk_ccci_port_cfg mtk_ccci_md1_ports[] = {
	/* ccmni0-7: idx0-7 (minor == ccmni index) */
	{ C_TX(CCMNI1), C_RX(CCMNI1), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 0, "ccmni0" },
	{ C_TX(CCMNI2), C_RX(CCMNI2), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 1, "ccmni1" },
	{ C_TX(CCMNI3), C_RX(CCMNI3), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 2, "ccmni2" },
	{ C_TX(CCMNI4), C_RX(CCMNI4), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 3, "ccmni3" },
	{ C_TX(CCMNI5), C_RX(CCMNI5), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 4, "ccmni4" },
	{ C_TX(CCMNI6), C_RX(CCMNI6), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 5, "ccmni5" },
	{ C_TX(CCMNI7), C_RX(CCMNI7), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 6, "ccmni6" },
	{ C_TX(CCMNI8), C_RX(CCMNI8), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 7, "ccmni7" },
	/* idx8 ABSENT: no CCCI_CCMNI9_* channels exist (gap, not a typo).
	 * ccmni names track the minor: CCMNI10-* channels back "ccmni9". */
	{ C_TX(CCMNI10), C_RX(CCMNI10), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 9, "ccmni9" },
	{ C_TX(CCMNI11), C_RX(CCMNI11), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 10, "ccmni10" },
	{ C_TX(CCMNI12), C_RX(CCMNI12), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 11, "ccmni11" },
	{ C_TX(CCMNI13), C_RX(CCMNI13), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 12, "ccmni12" },
	{ C_TX(CCMNI14), C_RX(CCMNI14), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 13, "ccmni13" },
	{ C_TX(CCMNI15), C_RX(CCMNI15), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 14, "ccmni14" },
	{ C_TX(CCMNI16), C_RX(CCMNI16), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 15, "ccmni15" },
	{ C_TX(CCMNI17), C_RX(CCMNI17), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 16, "ccmni16" },
	{ C_TX(CCMNI18), C_RX(CCMNI18), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 17, "ccmni17" },
	{ C_TX(CCMNI19), C_RX(CCMNI19), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 18, "ccmni18" },
	{ C_TX(CCMNI20), C_RX(CCMNI20), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 19, "ccmni19" },
	{ C_TX(CCMNI21), C_RX(CCMNI21), NQ, NQ, 0xF0 | 1, FF, HIF_NET, 0, OP_NET, 20, "ccmni20" },
	/* ccmni-lan: minor == ccmni_idx per ccci_get_ccmni_channel() */
	{ C_TX(CCMNILAN), C_RX(CCMNILAN), 0, 0, 0xF0 | 0, FF, HIF_NET, 0, OP_NET, 21, "ccmni-lan" },
	/* Char ports. ccci_monitor is FIRST (minor 0) for the
	 * get-by-minor implementation; downstream minors follow unchanged. */
	{ MTK_CCCI_INVALID_CH_ID, MTK_CCCI_INVALID_CH_ID, FF, FF, FF, FF,
	  HIF_N, F_CHAR, OP_CHAR, 0, "ccci_monitor" },
	{ C_TX(PCM), C_RX(PCM), 0, 0, FF, FF, HIF_N, F_UH | F_CHAR, OP_CHAR, 1, "ccci_aud" },
	{ C_TX(UART1), C_RX(UART1), 1, 1, EXP, EXP, HIF_N, F_CHAR, OP_CHAR, 2, "ccci_md_log_ctrl" },
	{ C_TX(UART2), C_RX(UART2), 5, 5, FF, FF, HIF_N, F_CHAR, OP_CHAR, 3, "ttyC0" },
	{ C_TX(FS), C_RX(FS), 4, 4, 1, 1, HIF_N, F_UH | F_CHAR | MTK_CCCI_PORT_F_CLEAN,
	  OP_CHAR, 4, "ccci_fs" },
	{ C_TX(IPC_UART), C_RX(IPC_UART), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 5, "ttyC2" },
	{ C_TX(ICUSB), C_RX(ICUSB), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 6, "ttyC3" },
	{ C_TX(MD_LOG), C_RX(MD_LOG), 2, 2, 2, 2, HIF_N, F_CHAR, OP_CHAR, 7, "ttyC1" },
	{ MTK_CCCI_IMSV_UL, MTK_CCCI_IMSV_DL, 6, 6, FF, FF, HIF_N, F_CHAR, OP_CHAR, 8, "ccci_imsv" },
	{ MTK_CCCI_IMSC_UL, MTK_CCCI_IMSC_DL, 6, 6, FF, FF, HIF_N, F_CHAR, OP_CHAR, 9, "ccci_imsc" },
	{ MTK_CCCI_IMSA_UL, MTK_CCCI_IMSA_DL, 6, 6, FF, FF, HIF_N, F_CHAR, OP_CHAR, 10, "ccci_imsa" },
	{ MTK_CCCI_IMSDC_UL, MTK_CCCI_IMSDC_DL, 6, 6, FF, FF, HIF_N, F_CHAR, OP_CHAR, 11, "ccci_imsdc" },
	{ MTK_CCCI_DUMMY_CH, MTK_CCCI_DUMMY_CH, FF, FF, FF, FF, HIF_N, F_CHAR, OP_CHAR, 12, "ccci_ioctl0" },
	{ MTK_CCCI_DUMMY_CH, MTK_CCCI_DUMMY_CH, FF, FF, FF, FF, HIF_N, F_CHAR, OP_CHAR, 13, "ccci_ioctl1" },
	{ MTK_CCCI_DUMMY_CH, MTK_CCCI_DUMMY_CH, FF, FF, FF, FF, HIF_N, F_CHAR, OP_CHAR, 14, "ccci_ioctl2" },
	{ MTK_CCCI_DUMMY_CH, MTK_CCCI_DUMMY_CH, FF, FF, FF, FF, HIF_N, F_CHAR, OP_CHAR, 15, "ccci_ioctl3" },
	{ MTK_CCCI_DUMMY_CH, MTK_CCCI_DUMMY_CH, FF, FF, FF, FF, HIF_N, F_CHAR, OP_CHAR, 16, "ccci_ioctl4" },
	{ C_TX(IT), C_RX(IT), 0, 0, FF, FF, HIF_N, F_UH | F_CHAR, OP_CHAR, 17, "ccci_it" },
	{ C_TX(LB_IT), C_RX(LB_IT), 0, 0, FF, FF, HIF_N, F_CHAR, OP_CHAR, 18, "ccci_lb_it" },
	{ MTK_CCCI_MDL_MONITOR_UL, MTK_CCCI_MDL_MONITOR_DL, 1, 1, FF, FF,
	  HIF_N, F_CHAR, OP_CHAR, 19, "ccci_mdl_monitor" },
	{ C_TX(RPC), C_RX(RPC), 1, 1, 1, 1, HIF_N, F_UH | F_CHAR, OP_RPC, 20, "ccci_rpc" },
	{ C_TX(RPC), C_RX(RPC), 1, 1, 1, 1, HIF_N, 0, OP_RPC, 0xFF, "ccci_rpc_k" },
	{ MTK_CCCI_IMSEM_UL, MTK_CCCI_IMSEM_DL, 6, 6, FF, FF, HIF_N, F_CHAR, OP_CHAR, 21, "ccci_imsem" },
	{ C_TX(IMSM), C_RX(IMSM), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 22, "ccci_imsm" },
	{ C_TX(WOA), C_RX(WOA), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 23, "ccci_woa" },
	{ C_TX(C2K_PPP), C_RX(C2K_PPP), 3, 3, FF, FF,
	  HIF_N, F_CHAR, OP_CHAR, 24, "ccci_c2k_ppp" },
	{ C_TX(C2K_AGPS), C_RX(C2K_AGPS), 1, 1, FF, FF,
	  HIF_N, F_CHAR, OP_CHAR, 25, "ccci_c2k_agps" },
	{ C_TX(XCAP), C_RX(XCAP), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 26, "ccci_ss_xcap" },
	{ C_TX(BIP), C_RX(BIP), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 27, "ccci_bip" },
	{ C_TX(TCHE), C_RX(TCHE), 2, 2, FF, FF, HIF_N, F_CHAR, OP_CHAR, 31, "ttyC5" },
	{ C_TX(DISP), C_RX(DISP), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 32, "ttyC6" },
	{ C_TX(UDC), C_RX(UDC), 1, 1, FF, FF, HIF_N, 0, OP_MISC, 30, "ccci_udc" },
	/* NOTE (MD_GENERATION 6293 < 6297): downstream port_cfg.c guards
	 * ccci_wifi_proxy m33, ccci_vts m34, ccci_0_200 m35, ccci_0_202 m36,
	 * ccci_0_204 m37 and the first ccci_ikeraw m38 behind
	 * "#if (MD_GENERATION >= 6297)", so they do NOT exist on MT6768
	 * and are deliberately absent here. Only the unconditional second
	 * ccci_ikeraw row (m39) exists; there is no m38/m39 duplicate
	 * on this generation. */
	{ C_TX(IKERAW), C_RX(IKERAW), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 39, "ccci_ikeraw" },
	{ C_TX(EPDG1), C_RX(EPDG1), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 40, "ccci_epdg1" },
	{ C_TX(EPDG2), C_RX(EPDG2), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 41, "ccci_epdg2" },
	{ C_TX(EPDG3), C_RX(EPDG3), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 42, "ccci_epdg3" },
	{ C_TX(EPDG4), C_RX(EPDG4), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 43, "ccci_epdg4" },
	{ C_TX(MIPI_CHANNEL), C_RX(MIPI_CHANNEL), 1, 1, FF, FF, HIF_N, 0, OP_MISC, 0xFF, "ccci_misc" },
	{ C_TX(AT), C_RX(AT), 1, 1, FF, FF, HIF_N, F_CHAR, OP_CHAR, 44, "ttyC_AT" },
	/* IPC char ports: dev minor = table minor + CCCI_IPC_MINOR_BASE.
	 * QUIRK (downstream verbatim): table minor 1 is skipped. */
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, F_CHAR, OP_IPC, 0, "ccci_ipc_1220_0" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, F_CHAR, OP_IPC, 2, "ccci_ipc_2" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, 0, OP_IPC, 3, "ccci_ipc_3" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, F_CHAR, OP_IPC, 4, "ccci_ipc_4" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, F_CHAR, OP_IPC, 5, "ccci_ipc_5" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, 0, OP_IPC, 6, "ccci_ipc_6" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, 0, OP_IPC, 7, "ccci_ipc_7" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, 0, OP_IPC, 8, "ccci_ipc_8" },
	{ C_TX(IPC), C_RX(IPC), 1, 1, FF, FF, HIF_N, F_CHAR, OP_IPC, 9, "ccci_ipc_9" },
	/* Kernel-only sys/ctl/poller ports (no char node, minor 0xFF) */
	{ C_TX(SYSTEM), C_RX(SYSTEM), 0, 0, FF, FF, HIF_N, 0, OP_SYS, 0xFF, "ccci_sys" },
	{ C_TX(CONTROL), C_RX(CONTROL), 0, 0, 0, 0, HIF_N, 0, OP_CTL, 0xFF, "ccci_ctrl" },
	{ C_TX(STATUS), C_RX(STATUS), 0, 0, 0, 0, HIF_N, 0, OP_POLL, 0xFF, "ccci_poll" },
	/* Smem ports: dev minor = smem user id + CCCI_SMEM_MINOR_BASE.
	 * CCB rows stay adjacent and in memory-layout order (downstream). */
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, 0,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_DBM, "ccci_raw_dbm" },
	{ MTK_CCCI_CCB_CTRL, MTK_CCCI_CCB_CTRL, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_CCB_CTRL, "ccci_ccb_ctrl" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, SQ, SQ, SQ, SQ, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_CCB_DHL, "ccci_ccb_dhl" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_DHL, "ccci_raw_dhl" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_NETD, "ccci_raw_netd" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_USB, "ccci_raw_usb" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_AUDIO, "ccci_raw_audio" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, 0,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_LWA, "ccci_raw_lwa" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, FF, FF, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_RAW_MDM, "ccci_raw_mdm" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, SQ, SQ, FF, FF, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_CCB_MD_MONITOR, "ccci_ccb_md_monitor" },
	{ MTK_CCCI_SMEM_CH, MTK_CCCI_SMEM_CH, SQ, SQ, SQ, SQ, HIF_CCIF, F_CHAR,
	  OP_SMEM, MTK_CCCI_SMEM_CCB_META, "ccci_ccb_meta" },
};

#define MTK_CCCI_MD1_PORT_COUNT ARRAY_SIZE(mtk_ccci_md1_ports)

const struct mtk_ccci_port_cfg *mtk_ccci_get_md1_ports(unsigned int *count)
{
	if (count)
		*count = MTK_CCCI_MD1_PORT_COUNT;
	return mtk_ccci_md1_ports;
}
EXPORT_SYMBOL_GPL(mtk_ccci_get_md1_ports);

const struct mtk_ccci_port_cfg *mtk_ccci_port_get_by_minor(unsigned int md_id,
							   int minor)
{
	unsigned int i;

	if (md_id != MTK_CCCI_MD_SYS1)
		return NULL;
	/* Monitor-first invariant: char space starts at the monitor
	 * entry, so a minor indexes the char class from a known base
	 * exactly like the downstream implementation requires. */
	for (i = 0; i < MTK_CCCI_MD1_PORT_COUNT; i++) {
		if (mtk_ccci_md1_ports[i].minor != minor)
			continue;
		if (mtk_ccci_md1_ports[i].ops == MTK_CCCI_OPS_NET)
			continue;
		return &mtk_ccci_md1_ports[i];
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(mtk_ccci_port_get_by_minor);

const struct mtk_ccci_port_cfg *
mtk_ccci_port_get_by_channel(unsigned int md_id, u32 ch)
{
	unsigned int i;

	if (md_id != MTK_CCCI_MD_SYS1)
		return NULL;
	for (i = 0; i < MTK_CCCI_MD1_PORT_COUNT; i++) {
		if (mtk_ccci_md1_ports[i].rx_ch == ch ||
		    mtk_ccci_md1_ports[i].tx_ch == ch)
			return &mtk_ccci_md1_ports[i];
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(mtk_ccci_port_get_by_channel);

/* ccmni index -> channel tuple (downstream port_net.c
 * ccci_get_ccmni_channel, verbatim incl. idx8 INVALID replica and the
 * dl_ack quirks: idx0/1 use DL_ACK, idx2-6 echo TX, idx7 uses DLACK_RX).
 */
static const struct {
	int rx;
	int tx;
	int dl_ack;
} mtk_ccmni_ch_map[] = {
	{ 20, 22, 64 },		/* idx0  CCMNI1 */
	{ 24, 26, 65 },		/* idx1  CCMNI2 */
	{ 28, 30, 30 },		/* idx2  CCMNI3 (dl_ack = TX, downstream) */
	{ 69, 71, 71 },		/* idx3  CCMNI4 */
	{ 74, 76, 76 },		/* idx4  CCMNI5 */
	{ 79, 81, 81 },		/* idx5  CCMNI6 */
	{ 84, 86, 86 },		/* idx6  CCMNI7 */
	{ 89, 91, 93 },		/* idx7  CCMNI8 */
	{ -1, -1, -1 },		/* idx8  INVALID replica (gap, not usable) */
	{ 103, 105, 105 },	/* idx9  CCMNI10 */
	{ 108, 110, 110 },	/* idx10 CCMNI11 */
	{ 113, 115, 115 },	/* idx11 CCMNI12 */
	{ 118, 120, 120 },	/* idx12 CCMNI13 */
	{ 123, 125, 125 },	/* idx13 CCMNI14 */
	{ 128, 130, 130 },	/* idx14 CCMNI15 */
	{ 133, 135, 135 },	/* idx15 CCMNI16 */
	{ 138, 140, 140 },	/* idx16 CCMNI17 */
	{ 143, 145, 145 },	/* idx17 CCMNI18 */
	{ 148, 150, 150 },	/* idx18 CCMNI19 */
	{ 153, 155, 155 },	/* idx19 CCMNI20 */
	{ 158, 160, 160 },	/* idx20 CCMNI21 */
	{ 96, 98, 100 },	/* idx21 CCMNI-LAN (always last) */
};

int mtk_ccmni_get_channel(unsigned int md_id, int ccmni_idx,
			  struct mtk_ccmni_ch *ch)
{
	if (!ch)
		return -EINVAL;
	if (ccmni_idx < 0 || ccmni_idx > MTK_CCMNI_LAN_INDEX)
		return -EINVAL;
	if (ccmni_idx == MTK_CCMNI_INVALID_INDEX) {
		/* idx8 is a replica for ccmni-lan and must not be used
		 * (downstream port_net.c case 8): report INVALID channels
		 * exactly like the reference instead of failing.
		 */
		ch->rx = -1;
		ch->rx_ack = 0xFF;
		ch->tx = -1;
		ch->tx_ack = 0xFF;
		ch->dl_ack = -1;
		ch->multiq = 0;
		return 0;
	}
	ch->rx = mtk_ccmni_ch_map[ccmni_idx].rx;
	ch->rx_ack = 0xFF;
	ch->tx = mtk_ccmni_ch_map[ccmni_idx].tx;
	ch->tx_ack = 0xFF;
	ch->dl_ack = mtk_ccmni_ch_map[ccmni_idx].dl_ack;
	ch->multiq = (md_id == MTK_CCCI_MD_SYS1 &&
		      ccmni_idx != MTK_CCMNI_LAN_INDEX) ? 1 : 0;
	return 0;
}

int mtk_ccmni_ifname(int idx, char *buf, unsigned int len)
{
	int ret;

	if (!buf || len == 0)
		return -EINVAL;
	if (idx < 0 || idx > MTK_CCMNI_LAN_INDEX ||
	    idx == MTK_CCMNI_INVALID_INDEX)
		return -EINVAL;
	if (idx == MTK_CCMNI_LAN_INDEX)
		ret = snprintf(buf, len, "ccmni-lan");
	else
		ret = snprintf(buf, len, "%s%d", MTK_CCMNI_IF_NAME, idx);
	if (ret < 0 || (unsigned int)ret >= len)
		return -EINVAL;
	return 0;
}
