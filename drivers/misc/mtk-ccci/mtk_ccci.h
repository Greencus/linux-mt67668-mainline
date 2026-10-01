// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6768 CCCI modem transport (mainline clean-room port, Track A).
 *
 * Shared definitions. Values traced from the downstream reference tree
 * (read-only, kernel/downstream/); see docs/CCCI-MAINLINE-ARCHITECTURE.md
 * for the per-symbol evidence pointers. This file contains no downstream
 * code, only re-typed numeric/string constants and the port-table shape.
 */

#ifndef _MTK_CCCI_H_
#define _MTK_CCCI_H_

#include <linux/bits.h>
#include <linux/types.h>
/* NOTE: include/linux/sockios.h is absent from this tree, so the private
 * ioctl base is taken from the UAPI header directly (SIOCDEVPRIVATE 0x89F0).
 */
#include <uapi/linux/sockios.h>

/* Platform identity (downstream ccci_config.h: AP_PLATFORM_INFO "MT6768",
 * MD_GENERATION 6293). Kept as informational strings only.
 */
#define MTK_CCCI_AP_PLATFORM		"MT6768"
#define MTK_CCCI_MD_GENERATION		6293
#define MTK_CCCI_DRIVER_VER		0x20110118

/* Buffer / MTU sizing (downstream ccci_config.h) */
#define MTK_CCCI_MTU			(3584 - 128)
#define MTK_CCCI_NET_MTU		1500
#define MTK_CCCI_SMEM_RUNTIME_AP	0x800
#define MTK_CCCI_SMEM_RUNTIME_MD	0x800

/* Modem instances (MD_SYS1 only on MT6768; MD_SYS3/C2K not populated) */
#define MTK_CCCI_MD_SYS1		0
#define MTK_CCCI_MAX_MD_NUM		1

/* Char/smem minor bases (downstream ccci_core.h:40-42) */
#define MTK_CCCI_IPC_MINOR_BASE		100
#define MTK_CCCI_SMEM_MINOR_BASE	150
#define MTK_CCCI_NET_MINOR_BASE		200

/* CLDMA queue selectors (downstream port_cfg.c: MD_GENERATION > 6292) */
#define MTK_CCCI_EXP_CTRL_Q		6
#define MTK_CCCI_DATA_TX_Q		0
#define MTK_CCCI_DATA_RX_Q		0
#define MTK_CCCI_DATA_TX_ACK_Q		1
#define MTK_CCCI_SMEM_Q			6	/* AP_MD_CCB_WAKEUP placeholder */

/* HIF identifiers (downstream ccci_hif.h enum CCCI_HIF order
 * CLDMA=0, CCIF=1, DPMAIF=2, with MD_GENERATION == 6293 selecting
 * MD1_NET_HIF = CLDMA and MD1_NORMAL_HIF = CCIF).
 */
#define MTK_CCCI_MD1_NORMAL_HIF		1
#define MTK_CCCI_MD1_NET_HIF		0
#define MTK_CCCI_CCIF_HIF_ID		1

/* Port flags (downstream port_t.h semantics, renamed) */
#define MTK_CCCI_PORT_F_USER_HEADER	BIT(0)
#define MTK_CCCI_PORT_F_WITH_CHAR_NODE	BIT(1)
#define MTK_CCCI_PORT_F_CLEAN		BIT(2)

/* ccmni netdev identity (downstream port_cfg.c / ccmni.h) */
#define MTK_CCMNI_IF_NAME		"ccmni"
#define MTK_CCMNI_MTU			1500
#define MTK_CCMNI_LAN_INDEX		21
#define MTK_CCMNI_INVALID_INDEX		8	/* gap: no CCCI_CCMNI9_* channels */

/* ccmni private ioctls (downstream ccmni.h:53-61, SIOCDEVPRIVATE+0..4) */
#define MTK_CCMNI_SIOCSTXQSTATE		(SIOCDEVPRIVATE + 0)
#define MTK_CCMNI_SIOCCCMNICFG		(SIOCDEVPRIVATE + 1)
#define MTK_CCMNI_SIOCFWDFILTER		(SIOCDEVPRIVATE + 2)
#define MTK_CCMNI_SIOCACKPRIO		(SIOCDEVPRIVATE + 3)
#define MTK_CCMNI_SIOPUSHPENDING	(SIOCDEVPRIVATE + 4)

/* CCCI channel IDs (downstream ccci_core.h enum CCCI_CH, exact values).
 * Only channels referenced by the MD1 port table plus the control/system/
 * status/poller/smem virtual channels are listed; values are verbatim.
 */
enum mtk_ccci_ch {
	MTK_CCCI_CONTROL_RX = 0,
	MTK_CCCI_CONTROL_TX = 1,
	MTK_CCCI_SYSTEM_RX = 2,
	MTK_CCCI_SYSTEM_TX = 3,
	MTK_CCCI_PCM_RX = 4,
	MTK_CCCI_PCM_TX = 5,
	MTK_CCCI_UART1_RX = 6,
	MTK_CCCI_UART1_TX = 8,
	MTK_CCCI_UART2_RX = 10,
	MTK_CCCI_UART2_TX = 12,
	MTK_CCCI_FS_RX = 14,
	MTK_CCCI_FS_TX = 15,
	MTK_CCCI_RPC_RX = 32,
	MTK_CCCI_RPC_TX = 33,
	MTK_CCCI_IPC_RX = 34,
	MTK_CCCI_IPC_TX = 36,
	MTK_CCCI_IPC_UART_RX = 38,
	MTK_CCCI_IPC_UART_TX = 40,
	MTK_CCCI_MD_LOG_RX = 42,
	MTK_CCCI_MD_LOG_TX = 43,
	MTK_CCCI_IT_RX = 50,
	MTK_CCCI_IT_TX = 51,
	MTK_CCCI_IMSV_UL = 52,
	MTK_CCCI_IMSV_DL = 53,
	MTK_CCCI_IMSC_UL = 54,
	MTK_CCCI_IMSC_DL = 55,
	MTK_CCCI_IMSA_UL = 56,
	MTK_CCCI_IMSA_DL = 57,
	MTK_CCCI_IMSDC_UL = 58,
	MTK_CCCI_IMSDC_DL = 59,
	MTK_CCCI_ICUSB_RX = 60,
	MTK_CCCI_ICUSB_TX = 61,
	MTK_CCCI_LB_IT_RX = 62,
	MTK_CCCI_LB_IT_TX = 63,
	MTK_CCCI_STATUS_RX = 67,
	MTK_CCCI_STATUS_TX = 68,
	MTK_CCCI_MDL_MONITOR_DL = 94,
	MTK_CCCI_MDL_MONITOR_UL = 95,
	MTK_CCCI_CCMNILAN_RX = 96,
	MTK_CCCI_CCMNILAN_TX = 98,
	MTK_CCCI_CCMNILAN_DLACK_RX = 100,
	MTK_CCCI_IMSEM_UL = 101,
	MTK_CCCI_IMSEM_DL = 102,
	/* CCMNI1..8: RX=20+5*(n-1) pattern is NOT linear; exact values: */
	MTK_CCCI_CCMNI1_RX = 20,
	MTK_CCCI_CCMNI1_TX = 22,
	MTK_CCCI_CCMNI1_DL_ACK = 64,
	MTK_CCCI_CCMNI2_RX = 24,
	MTK_CCCI_CCMNI2_TX = 26,
	MTK_CCCI_CCMNI2_DL_ACK = 65,
	MTK_CCCI_CCMNI3_RX = 28,
	MTK_CCCI_CCMNI3_TX = 30,
	MTK_CCCI_CCMNI3_DL_ACK = 66,
	MTK_CCCI_CCMNI4_RX = 69,
	MTK_CCCI_CCMNI4_TX = 71,
	MTK_CCCI_CCMNI4_DLACK_RX = 73,
	MTK_CCCI_CCMNI5_RX = 74,
	MTK_CCCI_CCMNI5_TX = 76,
	MTK_CCCI_CCMNI5_DLACK_RX = 78,
	MTK_CCCI_CCMNI6_RX = 79,
	MTK_CCCI_CCMNI6_TX = 81,
	MTK_CCCI_CCMNI6_DLACK_RX = 83,
	MTK_CCCI_CCMNI7_RX = 84,
	MTK_CCCI_CCMNI7_TX = 86,
	MTK_CCCI_CCMNI7_DLACK_RX = 88,
	MTK_CCCI_CCMNI8_RX = 89,
	MTK_CCCI_CCMNI8_TX = 91,
	MTK_CCCI_CCMNI8_DLACK_RX = 93,
	/* CCMNI10..21: RX=103+5*(n-10), TX=RX+2, DLACK=RX+4 */
	MTK_CCCI_CCMNI10_RX = 103,
	MTK_CCCI_CCMNI10_TX = 105,
	MTK_CCCI_CCMNI11_RX = 108,
	MTK_CCCI_CCMNI11_TX = 110,
	MTK_CCCI_CCMNI12_RX = 113,
	MTK_CCCI_CCMNI12_TX = 115,
	MTK_CCCI_CCMNI13_RX = 118,
	MTK_CCCI_CCMNI13_TX = 120,
	MTK_CCCI_CCMNI14_RX = 123,
	MTK_CCCI_CCMNI14_TX = 125,
	MTK_CCCI_CCMNI15_RX = 128,
	MTK_CCCI_CCMNI15_TX = 130,
	MTK_CCCI_CCMNI16_RX = 133,
	MTK_CCCI_CCMNI16_TX = 135,
	MTK_CCCI_CCMNI17_RX = 138,
	MTK_CCCI_CCMNI17_TX = 140,
	MTK_CCCI_CCMNI18_RX = 143,
	MTK_CCCI_CCMNI18_TX = 145,
	MTK_CCCI_CCMNI19_RX = 148,
	MTK_CCCI_CCMNI19_TX = 150,
	MTK_CCCI_CCMNI20_RX = 153,
	MTK_CCCI_CCMNI20_TX = 155,
	MTK_CCCI_CCMNI21_RX = 158,
	MTK_CCCI_CCMNI21_TX = 160,
	MTK_CCCI_IMSM_RX = 169,
	MTK_CCCI_IMSM_TX = 170,
	MTK_CCCI_WOA_RX = 171,
	MTK_CCCI_WOA_TX = 172,
	MTK_CCCI_XCAP_RX = 173,
	MTK_CCCI_XCAP_TX = 174,
	MTK_CCCI_BIP_RX = 175,
	MTK_CCCI_BIP_TX = 176,
	MTK_CCCI_UDC_RX = 177,
	MTK_CCCI_UDC_TX = 178,
	MTK_CCCI_MIPI_CHANNEL_RX = 179,
	MTK_CCCI_MIPI_CHANNEL_TX = 180,
	MTK_CCCI_TCHE_RX = 181,
	MTK_CCCI_TCHE_TX = 182,
	MTK_CCCI_DISP_RX = 183,
	MTK_CCCI_DISP_TX = 184,
	MTK_CCCI_WIFI_RX = 187,
	MTK_CCCI_WIFI_TX = 188,
	MTK_CCCI_VTS_RX = 189,
	MTK_CCCI_VTS_TX = 190,
	MTK_CCCI_IKERAW_RX = 191,
	MTK_CCCI_IKERAW_TX = 192,
	MTK_CCCI_MD_DIRC_RX = 200,
	MTK_CCCI_MD_DIRC_TX = 201,
	MTK_CCCI_TIME_RX = 202,
	MTK_CCCI_TIME_TX = 203,
	MTK_CCCI_GARB_RX = 204,
	MTK_CCCI_GARB_TX = 205,
	MTK_CCCI_EPDG1_RX = 236,
	MTK_CCCI_EPDG1_TX = 237,
	MTK_CCCI_EPDG2_RX = 238,
	MTK_CCCI_EPDG2_TX = 239,
	MTK_CCCI_EPDG3_RX = 240,
	MTK_CCCI_EPDG3_TX = 241,
	MTK_CCCI_EPDG4_RX = 242,
	MTK_CCCI_EPDG4_TX = 243,
	MTK_CCCI_AT_RX = 258,
	MTK_CCCI_AT_TX = 259,
	MTK_CCCI_C2K_PPP_RX = 165,
	MTK_CCCI_C2K_PPP_TX = 166,
	MTK_CCCI_C2K_AGPS_RX = 167,
	MTK_CCCI_C2K_AGPS_TX = 168,
	/* Virtual channels */
	MTK_CCCI_DUMMY_CH = 270,
	MTK_CCCI_SMEM_CH = 271,
	MTK_CCCI_CCB_CTRL = 272,
	MTK_CCCI_INVALID_CH_ID = 0xffffffff,
};

/* Shared-memory users (downstream mtk_ccci_common.h enum SMEM_USER_ID).
 * Order matters for CCB users; kept verbatim for the subset we expose.
 */
enum mtk_ccci_smem_user {
	MTK_CCCI_SMEM_RAW_DBM = 0,
	MTK_CCCI_SMEM_CCB_START,
	MTK_CCCI_SMEM_CCB_DHL = MTK_CCCI_SMEM_CCB_START,
	MTK_CCCI_SMEM_CCB_MD_MONITOR,
	MTK_CCCI_SMEM_CCB_META,
	MTK_CCCI_SMEM_CCB_END = MTK_CCCI_SMEM_CCB_META,
	MTK_CCCI_SMEM_RAW_CCB_CTRL,
	MTK_CCCI_SMEM_RAW_DHL,
	MTK_CCCI_SMEM_RAW_MDM,
	MTK_CCCI_SMEM_RAW_NETD,
	MTK_CCCI_SMEM_RAW_USB,
	MTK_CCCI_SMEM_RAW_AUDIO,
	MTK_CCCI_SMEM_RAW_DFD,
	MTK_CCCI_SMEM_RAW_LWA,
	MTK_CCCI_SMEM_RAW_MDCCCI_DBG,
	MTK_CCCI_SMEM_RAW_MDSS_DBG,
	MTK_CCCI_SMEM_RAW_RUNTIME_DATA,
	MTK_CCCI_SMEM_RAW_FORCE_ASSERT,
	MTK_CCCI_SMEM_MAX,
};

/* Modem execution-environment states (downstream ccci_fsm.c model).
 * NOTE: there are deliberately NO BOOT_READY / BOOT_UP tokens: the
 * handshake is a two-stage HS1 -> runtime-data -> HS2 sequence carried
 * inside the FSM (see modem file). Names below mirror the FSM states.
 */
enum mtk_ccci_md_state {
	MTK_CCCI_MD_STATE_INVALID = 0,
	MTK_CCCI_MD_STATE_BOOT_WAITING_FOR_HS1,
	MTK_CCCI_MD_STATE_BOOT_WAITING_FOR_HS2,
	MTK_CCCI_MD_STATE_READY,
	MTK_CCCI_MD_STATE_EXCEPTION,
	MTK_CCCI_MD_STATE_MAX,
};

/* EE handshake stages dispatched from the modem layer
 * (downstream ccci_modem.c:1568 dispatch into modem_sys1/sys3).
 */
enum mtk_ccci_hif_ex_stage {
	MTK_CCCI_HIF_EX_INIT = 0,
	MTK_CCCI_HIF_EX_INIT_DONE,
	MTK_CCCI_HIF_EX_CLEARQ_DONE,
	MTK_CCCI_HIF_EX_ALLQ_RESET,
	MTK_CCCI_HIF_EX_MAX,
};

/* Port operation classes (downstream char/rpc/ipc/sys/ctl/poller vs net) */
enum mtk_ccci_port_ops {
	MTK_CCCI_OPS_CHAR = 0,
	MTK_CCCI_OPS_NET,
	MTK_CCCI_OPS_RPC,
	MTK_CCCI_OPS_IPC,
	MTK_CCCI_OPS_SYS,
	MTK_CCCI_OPS_CTL,
	MTK_CCCI_OPS_POLLER,
	MTK_CCCI_OPS_SMEM,
	MTK_CCCI_OPS_MISC,
};

/* One port-table row. Mirrors downstream struct port_t field order
 * (tx, rx, txq, rxq, tx_exp, rx_exp, hif, flags, ops, minor, name)
 * so the md1 table below can be audited line-by-line against
 * downstream port_cfg.c.
 */
struct mtk_ccci_port_cfg {
	u32 tx_ch;
	u32 rx_ch;
	u8 txq;
	u8 rxq;
	u8 txq_exp;
	u8 rxq_exp;
	u8 hif_id;
	u32 flags;
	enum mtk_ccci_port_ops ops;
	int minor;
	const char *name;
};

/* ccmni channel tuple (downstream struct ccmni_ch) */
struct mtk_ccmni_ch {
	int rx;
	int rx_ack;
	int tx;
	int tx_ack;
	int dl_ack;
	int multiq;
};

/* TTY indices that back real downstream ports (minor == index):
 * ttyC0 m3, ttyC2 m5, ttyC3 m6, ttyC1 m7, ttyC5 m31, ttyC6 m32,
 * ttyC_AT m44 (downstream port_cfg.c, preserved verbatim).
 */
#define MTK_CCCI_TTY_COUNT	7

static inline bool mtk_ccci_tty_index_valid(unsigned int index)
{
	switch (index) {
	case 3:
	case 5:
	case 6:
	case 7:
	case 31:
	case 32:
	case 44:
		return true;
	default:
		return false;
	}
}

/* RAW-IP framing rule (downstream ccmni is a pure-IP device, NOT QMI/MBIM):
 * the first nibble must be 4 (IPv4) or 6 (IPv6); anything else is not an
 * IP packet and is dropped + counted.
 */
static inline bool mtk_ccci_is_raw_ip_frame(const u8 *data, unsigned int len)
{
	u8 ver;

	if (!data || len < 1)
		return false;
	ver = (data[0] >> 4) & 0xF;
	return ver == 4 || ver == 6;
}

/* MTU rule (downstream ccmni_change_mtu: only the upper bound is enforced
 * against CCMNI_MTU; the lower bound 68 is the IPv4 minimum reassembled
 * datagram floor kept by this driver).
 */
static inline bool mtk_ccmni_mtu_valid(int mtu)
{
	return mtu >= 68 && mtu <= MTK_CCMNI_MTU;
}

/* Cross-module API (all implemented in this directory) */
struct mtk_ccci_modem;

int mtk_ccci_core_init(void);
void mtk_ccci_core_exit(void);
int mtk_ccci_get_md_state(unsigned int md_id);
bool mtk_ccci_md_is_enabled(unsigned int md_id);

int mtk_ccci_modem_register(struct mtk_ccci_modem *md);
void mtk_ccci_modem_unregister(struct mtk_ccci_modem *md);
int mtk_ccci_modem_ee_handshake(struct mtk_ccci_modem *md, int timeout_ms);
int mtk_ccci_modem_exception(struct mtk_ccci_modem *md,
			     enum mtk_ccci_hif_ex_stage stage);
int mtk_ccci_modem_get_boot_stats(struct mtk_ccci_modem *md,
				  u32 *stats0, u32 *stats1);

const struct mtk_ccci_port_cfg *mtk_ccci_get_md1_ports(unsigned int *count);
const struct mtk_ccci_port_cfg *mtk_ccci_port_get_by_minor(unsigned int md_id,
							   int minor);
const struct mtk_ccci_port_cfg *mtk_ccci_port_get_by_channel(unsigned int md_id,
							     u32 ch);
int mtk_ccmni_get_channel(unsigned int md_id, int ccmni_idx,
			  struct mtk_ccmni_ch *ch);
/* Interface-name rule (downstream ccmni register loop): "ccmni%d" for
 * idx0-7/9-20, "ccmni-lan" for idx21. Returns 0 on success, -EINVAL for
 * the idx8 gap or truncation.
 */
int mtk_ccmni_ifname(int idx, char *buf, unsigned int len);

int mtk_ccci_tty_register(void);
void mtk_ccci_tty_unregister(void);

int mtk_ccmni_register_all(unsigned int md_id);
void mtk_ccmni_unregister_all(unsigned int md_id);

#endif /* _MTK_CCCI_H_ */
