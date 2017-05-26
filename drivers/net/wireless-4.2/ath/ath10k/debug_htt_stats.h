/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _DEBUG_HTT_H_
#define _DEBUG_HTT_H_

#include "htt.h"

#ifdef CONFIG_ATH10K_DEBUGFS

int ath10k_htt_process_stats(struct ath10k *ar, struct sk_buff *skb);

#else

static inline int ath10k_htt_process_stats(struct ath10k *ar,
					   struct sk_buff *skb)
{
	return 0;
}

#endif /* CPTCFG_ATH10K_DEBUGFS */

struct htt_10_4_wal_tx_stats {
	/* Num HTT cookies queued to dispatch list */
	__le32 comp_queued;

	/* Num HTT cookies dispatched */
	__le32 comp_delivered;

	/* Num MSDU queued to WAL */
	__le32 msdu_enqued;

	/* Num MPDU queue to WAL */
	__le32 mpdu_enqued;

	/* Num MSDUs dropped by WMM limit */
	__le32 wmm_drop;

	/* Num Local frames queued */
	__le32 local_enqued;

	/* Num Local frames done */
	__le32 local_freed;

	/* Num queued to HW */
	__le32 hw_queued;

	/* Num PPDU reaped from HW */
	__le32 hw_reaped;

	/* Num underruns */
	__le32 underrun;

	__le32 hw_paused;

	/* Num PPDUs cleaned up in TX abort */
	__le32 tx_abort;

	/* Num MPDUs requed by SW */
	__le32 mpdus_requed;

	/* excessive retries */
	__le32 tx_ko;

	/* data hw rate code */
	__le32 data_rc;

	/* Scheduler self triggers */
	__le32 self_triggers;

	/* frames dropped due to excessive sw retries */
	__le32 sw_retry_failure;

	/* illegal rate phy errors  */
	__le32 illgl_rate_phy_err;

	/* wal pdev continuous xretry */
	__le32 pdev_cont_xretry;

	/* wal pdev continuous xretry */
	__le32 pdev_tx_timeout;

	/* wal pdev resets  */
	__le32 pdev_resets;
	__le32 stateless_tid_alloc_failure;
	__le32 phy_underrun;
	__le32 txop_ovf;
	__le32 seq_posted;
	__le32 seq_failed_queueing;
	__le32 seq_completed;
	__le32 seq_restarted;
	__le32 mu_seq_posted;
	__le32 mpdus_sw_flush;
	__le32 mpdus_hw_filter;
	__le32 mpdus_truncated;
	__le32 mpdus_ack_failed;
	__le32 mpdus_expired;
	__le32 mc_drop;
} __packed;

struct htt_10_4_wal_rx_stats {
	/* Cnts any change in ring routing mid-ppdu */
	__le32 mid_ppdu_route_change;

	/* Total number of statuses processed */
	__le32 status_rcvd;

	/* Extra frags on rings 0-3 */
	__le32 r0_frags;
	__le32 r1_frags;
	__le32 r2_frags;
	/*__le32 r3_frags;*/

	/* MSDUs / MPDUs delivered to HTT */
	__le32 htt_msdus;
	__le32 htt_mpdus;

	/* MSDUs / MPDUs delivered to local stack */
	__le32 loc_msdus;
	__le32 loc_mpdus;

	/* AMSDUs that have more MSDUs than the status ring size */
	__le32 oversize_amsdu;

	/* Number of PHY errors */
	__le32 phy_errs;

	/* Number of PHY errors drops */
	__le32 phy_err_drop;

	/* Number of mpdu errors - FCS, MIC, ENC etc. */
	__le32 mpdu_errs;

	/* Number of rx inactivity timeouts */
	__le32 pdev_rx_timeout;

	__le32 rx_ovfl_errs;
} __packed;

struct htt_10_4_wal_mem_stats {
	__le32 iram_free_size;
	__le32 dram_free_size;
	__le32 sram_free_size;
} __packed;

struct htt_10_4_wal_pdev_txrx {
	struct htt_10_4_wal_tx_stats tx_stats;
	struct htt_10_4_wal_rx_stats rx_stats;
	struct htt_10_4_wal_mem_stats mem_stats;
} __packed;

struct htt_10_4_rx_rate_info {
	__le32 mcs[10];
	__le32 sgi[10];
	__le32 nss[4];
	__le32 nsts;
	__le32 stbc[10];
	__le32 bw[4];
	__le32 pream[6];
	__le32 ldpc;
	__le32 txbf;
	__le32 rssi_chain0;
	__le32 rssi_chain1;
	__le32 rssi_chain2;
	__le32 rssi_chain3;
	u8 mgmt_rssi;
	u8 data_rssi;
	u8 rssi_comb_ht;
} __packed;

struct htt_10_4_tx_rate_info {
	__le32 mcs[10];
	__le32 sgi[10];
	__le32 nss[4];
	__le32 stbc[10];
	__le32 bw[4];
	__le32 pream[4];
	__le32 ldpc;
	__le32 rts_cnt;
	__le32 ack_rssi;
	__le32 mu_mcs[10];
} __packed;

#define DBG_STATS_MAX_HWQ_NUM 10
#define DBG_STATS_MAX_TID_NUM 20
#define DBG_STATS_MAX_CONG_NUM 16

struct wlan_10_x_txq_stats {
	__le16 num_pkts_queued[DBG_STATS_MAX_HWQ_NUM];
	__le16 tid_hw_qdepth[DBG_STATS_MAX_TID_NUM];
	__le16 tid_sw_qdepth[DBG_STATS_MAX_TID_NUM];
} __packed;

struct htt_10_x_tidq_stats {
	__le32  wlan_dbg_tid_txq_status;
	struct wlan_10_x_txq_stats txq_st;
} __packed;

#define WLAN_DBG_STATS_SIZE_TXBF_VHT 10
#define WLAN_DBG_STATS_SIZE_TXBF_HT 8
#define WLAN_DBG_STATS_SIZE_TXBF_OFDM 8

struct wlan_10_4_txbf_data_stats {
	__le32 tx_txbf_vht[WLAN_DBG_STATS_SIZE_TXBF_VHT];
	__le32 rx_txbf_vht[WLAN_DBG_STATS_SIZE_TXBF_VHT];
	__le32 tx_txbf_ht[WLAN_DBG_STATS_SIZE_TXBF_HT];
	__le32 tx_txbf_ofdm[WLAN_DBG_STATS_SIZE_TXBF_OFDM];
	__le32 tx_txbf_ibf_vht[WLAN_DBG_STATS_SIZE_TXBF_VHT];
	__le32 tx_txbf_ibf_ht[WLAN_DBG_STATS_SIZE_TXBF_HT];
	__le32 tx_txbf_ibf_ofdm[WLAN_DBG_STATS_SIZE_TXBF_OFDM];
} __packed;

#define NUM_OF_SOUNDING_STATS_WORDS (3 * 4) /*max_users * max_bw) */

struct wlan_10_4_txbf_snd_stats {
	__le32 cbf_20[4];
	__le32 cbf_40[4];
	__le32 cbf_80[4];
	__le32 cbf_160[4];

	/*
	* The sounding array is a 2-D array stored as an 1-D array of
	* u32. The stats for a particular mu_user/bw combination is
	* referenced with the following:
	*
	*          sounding[(mu_user* max_bw) + bw]"
	*
	* ... where max_bw == 4 for 160mhz HW and max_bw == 3 for
	* everything else. The maximum bandwidth must be determined
	* through some other means.
	*/
	__le32 sounding[NUM_OF_SOUNDING_STATS_WORDS];
} __packed;

#define WHAL_DBG_PHY_ERR_MAXCNT 21
#define WHAL_DBG_FLUSH_REASON_MAXCNT 18
#define WHAL_DBG_CMD_STALL_ERR_MAXCNT 4
#define WHAL_DBG_CMD_RESULT_MAXCNT 8
#define WHAL_DBG_SIFS_STATUS_MAXCNT 8
#define WHAL_DBG_SIFS_ERR_MAXCNT 8

enum {
	WIFI_URRN_STATS_FIRST_PKT,
	WIFI_URRN_STATS_BETWEEN_MPDU,
	WIFI_URRN_STATS_WITHIN_MPDU,
	WHAL_MAX_URRN_STATS
};

struct wlan_10_4_wifi2_error_stats {
	__le32 urrn_stats[WHAL_MAX_URRN_STATS];
	__le32 flush_errs[WHAL_DBG_FLUSH_REASON_MAXCNT];
	__le32 schd_stall_errs[WHAL_DBG_CMD_STALL_ERR_MAXCNT];
	__le32 schd_cmd_result[WHAL_DBG_CMD_RESULT_MAXCNT];
	__le32 sifs_status[WHAL_DBG_SIFS_STATUS_MAXCNT];
	u8 phy_errs[WHAL_DBG_PHY_ERR_MAXCNT];
	__le32 rx_rate_inval;
} __packed;

struct wlan_10_4_tx_selfgen_stats {
	__le32 su_ndpa;
	__le32 su_ndp;
	__le32 mu_ndpa;
	__le32 mu_ndp;
	__le32 mu_brpoll_1;
	__le32 mu_brpoll_2;
	__le32 su_bar;
	__le32 mu_bar_1;
	__le32 mu_bar_2;
	__le32 su_cts;
	__le32 mu_cts;
	__le32 su_ndpa_err;
	__le32 mu_ndpa_err;
	__le32 su_ndp_err;
	__le32 mu_ndp_err;
	__le32 mu_brp1_err;
	__le32 mu_brp2_err;
} __packed;

struct wlan_10_4_tx_mu_stats {
	__le32 mu_sch_nusers_2;
	__le32 mu_sch_nusers_3;
	__le32 mu_mpdus_queued_usr[4];
	__le32 mu_mpdus_tried_usr[4];
	__le32 mu_mpdus_failed_usr[4];
	__le32 mu_mpdus_requeued_usr[4];
	__le32 mu_err_no_ba_usr[4];
	__le32 mu_mpdu_underrun_usr[4];
	__le32 mu_ampdu_underrun_usr[4];
} __packed;

struct wlan_10_4_sifs_resp_stats {
	__le32  ps_poll_trigger;
	__le32  uapsd_trigger;
	__le32  qb_data_trigger[2];
	__le32  qb_bar_trigger[2];
	__le32  sifs_resp_data;
	__le32  sifs_resp_err;
} __packed;

struct wlan_10_4_reset_stats {
	__le16 warm_reset;
	__le16 cold_reset;
	__le16 tx_flush;
	__le16 tx_glb_reset;
	__le16 tx_txq_reset;
	__le16 rx_timeout_reset;
	__le16 hw_status_mismatch;
	__le16 hw_status_multi_mismatch;
} __packed;

struct wlan_10_4_mac_wdog_stats {
	__le16 rxpcu;
	__le16 txpcu;
	__le16 ole;
	__le16 rxdma;
	__le16 hwsch;
	__le16 crypto;
	__le16 pdg;
	__le16 txdma;
} __packed;

#define WLAN_TX_DESC_MAX_BINS 9

/* Set/Get helper macros to encode/decode values in to 32bit fields
 * that facilitates easy access on both little/big-endian hosts
 */

#define WLAN_DBG_TX_DESC_BIN_IDX_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, cfg_min__bin_idx)))) \
	>> 0) & 0xFF)

#define WLAN_DBG_TX_DESC_CFG_MIN_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, cfg_min__bin_idx)))) \
	>> 16) & 0xFFFF)

#define WLAN_DBG_TX_DESC_CFG_MAX_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, cfg_prio__cfg_max)))) \
	>> 0) & 0xFFFF)

#define WLAN_DBG_TX_DESC_CFG_PRIO_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, cfg_prio__cfg_max)))) \
	>> 16) & 0xFF)

#define WLAN_DBG_TX_DESC_CFG_BIN_HYST_THR_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, curr_total__cfg_bin_hist_th)))) \
	>> 0) & 0xFFFF)

#define WLAN_DBG_TX_DESC_CURR_TOT_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, curr_total__cfg_bin_hist_th)))) \
	>> 16) & 0xFFFF)

#define WLAN_DBG_TX_DESC_PREALLOC_CNT_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, bin_max__pre_alloc_cnt)))) \
	>> 0) & 0xFFFF)

#define WLAN_DBG_TX_DESC_BIN_MAX_GET(tx_desc_bin) \
	(((*((__le32 *)(((u8 *) tx_desc_bin) + \
	offsetof(struct wlan_10_4_dbg_tx_desc_bin_t, bin_max__pre_alloc_cnt)))) \
	>> 16) & 0xFFFF)

struct wlan_10_4_dbg_tx_desc_bin_t {
	__le32 cfg_min__bin_idx; /* bits 15-8 unused */
	__le32 cfg_prio__cfg_max; /* 31-24 unused */
	__le32 curr_total__cfg_bin_hist_th;
	__le32 bin_max__pre_alloc_cnt;
	__le32 bin_hist_low;
	__le32 bin_hist_high;
} __packed;

#define WLAN_DBG_TX_DESC_CFG_TOTAL_GET(stats) \
	(((stats) & 0xFFFF) >> 0)

#define WLAN_DBG_TX_DESC_TOTAL_AVAIL_GET(stats) \
	(((stats) & 0xFFFF0000) >> 16)

struct wlan_10_4_dbg_tx_desc_stats {
	__le32 word1;
	struct wlan_10_4_dbg_tx_desc_bin_t bin_stats[WLAN_TX_DESC_MAX_BINS];
} __packed;

#define WAL_STATS_PREFETCH_MAX_QUEUES 4
#define WAL_STATS_FETCH_MGR_MAX_HIST_SLOTS 8

/* Set/Get helper macros to encode/decode values in to 32bit fields
 * that facilitates easy access on both little/big-endian hosts
 */

#define WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DUR_GET(stats) \
	(((stats) & 0xFFFF) >> 0)

#define WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DESC_GET(stats) \
	(((stats) & 0xFFFF0000) >> 16)

struct wlan_10_4_dbg_tx_fetch_mgr_stats {
	__le32 fetch_desc__fetch_dur[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 fetch_mgr_total_outstanding_fetch_desc;
	__le32 fetch_mgr_rtt_histogram_4ms[WAL_STATS_FETCH_MGR_MAX_HIST_SLOTS];
	__le32 fetch_mgr_rtt_histogram_500us[WAL_STATS_FETCH_MGR_MAX_HIST_SLOTS];
} __packed;

struct wlan_10_4_dbg_tx_pf_sched_stats {
	__le32 tx_queued[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 tx_reaped[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 tx_sched[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 abort_sched[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 sched_timeout[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 tx_sched_waitq[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 fetch_resp[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 fetch_resp_invld[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 fetch_resp_delayed[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 fetch_request[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 tx_requeued[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 sched_fail[WAL_STATS_PREFETCH_MAX_QUEUES];
	__le32 pfsched_peer_skipped[WAL_STATS_PREFETCH_MAX_QUEUES];
} __packed;

struct wlan_10_4_htt_stats {
	struct htt_10_4_wal_pdev_txrx txrx_stats;
        struct htt_dbg_stats_rx_reorder_stats rx_reorder_stats;
        struct htt_10_4_rx_rate_info rx_rate_info;
        struct htt_10_4_tx_rate_info tx_rate_info;
        struct htt_10_x_tidq_stats tidq;
        struct wlan_10_4_txbf_data_stats txbf_data_stats;
        struct wlan_10_4_txbf_snd_stats txbf_snd_stats;
        struct wlan_10_4_wifi2_error_stats error_stats;
        struct wlan_10_4_tx_selfgen_stats tx_selfgen_stats;
        struct wlan_10_4_tx_mu_stats tx_mu_stats;
        struct wlan_10_4_sifs_resp_stats resp_stats;
        struct wlan_10_4_reset_stats reset_stats;
        struct wlan_10_4_mac_wdog_stats wdog_stats;
        struct wlan_10_4_dbg_tx_desc_stats tx_desc_stats;
        struct wlan_10_4_dbg_tx_fetch_mgr_stats tx_fetch_mgr_stats;
        struct wlan_10_4_dbg_tx_pf_sched_stats tx_pf_sched_stats;
} __packed;

struct htt_10_2_wal_tx_stats {
	/* Num HTT cookies queued to dispatch list */
	__le32 comp_queued;

	/* Num HTT cookies dispatched */
	__le32 comp_delivered;

	/* Num MSDU queued to WAL */
	__le32 msdu_enqued;

	/* Num MPDU queue to WAL */
	__le32 mpdu_enqued;

	/* Num MSDUs dropped by WMM limit */
	__le32 wmm_drop;

	/* Num Local frames queued */
	__le32 local_enqued;

	/* Num Local frames done */
	__le32 local_freed;

	/* Num queued to HW */
	__le32 hw_queued;

	/* Num PPDU reaped from HW */
	__le32 hw_reaped;

	/* Num underruns */
	__le32 underrun;

	__le32 hw_paused;

	/* Num PPDUs cleaned up in TX abort */
	__le32 tx_abort;

	/* Num MPDUs requed by SW */
	__le32 mpdus_requed;

	/* excessive retries */
	__le32 tx_ko;

	/* data hw rate code */
	__le32 data_rc;

	/* Scheduler self triggers */
	__le32 self_triggers;

	/* frames dropped due to excessive sw retries */
	__le32 sw_retry_failure;

	/* illegal rate phy errors  */
	__le32 illgl_rate_phy_err;

	/* wal pdev continuous xretry */
	__le32 pdev_cont_xretry;

	/* wal pdev continuous xretry */
	__le32 pdev_tx_timeout;

	/* wal pdev resets  */
	__le32 pdev_resets;
	__le32 stateless_tid_alloc_failure;
	__le32 phy_underrun;
	__le32 txop_ovf;
	__le32 mc_drop;
} __packed;

struct htt_10_2_wal_rx_stats {
	/* Cnts any change in ring routing mid-ppdu */
	__le32 mid_ppdu_route_change;

	/* Total number of statuses processed */
	__le32 status_rcvd;

	/* Extra frags on rings 0-3 */
	__le32 r0_frags;
	__le32 r1_frags;
	__le32 r2_frags;
	__le32 r3_frags;

	/* MSDUs / MPDUs delivered to HTT */
	__le32 htt_msdus;
	__le32 htt_mpdus;

	/* MSDUs / MPDUs delivered to local stack */
	__le32 loc_msdus;
	__le32 loc_mpdus;

	/* AMSDUs that have more MSDUs than the status ring size */
	__le32 oversize_amsdu;

	/* Number of PHY errors */
	__le32 phy_errs;

	/* Number of PHY errors drops */
	__le32 phy_err_drop;

	/* Number of mpdu errors - FCS, MIC, ENC etc. */
	__le32 mpdu_errs;

	/* wal pdev rx timeout */
	__le32 pdev_rx_timeout;
} __packed;

struct htt_10_2_wal_mem_stats {
	__le32 dram_free_size;
	__le32 sram_free_size;
} __packed;

struct htt_10_2_wal_pdev_txrx {
	struct htt_10_2_wal_tx_stats tx_stats;
	struct htt_10_2_wal_rx_stats rx_stats;
	struct htt_10_2_wal_mem_stats mem_stats;
} __packed;

struct htt_10_2_rx_rate_info {
	__le32 mcs[10];
	__le32 sgi[10];
	__le32 nss[4];
	__le32 nsts;
	__le32 stbc[10];
	__le32 bw[3];
	__le32 pream[6];
	__le32 ldpc;
	__le32 txbf;
	__le32 mgmt_rssi;
	__le32 data_rssi;
	__le32 rssi_chain0;
	__le32 rssi_chain1;
	__le32 rssi_chain2;
} __packed;

struct htt_10_2_tx_rate_info {
	__le32 mcs[10];
	__le32 sgi[10];
	__le32 nss[3];
	__le32 stbc[10];
	__le32 bw[3];
	__le32 pream[4];
	__le32 ldpc;
	__le32 rts_cnt;
	__le32 ack_rssi;
} __packed;

struct wlan_10_2_htt_stats {
	struct htt_10_2_wal_pdev_txrx txrx_stats;
        struct htt_10_2_rx_rate_info rx_rate_info;
        struct htt_10_2_tx_rate_info tx_rate_info;
        struct htt_10_x_tidq_stats tidq;
} __packed;

#endif
