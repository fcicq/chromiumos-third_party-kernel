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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/utsname.h>

#include "core.h"
#include "debug.h"
#include "htt.h"
#include "debug_htt_stats.h"

#define ATH10K_HTT_STATS_BUF_SIZE (1024 * 512)

static int ath10k_htt_10_4_process_stats(struct ath10k *ar, struct sk_buff *skb)
{
	while (true) {
		struct htt_stats_conf_item *conf_item =
				(struct htt_stats_conf_item *)skb->data;

		u16 len;

		if (!skb_pull(skb, sizeof(*conf_item)))
			return -EPROTO;

		len = roundup(le16_to_cpu(conf_item->length), 4);

		if (conf_item->status == HTT_DBG_STATS_STATUS_SERIES_DONE)
			break;

		if (conf_item->status != HTT_DBG_STATS_STATUS_PRESENT &&
		    conf_item->status != HTT_DBG_STATS_STATUS_PARTIAL) {

			if (!skb_pull(skb, len))
				return -EPROTO;

			continue;
		}

		switch (BIT(conf_item->stat_type)) {
		case HTT_DBG_STATS_WAL_PDEV_TXRX: {
			struct htt_10_4_wal_pdev_txrx *txrx_stats =
				(struct htt_10_4_wal_pdev_txrx *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.txrx_stats, txrx_stats, len);
			break;
		}
		case HTT_DBG_STATS_RX_REORDER: {
			struct htt_dbg_stats_rx_reorder_stats *rx_reord_stats =
				(struct htt_dbg_stats_rx_reorder_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.rx_reorder_stats,
			       rx_reord_stats, len);
			break;
		}
		case HTT_DBG_STATS_RX_RATE_INFO: {
			struct htt_10_4_rx_rate_info *rx_rate_info =
				(struct htt_10_4_rx_rate_info *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.rx_rate_info, rx_rate_info, len);
			break;
		}
		case HTT_DBG_STATS_TX_PPDU_LOG:
			if (!skb_pull(skb, len))
				return -EPROTO;

			break;
		case HTT_DBG_STATS_TX_RATE_INFO: {
			struct htt_10_4_tx_rate_info *tx_rate_info =
				(struct htt_10_4_tx_rate_info *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_rate_info, tx_rate_info, len);
			break;
		}
		case HTT_DBG_STATS_TIDQ: {
			struct htt_10_x_tidq_stats *tidq =
				(struct htt_10_x_tidq_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tidq, tidq, len);
			break;
		}
		case HTT_DBG_STATS_TXBF_INFO: {
			struct wlan_10_4_txbf_data_stats *txbf_data_stats =
				(struct wlan_10_4_txbf_data_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.txbf_data_stats,
			       txbf_data_stats, len);
			break;
		}
		case HTT_DBG_STATS_SND_INFO: {
			struct wlan_10_4_txbf_snd_stats *txbf_snd_stats =
				(struct wlan_10_4_txbf_snd_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.txbf_snd_stats, txbf_snd_stats,
			       len);
			break;
		}
		case HTT_DBG_STATS_ERROR_INFO: {
			struct wlan_10_4_wifi2_error_stats *error_stats =
				(struct wlan_10_4_wifi2_error_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.error_stats, error_stats, len);
			break;
		}
		case HTT_DBG_STATS_TX_SELFGEN_INFO: {
			struct wlan_10_4_tx_selfgen_stats *tx_selfgen_stats =
				(struct wlan_10_4_tx_selfgen_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_selfgen_stats,
			       tx_selfgen_stats, len);
			break;
		}
		case HTT_DBG_STATS_TX_MU_INFO: {
			struct wlan_10_4_tx_mu_stats *tx_mu_stats =
				(struct wlan_10_4_tx_mu_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_mu_stats, tx_mu_stats, len);
			break;
		}
		case HTT_DBG_STATS_SIFS_RESP_INFO: {
			struct wlan_10_4_sifs_resp_stats *resp_stats =
				(struct wlan_10_4_sifs_resp_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.resp_stats, resp_stats, len);
			break;
		}
		case HTT_DBG_STATS_RESET_INFO: {
			struct wlan_10_4_reset_stats *reset_stats =
				(struct wlan_10_4_reset_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.reset_stats, reset_stats, len);
			break;
		}
		case HTT_DBG_STATS_MAC_WDOG_INFO: {
			struct wlan_10_4_mac_wdog_stats *wdog_stats =
				(struct wlan_10_4_mac_wdog_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.wdog_stats, wdog_stats, len);
			break;
		}
		case HTT_DBG_STATS_TX_DESC_INFO: {
			struct wlan_10_4_dbg_tx_desc_stats *tx_desc_stats =
				(struct wlan_10_4_dbg_tx_desc_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_desc_stats, tx_desc_stats, len);
			break;
		}
		case HTT_DBG_STATS_TX_FETCH_MGR_INFO: {
			struct wlan_10_4_dbg_tx_fetch_mgr_stats *tx_fetch_mgr_stats =
				(struct wlan_10_4_dbg_tx_fetch_mgr_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_fetch_mgr_stats, tx_fetch_mgr_stats, len);
			break;
		}
		case HTT_DBG_STATS_TX_PFSCHED_INFO: {
			struct wlan_10_4_dbg_tx_pf_sched_stats *tx_pf_sched_stats =
				(struct wlan_10_4_dbg_tx_pf_sched_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_4.tx_pf_sched_stats, tx_pf_sched_stats, len);
			break;
		}
		default:
			if (!skb_pull(skb, len))
				return -EPROTO;

			ath10k_warn(ar, "HTT stats (%d) not handled\n",
				    conf_item->stat_type);
		}
	}
	return 0;
}

static int ath10k_htt_10_2_process_stats(struct ath10k *ar, struct sk_buff *skb)
{
	while (true) {
		struct htt_stats_conf_item *conf_item =
				(struct htt_stats_conf_item *)skb->data;

		u16 len;

		if (!skb_pull(skb, sizeof(*conf_item)))
			return -EPROTO;

		len = roundup(le16_to_cpu(conf_item->length), 4);

		if (conf_item->status == HTT_DBG_STATS_STATUS_SERIES_DONE)
			break;

		if (conf_item->status != HTT_DBG_STATS_STATUS_PRESENT &&
		    conf_item->status != HTT_DBG_STATS_STATUS_PARTIAL) {

			if (!skb_pull(skb, len))
				return -EPROTO;

			continue;
		}

		switch (BIT(conf_item->stat_type)) {
		case HTT_DBG_STATS_WAL_PDEV_TXRX: {
			struct htt_10_2_wal_pdev_txrx *txrx_stats =
				(struct htt_10_2_wal_pdev_txrx *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_2.txrx_stats, txrx_stats, len);
			break;
		}
		case HTT_DBG_STATS_RX_RATE_INFO: {
			struct htt_10_2_rx_rate_info *rx_rate_info =
				(struct htt_10_2_rx_rate_info *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_2.rx_rate_info, rx_rate_info, len);
			break;
		}
		case HTT_DBG_STATS_TX_RATE_INFO: {
			struct htt_10_2_tx_rate_info *tx_rate_info =
				(struct htt_10_2_tx_rate_info *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_2.tx_rate_info, tx_rate_info, len);
			break;
		}
		case HTT_DBG_STATS_TIDQ: {
			struct htt_10_x_tidq_stats *tidq =
				(struct htt_10_x_tidq_stats *)skb->data;

			if (!skb_pull(skb, len))
				return -EPROTO;

			memcpy(&ar->debug.htt_10_2.tidq, tidq, len);
			break;
		}
		default:
			if (!skb_pull(skb, len))
				return -EPROTO;

			ath10k_warn(ar, "HTT stats (%d) not handled\n",
				    conf_item->stat_type);
		}
	}
	return 0;
}

int ath10k_htt_process_stats(struct ath10k *ar, struct sk_buff *skb)
{
	struct htt_stats_conf *conf;
	u32 cookie_lsb;
	u32 cookie_msb;
	int ret = 0;

	skb_pull(skb, sizeof(struct htt_resp_hdr));

	conf = (struct htt_stats_conf *)skb->data;
	cookie_lsb = __le32_to_cpu(conf->cookie_lsb);
	cookie_msb = __le32_to_cpu(conf->cookie_msb);

	if (ar->debug.htt_req_cookie != (cookie_lsb | (u64)cookie_msb << 32)) {
		ath10k_warn(ar, "Request and Response are not same\n");
		ret = -EPROTO;
		goto exit;
	}

	if (!skb_pull(skb, sizeof(*conf))) {
		ret = -EPROTO;
		goto exit;
	}

	if (ar->wmi.op_version <
	    ATH10K_FW_WMI_OP_VERSION_10_4)
		ret = ath10k_htt_10_2_process_stats(ar, skb);
	else
		ret = ath10k_htt_10_4_process_stats(ar, skb);
exit:
	return ret;
}

static void ath10k_htt_10_4_txrx_debug_stats_fill(struct htt_10_4_wal_pdev_txrx *stats,
						  char *buf, int *length)
{
	unsigned int len = *length;
	struct htt_10_4_wal_tx_stats *tx_stats = &stats->tx_stats;
	struct htt_10_4_wal_rx_stats *rx_stats = &stats->rx_stats;
	struct htt_10_4_wal_mem_stats *mem_stats = &stats->mem_stats;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k HTT TX stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "=================");

	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HTT cookies queued",
			 le32_to_cpu(tx_stats->comp_queued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HTT cookies disp.",
			 le32_to_cpu(tx_stats->comp_delivered));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDU queued", le32_to_cpu(tx_stats->msdu_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU queued", le32_to_cpu(tx_stats->mpdu_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs dropped", le32_to_cpu(tx_stats->wmm_drop));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Local enqued", le32_to_cpu(tx_stats->local_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Local freed", le32_to_cpu(tx_stats->local_freed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW queued", le32_to_cpu(tx_stats->hw_queued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PPDUs reaped", le32_to_cpu(tx_stats->hw_reaped));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Num underruns", le32_to_cpu(tx_stats->underrun));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW paused", le32_to_cpu(tx_stats->hw_paused));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PPDUs cleaned", le32_to_cpu(tx_stats->tx_abort));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs requed", le32_to_cpu(tx_stats->mpdus_requed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Excessive retries", le32_to_cpu(tx_stats->tx_ko));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW rate", le32_to_cpu(tx_stats->data_rc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Sched self tiggers",
			 le32_to_cpu(tx_stats->self_triggers));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Dropped due to SW retries",
			 le32_to_cpu(tx_stats->sw_retry_failure));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Illegal rate phy errors",
			 le32_to_cpu(tx_stats->illgl_rate_phy_err));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Pdev continuous xretry",
			 le32_to_cpu(tx_stats->pdev_cont_xretry));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "TX timeout",
			 le32_to_cpu(tx_stats->pdev_tx_timeout));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PDEV resets",
			 le32_to_cpu(tx_stats->pdev_resets));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY underrun",
			 le32_to_cpu(tx_stats->phy_underrun));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU is more than txop limit",
			 le32_to_cpu(tx_stats->txop_ovf));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Seqs posted",
			 le32_to_cpu(tx_stats->seq_posted));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Seqs failed queueing",
			 le32_to_cpu(tx_stats->seq_failed_queueing));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Seqs completed",
			 le32_to_cpu(tx_stats->seq_completed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Seqs restarted",
			 le32_to_cpu(tx_stats->seq_restarted));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MU Seqs posted",
			 le32_to_cpu(tx_stats->mu_seq_posted));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs SW flushed",
			 le32_to_cpu(tx_stats->mpdus_sw_flush));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs HW filtered",
			 le32_to_cpu(tx_stats->mpdus_hw_filter));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs truncated",
			 le32_to_cpu(tx_stats->mpdus_truncated));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs receive no ACK",
			 le32_to_cpu(tx_stats->mpdus_ack_failed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs expired",
			 le32_to_cpu(tx_stats->mpdus_expired));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MC drop",
			 le32_to_cpu(tx_stats->mc_drop));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k HTT RX stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "=================");

	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Mid PPDU route change",
			 le32_to_cpu(rx_stats->mid_ppdu_route_change));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Tot. number of statuses",
			 le32_to_cpu(rx_stats->status_rcvd));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 0",
			 le32_to_cpu(rx_stats->r0_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 1",
			 le32_to_cpu(rx_stats->r1_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 2",
			 le32_to_cpu(rx_stats->r2_frags));
	/*len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 3",
			 le32_to_cpu(rx_stats->r3_frags));*/
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs delivered to HTT",
			 le32_to_cpu(rx_stats->htt_msdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs delivered to HTT",
			 le32_to_cpu(rx_stats->htt_mpdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs delivered to stack",
			 le32_to_cpu(rx_stats->loc_msdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs delivered to stack",
			 le32_to_cpu(rx_stats->loc_mpdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Oversized AMSUs",
			 le32_to_cpu(rx_stats->oversize_amsdu));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY errors", le32_to_cpu(rx_stats->phy_errs));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY errors drops",
			 le32_to_cpu(rx_stats->phy_err_drop));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU errors (FCS, MIC, ENC)",
			 le32_to_cpu(rx_stats->mpdu_errs));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PDEV Rx Timeout",
			 le32_to_cpu(rx_stats->pdev_rx_timeout));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Rx Overflow errors",
			 le32_to_cpu(rx_stats->rx_ovfl_errs));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k HTT Memory stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "=================");

	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "I-RAM free size",
			 le32_to_cpu(mem_stats->iram_free_size));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "D-RAM free size",
			 le32_to_cpu(mem_stats->dram_free_size));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "S-RAM free size",
			 le32_to_cpu(mem_stats->sram_free_size));

	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_rx_reord_stats_fill(struct htt_dbg_stats_rx_reorder_stats *stats,
						char *buf, int *length)
{
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k RX_REORDER stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "Non QoS MPDUs received",
			 le32_to_cpu(stats->deliver_non_qos));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "MPDUs received in-order",
			 le32_to_cpu(stats->deliver_in_order));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "Flush due to reorder timer expired",
			 le32_to_cpu(stats->deliver_flush_timeout));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "Flush out of window",
			 le32_to_cpu(stats->deliver_flush_oow));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "deliver_flush_delba",
			 le32_to_cpu(stats->deliver_flush_delba));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "fcs_error", le32_to_cpu(stats->fcs_error));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "mgmt_ctrl", le32_to_cpu(stats->mgmt_ctrl));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "invalid_peer",
			 le32_to_cpu(stats->invalid_peer));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "dup_non_aggr", le32_to_cpu(stats->dup_non_aggr));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "dup_past", le32_to_cpu(stats->dup_past));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "dup_in_reorder",
			 le32_to_cpu(stats->dup_in_reorder));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			 "reorder_timeout",
			 le32_to_cpu(stats->reorder_timeout));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n",
			  "invalid bar ssn",
			 le32_to_cpu(stats->invalid_bar_ssn));
	len += scnprintf(buf + len, buf_len - len, "%40s %10d\n\n",
			 "ssn reset", le32_to_cpu(stats->ssn_reset));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_rx_rate_info_stats_fill(struct htt_10_4_rx_rate_info *stats,
						    char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k RX_RATE_INFO stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "MCS counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->mcs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->mcs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "SGI counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->sgi); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->sgi[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "STBC counts (0..9) : ");

	for (i = 0; i < ARRAY_SIZE(stats->stbc); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->stbc[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "NSS counts : ");

	for (i = 1; i <= ARRAY_SIZE(stats->nss); i++)
		len += scnprintf(buf + len, buf_len - len, "%dx%d\t%-7d ",
				 i, i, le32_to_cpu(stats->nss[i - 1]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n\n",
			 "NSTS counts : ", le32_to_cpu(stats->nsts));

	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "BW counts (0..3) : ");

	for (i = 0; i < ARRAY_SIZE(stats->bw); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->bw[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "Preamble counts (0..5) : ");

	for (i = 0; i < ARRAY_SIZE(stats->pream); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->pream[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n",
			 "LDPC counts : ", le32_to_cpu(stats->ldpc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n\n",
			 "TXBF counts : ", le32_to_cpu(stats->txbf));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\t%10d\n\n",
			 "RSSI (data, mgmt) : ", stats->data_rssi,
			 stats->mgmt_rssi);
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 0 : ",
			 ((le32_to_cpu(stats->rssi_chain0) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 0) & 0xff));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 1 : ",
			 ((le32_to_cpu(stats->rssi_chain1) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 0) & 0xff));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 2 : ",
			 ((le32_to_cpu(stats->rssi_chain2) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 0) & 0xff));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 3 : ",
			 ((le32_to_cpu(stats->rssi_chain3) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain3) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain3) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain3) >> 0) & 0xff));
	 len += scnprintf(buf + len, buf_len - len, "%30s %10d\n\n",
			 "RSSI (comb_ht) ", stats->rssi_comb_ht);

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_tx_rate_info_stats_fill(struct htt_10_4_tx_rate_info *stats,
						    char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k TX_RATE_INFO stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "MCS counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->mcs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->mcs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "SGI counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->sgi); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->sgi[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "STBC counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->stbc); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->stbc[i]));

	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "MU MCS counts (0..9): ");

	for (i = 0;i < ARRAY_SIZE(stats->mu_mcs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->mu_mcs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "NSS counts : ");

	for (i = 1; i <= ARRAY_SIZE(stats->nss); i++)
		len += scnprintf(buf + len, buf_len - len, "%dx%d\t%-7d ",
				 i, i, le32_to_cpu(stats->nss[i - 1]));
	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "BW counts (0..3) : ");

	for (i = 0; i < ARRAY_SIZE(stats->bw); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->bw[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "Preamble counts (0..3): ");

	for (i = 0; i < ARRAY_SIZE(stats->pream); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->pream[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n",
			 "LDPC counts : ", le32_to_cpu(stats->ldpc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "RTS counts : ", le32_to_cpu(stats->rts_cnt));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n\n",
			 "Ack RSSI : ", le32_to_cpu(stats->ack_rssi));

	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_tidq_stats_fill(struct htt_10_x_tidq_stats *stats, char *buf,
					    int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	if (le32_to_cpu(stats->wlan_dbg_tid_txq_status) == 1)
		len += scnprintf(buf + len, buf_len - len, "\n%40s\n",
				 "Could not read TIDQ stats from firmware");
	else {
		len += scnprintf(buf + len, buf_len - len, "\n");
		len += scnprintf(buf + len, buf_len - len, "%30s\n",
				 "ath10k TID QUEUE STATS PER H/W Q");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
					"=================");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
					"Frames queued to h/w Q");

		for (i = 0; i < DBG_STATS_MAX_HWQ_NUM; i++)
			len += scnprintf(buf + len, buf_len - len, "\tQ%d", i);

		len += scnprintf(buf + len, buf_len - len, "\n");

		for (i = 0; i < DBG_STATS_MAX_HWQ_NUM; i++)
			len += scnprintf(buf + len, buf_len - len, "\t%-3d",
					 le16_to_cpu(stats->txq_st.num_pkts_queued[i]));

		len += scnprintf(buf + len, buf_len - len, "\n");
		len += scnprintf(buf + len, buf_len - len, "%30s",
				 "TID Queue stats");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "S/W queue stats <---> H/W queue stats");

		len += scnprintf(buf + len, buf_len - len, "%40s\n\n",
				 "------------------------------");

		for (i = 0; i < DBG_STATS_MAX_TID_NUM; i++)
			len += scnprintf(buf + len, buf_len - len,
					 "%20s\t%3d\t%3d\t\t%3d\n",
					 "TID", i,
					 le16_to_cpu(stats->txq_st.tid_sw_qdepth[i]),
					 le16_to_cpu(stats->txq_st.tid_hw_qdepth[i]));

		len += scnprintf(buf + len, buf_len - len, "%40s\n\n",
				 "------------------------------");
	}

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_txbf_stats_fill(struct wlan_10_4_txbf_data_stats *stats,
					    char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "TXBF Data Info");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");

	len += scnprintf(buf + len, buf_len - len, "%30s\t",
			 "VHT Tx TxBF counts(0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_vht); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_vht[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "VHT Rx TxBF counts(0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->rx_txbf_vht); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->rx_txbf_vht[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "HT Tx TxBF counts(0..7): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_ht); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_ht[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "OFDM Tx TxBF counts(0..7): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_ofdm); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_ofdm[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "IBF VHT Tx TxBF counts(0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_ibf_vht); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_ibf_vht[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "IBF HT Tx TxBF counts(0..7): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_ibf_ht); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_ibf_ht[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "IBF OFDM Tx TxBF counts(0..7): ");

	for (i = 0; i < ARRAY_SIZE(stats->tx_txbf_ibf_ofdm); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d  ",
				 le32_to_cpu(stats->tx_txbf_ibf_ofdm[i]));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_txbf_snd_stats_fill(struct wlan_10_4_txbf_snd_stats *stats,
						char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "TXBF SEND Info");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "TXBF Sounding Info :");
	len += scnprintf(buf + len, buf_len - len,
			 "%30s%8s%3d%8s%3d%8s%3d%8s%3d\n",
			 "Sounding User 1 :", "20Mhz",
			 le32_to_cpu(stats->sounding[0]),
			 "40Mhz", le32_to_cpu(stats->sounding[1]),
			 "80Mhz", le32_to_cpu(stats->sounding[2]),
			 "160Mhz", le32_to_cpu(stats->sounding[3]));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s%8s%3d%8s%3d%8s%3d%8s%3d\n",
			 "Sounding User 2 :", "20Mhz",
			 le32_to_cpu(stats->sounding[4]),
			 "40Mhz", le32_to_cpu(stats->sounding[5]),
			 "80Mhz", le32_to_cpu(stats->sounding[6]),
			 "160Mhz", le32_to_cpu(stats->sounding[7]));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s%8s%3d%8s%3d%8s%3d%8s%3d\n",
			 "Sounding User 3 :", "20Mhz",
			 le32_to_cpu(stats->sounding[8]),
			 "40Mhz", le32_to_cpu(stats->sounding[9]),
			 "80Mhz", le32_to_cpu(stats->sounding[10]),
			 "160Mhz", le32_to_cpu(stats->sounding[11]));
	len += scnprintf(buf + len, buf_len - len, "\n%30s\t",
			  "CBF 20 :");

	for (i = 0; i < ARRAY_SIZE(stats->cbf_20); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d ",
				 le32_to_cpu(stats->cbf_20[i]));

	len += scnprintf(buf + len, buf_len - len, "\n%30s\t",
			  "CBF 40 :");

	for (i = 0; i < ARRAY_SIZE(stats->cbf_40); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d ",
				 le32_to_cpu(stats->cbf_40[i]));

	len += scnprintf(buf + len, buf_len - len, "\n%30s\t",
			 "CBF 80 :");

	for (i = 0; i < ARRAY_SIZE(stats->cbf_80); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d ",
				 le32_to_cpu(stats->cbf_80[i]));

	len += scnprintf(buf + len, buf_len - len, "\n%30s\t",
			 "CBF 160 :");

	for (i = 0; i < ARRAY_SIZE(stats->cbf_160); i++)
		len += scnprintf(buf + len, buf_len - len, "%3d ",
				 le32_to_cpu(stats->cbf_160[i]));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "\n");
	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_error_stats_fill(struct wlan_10_4_wifi2_error_stats *stats,
					     char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "Wifi error stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s\t",
			 "HWSCH Error  (0..3):");

	for (i = 0; i < ARRAY_SIZE(stats->schd_stall_errs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->schd_stall_errs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "SchCmdResult (0..7):");

	for (i = 0; i < ARRAY_SIZE(stats->schd_cmd_result); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->schd_cmd_result[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "SIFS Status (0..7):");

	for (i = 0; i < ARRAY_SIZE(stats->sifs_status); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->sifs_status[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "URRN_stats Error  (0..20):");

	for (i = 0; i < ARRAY_SIZE(stats->urrn_stats); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->urrn_stats[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "Flush Error  (0..17):");

	for (i = 0; i < ARRAY_SIZE(stats->flush_errs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->flush_errs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\t",
			 "Phy Error  (0..20):");

	for (i = 0; i < ARRAY_SIZE(stats->phy_errs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 stats->phy_errs[i]);

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n",
			 "rx_rate_inval : ", le32_to_cpu(stats->rx_rate_inval));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_tx_selfgen_stats_fill(struct wlan_10_4_tx_selfgen_stats *stats,
						  char *buf, int *length)
{
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "TX_SELFGEN Info");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_ndpa :", le32_to_cpu(stats->su_ndpa));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_ndp  :", le32_to_cpu(stats->su_ndp));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_ndpa  :", le32_to_cpu(stats->mu_ndpa));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_ndp  :", le32_to_cpu(stats->mu_ndp));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_brpoll_1  :", le32_to_cpu(stats->mu_brpoll_1));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_brpoll_2  :", le32_to_cpu(stats->mu_brpoll_2));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_bar  :", le32_to_cpu(stats->su_bar));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_bar_1  :", le32_to_cpu(stats->mu_bar_1));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_bar_2  :", le32_to_cpu(stats->mu_bar_2));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_cts  :", le32_to_cpu(stats->su_cts));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_cts  :", le32_to_cpu(stats->mu_cts));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_ndpa_err  :", le32_to_cpu(stats->su_ndpa_err));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_ndpa_err  :", le32_to_cpu(stats->mu_ndpa_err));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "su_ndp_err  :", le32_to_cpu(stats->su_ndp_err));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_ndp_err  :", le32_to_cpu(stats->mu_ndp_err));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_brp1_err  :", le32_to_cpu(stats->mu_brp1_err));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "mu_brp2_err  :", le32_to_cpu(stats->mu_brp2_err));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_tx_mu_stats_fill(struct wlan_10_4_tx_mu_stats *stats,
					     char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "TX_MU Info :");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_sch_nusers_2      :",
			 le32_to_cpu(stats->mu_sch_nusers_2));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "mu_sch_nusers_3      :",
			 le32_to_cpu(stats->mu_sch_nusers_3));

	for (i = 0; i < ARRAY_SIZE(stats->mu_mpdus_queued_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_mpdus_queued_usr", i,
			 le32_to_cpu(stats->mu_mpdus_queued_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_mpdus_tried_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_mpdus_tried_usr", i,
			 le32_to_cpu(stats->mu_mpdus_tried_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_mpdus_failed_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_mpdus_failed_usr", i,
			 le32_to_cpu(stats->mu_mpdus_failed_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_mpdus_requeued_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_mpdus_requeued_usr", i,
			 le32_to_cpu(stats->mu_mpdus_requeued_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_err_no_ba_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_err_no_ba_usr", i,
			 le32_to_cpu(stats->mu_err_no_ba_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_mpdu_underrun_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
			 "mu_mpdu_underrun_usr", i,
			 le32_to_cpu(stats->mu_mpdu_underrun_usr[i]));

	for (i = 0; i < ARRAY_SIZE(stats->mu_ampdu_underrun_usr); i++)
		len += scnprintf(buf + len, buf_len - len, "%30s%d  :%3d\n",
				 "mu_ampdu_underrun_usr", i,
				 le32_to_cpu(stats->mu_ampdu_underrun_usr[i]));

	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_resp_stats_fill(struct wlan_10_4_sifs_resp_stats *stats,
					    char *buf, int *length)
{
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "SIFS RESP RX stats :");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "ps_poll_trigger  :",
			 le32_to_cpu(stats->ps_poll_trigger));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "uapsd_trigger  :",
			 le32_to_cpu(stats->uapsd_trigger));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "qboost trigger data[exp]  :",
			 le32_to_cpu(stats->qb_data_trigger[0]));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "qboost trigger bar[exp]  :",
			 le32_to_cpu(stats->qb_bar_trigger[0]));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "qboost trigger data[imp]  :",
			 le32_to_cpu(stats->qb_data_trigger[1]));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "qboost trigger bar[imp]  :",
			 le32_to_cpu(stats->qb_bar_trigger[1]));
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
			 "SIFS RESP TX stats");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "SIFS response data  :",
			 le32_to_cpu(stats->sifs_resp_data));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "SIFS response timing err  :",
			 le32_to_cpu(stats->sifs_resp_err));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_reset_stats_fill(struct wlan_10_4_reset_stats *stats,
					     char *buf, int *length)
{
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "RESET stats :");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "warm_reset  :", le16_to_cpu(stats->warm_reset));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "cold_reset  :", le16_to_cpu(stats->cold_reset));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "tx_flush  :", le16_to_cpu(stats->tx_flush));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "tx_glb_reset  :", le16_to_cpu(stats->tx_glb_reset));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "tx_txq_reset  :", le16_to_cpu(stats->tx_txq_reset));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "rx_timeout_reset  :",
			 le16_to_cpu(stats->rx_timeout_reset));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "hw_status_mismatch  :",
			 le16_to_cpu(stats->hw_status_mismatch));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "hw_status_multi_mismatch  :",
			 (stats->hw_status_multi_mismatch));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_mac_wdog_stats_fill(struct wlan_10_4_mac_wdog_stats *stats,
						char *buf, int *length)
{
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "MAC WDOG timeout stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "rxpcu  :", le16_to_cpu(stats->rxpcu));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "txpcu  :", le16_to_cpu(stats->txpcu));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "ole  :", le16_to_cpu(stats->ole));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "rxdma  :", le16_to_cpu(stats->rxdma));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "hwsch  :", le16_to_cpu(stats->hwsch));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "crypto  :", le16_to_cpu(stats->crypto));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "pdg  :", le16_to_cpu(stats->pdg));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "txdma  :", le16_to_cpu(stats->txdma));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

void ath10k_htt_10_4_tx_desc_info_fill(struct wlan_10_4_dbg_tx_desc_stats *stats,
				       char *buf, int *length)
{
	unsigned int i;
	unsigned int *word = NULL;
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n%30s\n",
			 "FW Descriptor Monitor Stats\n");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "Total FW Desc count : ",
			 WLAN_DBG_TX_DESC_CFG_TOTAL_GET(le32_to_cpu(stats->word1)));
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
			 "Current Desc Available : ",
			 WLAN_DBG_TX_DESC_TOTAL_AVAIL_GET(le32_to_cpu(stats->word1)));
	for (i = 0; i < WLAN_TX_DESC_MAX_BINS; i++) {
		word = (unsigned int *)le32_to_cpu(&stats->bin_stats[i]);

		len += scnprintf(buf + len, buf_len - len, "%30s\n",
				 "==================\n");
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "BIN id : ",
				 WLAN_DBG_TX_DESC_BIN_IDX_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Min desc : ",
				 WLAN_DBG_TX_DESC_CFG_MIN_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Max desc : ",
				 WLAN_DBG_TX_DESC_CFG_MAX_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Priority : ",
				 WLAN_DBG_TX_DESC_CFG_PRIO_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Hysteresis threshold : ",
				 WLAN_DBG_TX_DESC_CFG_BIN_HYST_THR_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Desc consumed : ",
				 WLAN_DBG_TX_DESC_CURR_TOT_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Pre-alloc count : ",
				 WLAN_DBG_TX_DESC_PREALLOC_CNT_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Max Desc consumed : ",
				 WLAN_DBG_TX_DESC_BIN_MAX_GET(word));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Low threshold cnt : ",
				 stats->bin_stats[i].bin_hist_low);
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "High threshold cnt : ",
				 stats->bin_stats[i].bin_hist_high);
	}

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

void ath10k_htt_10_4_tx_fetch_mgr_info_fill(struct wlan_10_4_dbg_tx_fetch_mgr_stats *stats,
					    char *buf, int *length)
{
	unsigned int i;
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s\n",
			 "Fetch Manager Stats\n");
	len += scnprintf(buf + len, buf_len - len, "%30s%3d\n\n",
			 "Total Outstanding Fetch desc : ",
			 le32_to_cpu(stats->fetch_mgr_total_outstanding_fetch_desc));
	for (i = 0; i < WAL_STATS_PREFETCH_MAX_QUEUES; i++) {
		len += scnprintf(buf + len, buf_len,
				 "Outstanding Fetch Duration/AC : [%u] Outstanding Fetch Desc/AC : [%u]\n",
				 WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DUR_GET(le32_to_cpu(stats->fetch_desc__fetch_dur[i])),
				 WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DESC_GET(le32_to_cpu(stats->fetch_desc__fetch_dur[i])));
	}

	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "\n -- FETCH HIST 500 USEC BIN -- \n");
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "   0 USEC -  500 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[0]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 " 500 USEC - 1000 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[1]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "1000 USEC - 1500 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[2]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "1500 USEC - 2000 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[3]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "2000 USEC - 2500 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[4]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "2500 USEC - 3000 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[5]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "3000 USEC - 3500 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[6]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "3500 USEC - 4000 USEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_500us[7]));
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "\n -- FETCH HIST 4 MSEC BIN -- \n");
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 " 0 MSEC -  4 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[0]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 " 4 MSEC -  8 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[1]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 " 8 MSEC - 12 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[2]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "12 MSEC - 16 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[3]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "16 MSEC - 20 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[4]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "20 MSEC - 24 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[5]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "24 MSEC - 28 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[6]));
	len += scnprintf(buf + len, buf_len - len, "%30s%10d\n",
			 "28 MSEC - 32 MSEC : ",
			 le32_to_cpu(stats->fetch_mgr_rtt_histogram_4ms[7]));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

void ath10k_htt_10_4_tx_pfsched_info_fill(struct wlan_10_4_dbg_tx_pf_sched_stats *stats,
					  char *buf, int *length)
{
	unsigned int i;
	unsigned int len = *length;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			"\n Pre-Fetch Manager Stats\n");

	for (i = 0; i < WAL_STATS_PREFETCH_MAX_QUEUES; i++) {
		len += scnprintf(buf + len, buf_len - len,
				 "\n========== AC  [ %u ] ==========\n", i);
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Tx Queued : ",
				 le32_to_cpu(stats->tx_queued[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Tx Reaped : ",
				 le32_to_cpu(stats->tx_reaped[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Tx Sched : ",
				 le32_to_cpu(stats->tx_sched[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Tx ReQueued : ",
				 le32_to_cpu(stats->tx_requeued[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Abort Sched : ",
				 le32_to_cpu(stats->abort_sched[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Sched Fail : ",
				 le32_to_cpu(stats->sched_fail[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Sched Timeout : ",
				 le32_to_cpu(stats->sched_timeout[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Sched WaitQ : ",
				 le32_to_cpu(stats->tx_sched_waitq[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Fetch Request : ",
				 le32_to_cpu(stats->fetch_request[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Fetch Response : ",
				 le32_to_cpu(stats->fetch_resp[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Fetch Response Invalid : ",
				 le32_to_cpu(stats->fetch_resp_invld[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "Fetch Response Delayed : ",
				 le32_to_cpu(stats->fetch_resp_delayed[i]));
		len += scnprintf(buf + len, buf_len - len, "%30s%3d\n",
				 "PFsched Peer Skipped :",
				 le32_to_cpu(stats->pfsched_peer_skipped[i]));
	}

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_4_stats_fill(struct ath10k *ar, char *buf,
				       int *length)
{
	unsigned long htt_stats_mask;
	u32 bit;
	mutex_lock(&ar->conf_mutex);
	htt_stats_mask = ar->debug.htt_stats_mask;

	spin_lock_bh(&ar->data_lock);
	for_each_set_bit(bit, &htt_stats_mask,
			 sizeof(htt_stats_mask) * BITS_PER_BYTE) {
		switch (BIT(bit)) {
		case HTT_DBG_STATS_WAL_PDEV_TXRX:
			ath10k_htt_10_4_txrx_debug_stats_fill(&ar->debug.htt_10_4.txrx_stats,
							      buf, length);
			break;
		case HTT_DBG_STATS_RX_REORDER:
			ath10k_htt_10_4_rx_reord_stats_fill(&ar->debug.htt_10_4.rx_reorder_stats,
							    buf, length);
			break;
		case HTT_DBG_STATS_RX_RATE_INFO:
			ath10k_htt_10_4_rx_rate_info_stats_fill(&ar->debug.htt_10_4.rx_rate_info,
								buf, length);
			break;
		case HTT_DBG_STATS_TX_PPDU_LOG:
			break;
		case HTT_DBG_STATS_TX_RATE_INFO:
			ath10k_htt_10_4_tx_rate_info_stats_fill(&ar->debug.htt_10_4.tx_rate_info,
								buf, length);
			break;
		case HTT_DBG_STATS_TIDQ:
			ath10k_htt_10_4_tidq_stats_fill(&ar->debug.htt_10_4.tidq, buf,
							length);
			break;
		case HTT_DBG_STATS_TXBF_INFO:
			ath10k_htt_10_4_txbf_stats_fill(&ar->debug.htt_10_4.txbf_data_stats,
							buf, length);
			break;
		case HTT_DBG_STATS_SND_INFO:
			ath10k_htt_10_4_txbf_snd_stats_fill(&ar->debug.htt_10_4.txbf_snd_stats,
							    buf, length);
			break;
		case HTT_DBG_STATS_ERROR_INFO:
			ath10k_htt_10_4_error_stats_fill(&ar->debug.htt_10_4.error_stats,
							 buf, length);
			break;
		case HTT_DBG_STATS_TX_SELFGEN_INFO:
			ath10k_htt_10_4_tx_selfgen_stats_fill(&ar->debug.htt_10_4.tx_selfgen_stats,
							      buf, length);
			break;
		case HTT_DBG_STATS_TX_MU_INFO:
			ath10k_htt_10_4_tx_mu_stats_fill(&ar->debug.htt_10_4.tx_mu_stats,
							 buf, length);
			break;
		case HTT_DBG_STATS_SIFS_RESP_INFO:
			ath10k_htt_10_4_resp_stats_fill(&ar->debug.htt_10_4.resp_stats,
							buf, length);
			break;
		case HTT_DBG_STATS_RESET_INFO:
			ath10k_htt_10_4_reset_stats_fill(&ar->debug.htt_10_4.reset_stats,
							 buf, length);
			break;
		case HTT_DBG_STATS_MAC_WDOG_INFO:
			ath10k_htt_10_4_mac_wdog_stats_fill(&ar->debug.htt_10_4.wdog_stats,
							    buf, length);
			break;
		case HTT_DBG_STATS_TX_DESC_INFO:
			ath10k_htt_10_4_tx_desc_info_fill(&ar->debug.htt_10_4.tx_desc_stats,
							  buf, length);
			break;
		case HTT_DBG_STATS_TX_FETCH_MGR_INFO:
			ath10k_htt_10_4_tx_fetch_mgr_info_fill(&ar->debug.htt_10_4.tx_fetch_mgr_stats,
							       buf, length);
			break;
		case HTT_DBG_STATS_TX_PFSCHED_INFO:
			ath10k_htt_10_4_tx_pfsched_info_fill(&ar->debug.htt_10_4.tx_pf_sched_stats,
							     buf, length);
			break;
		default:
			ath10k_warn(ar, "HTT stats (%lu) not handled\n",
				    BIT(bit));
		}
	}
	spin_unlock_bh(&ar->data_lock);
	mutex_unlock(&ar->conf_mutex);
}

static void ath10k_htt_10_2_txrx_debug_stats_fill(struct htt_10_2_wal_pdev_txrx *stats,
						  char *buf, int *length)
{
	unsigned int len = *length;
	struct htt_10_2_wal_tx_stats *tx_stats = &stats->tx_stats;
	struct htt_10_2_wal_rx_stats *rx_stats = &stats->rx_stats;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k HTT TX stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "=================");

	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HTT cookies queued",
			 le32_to_cpu(tx_stats->comp_queued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HTT cookies disp.",
			 le32_to_cpu(tx_stats->comp_delivered));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDU queued", le32_to_cpu(tx_stats->msdu_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU queued", le32_to_cpu(tx_stats->mpdu_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs dropped", le32_to_cpu(tx_stats->wmm_drop));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Local enqued", le32_to_cpu(tx_stats->local_enqued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Local freed", le32_to_cpu(tx_stats->local_freed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW queued", le32_to_cpu(tx_stats->hw_queued));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PPDUs reaped", le32_to_cpu(tx_stats->hw_reaped));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Num underruns", le32_to_cpu(tx_stats->underrun));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW paused", le32_to_cpu(tx_stats->hw_paused));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PPDUs cleaned", le32_to_cpu(tx_stats->tx_abort));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs requed", le32_to_cpu(tx_stats->mpdus_requed));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Excessive retries", le32_to_cpu(tx_stats->tx_ko));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "HW rate", le32_to_cpu(tx_stats->data_rc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Sched self tiggers",
			 le32_to_cpu(tx_stats->self_triggers));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Dropped due to SW retries",
			 le32_to_cpu(tx_stats->sw_retry_failure));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Illegal rate phy errors",
			 le32_to_cpu(tx_stats->illgl_rate_phy_err));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Pdev continuous xretry",
			 le32_to_cpu(tx_stats->pdev_cont_xretry));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "TX timeout",
			 le32_to_cpu(tx_stats->pdev_tx_timeout));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PDEV resets",
			 le32_to_cpu(tx_stats->pdev_resets));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY underrun",
			 le32_to_cpu(tx_stats->phy_underrun));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU is more than txop limit",
			 le32_to_cpu(tx_stats->txop_ovf));

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k HTT RX stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "=================");

	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Mid PPDU route change",
			 le32_to_cpu(rx_stats->mid_ppdu_route_change));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Tot. number of statuses",
			 le32_to_cpu(rx_stats->status_rcvd));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 0",
			 le32_to_cpu(rx_stats->r0_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 1",
			 le32_to_cpu(rx_stats->r1_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 2",
			 le32_to_cpu(rx_stats->r2_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Extra frags on rings 3",
			 le32_to_cpu(rx_stats->r3_frags));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs delivered to HTT",
			 le32_to_cpu(rx_stats->htt_msdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs delivered to HTT",
			 le32_to_cpu(rx_stats->htt_mpdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MSDUs delivered to stack",
			 le32_to_cpu(rx_stats->loc_msdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDUs delivered to stack",
			 le32_to_cpu(rx_stats->loc_mpdus));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "Oversized AMSUs",
			 le32_to_cpu(rx_stats->oversize_amsdu));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY errors", le32_to_cpu(rx_stats->phy_errs));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "PHY errors drops",
			 le32_to_cpu(rx_stats->phy_err_drop));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "MPDU errors (FCS, MIC, ENC)",
			 le32_to_cpu(rx_stats->mpdu_errs));

	len += scnprintf(buf + len, buf_len - len, "\n");

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_2_rx_rate_info_stats_fill(struct htt_10_2_rx_rate_info *stats,
						    char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k RX_RATE_INFO stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "MCS counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->mcs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d  ",
				 le32_to_cpu(stats->mcs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "SGI counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->sgi); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d  ",
				 le32_to_cpu(stats->sgi[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "STBC counts (0..9) : ");

	for (i = 0; i < ARRAY_SIZE(stats->stbc); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d  ",
				 le32_to_cpu(stats->stbc[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "NSS counts : ");

	for (i = 1; i <= ARRAY_SIZE(stats->nss); i++)
		len += scnprintf(buf + len, buf_len - len, "%dx%d\t%-7d  ",
				 i, i, le32_to_cpu(stats->nss[i - 1]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n\n",
			 "NSTS counts : ", le32_to_cpu(stats->nsts));

	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "BW counts (0..2) : ");

	for (i = 0; i < ARRAY_SIZE(stats->bw); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d  ",
				 le32_to_cpu(stats->bw[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "Preamble counts (0..5) : ");

	for (i = 0; i < ARRAY_SIZE(stats->pream); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d  ",
				 le32_to_cpu(stats->pream[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n",
			 "LDPC counts : ", le32_to_cpu(stats->ldpc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n\n",
			 "TXBF counts : ", le32_to_cpu(stats->txbf));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\t%10d\n\n",
			 "RSSI (data, mgmt) : ", stats->data_rssi,
			 stats->mgmt_rssi);
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 0 : ",
			 ((le32_to_cpu(stats->rssi_chain0) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain0) >> 0) & 0xff));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 1 : ",
			 ((le32_to_cpu(stats->rssi_chain1) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain1) >> 0) & 0xff));
	len += scnprintf(buf + len, buf_len - len,
			 "%30s (0x%02x 0x%02x 0x%02x 0x%02x)\n",
			 "RSSI Chain 2 : ",
			 ((le32_to_cpu(stats->rssi_chain2) >> 24) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 16) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 8) & 0xff),
			 ((le32_to_cpu(stats->rssi_chain2) >> 0) & 0xff));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_2_tx_rate_info_stats_fill(struct htt_10_2_tx_rate_info *stats,
						    char *buf, int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len, "%30s\n",
			 "ath10k TX_RATE_INFO stats");
	len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				"=================");
	len += scnprintf(buf + len, buf_len - len, "%30s",
			 "MCS counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->mcs); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->mcs[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "SGI counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->sgi); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->sgi[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "STBC counts (0..9): ");

	for (i = 0; i < ARRAY_SIZE(stats->stbc); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->stbc[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "NSS counts : ");

	for (i = 1; i <= ARRAY_SIZE(stats->nss); i++)
		len += scnprintf(buf + len, buf_len - len, "%dx%d\t%-7d ",
				 i, i, le32_to_cpu(stats->nss[i - 1]));
	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "BW counts (0..2) : ");

	for (i = 0; i < ARRAY_SIZE(stats->bw); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->bw[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s",
			 "Preamble counts (0..3): ");

	for (i = 0; i < ARRAY_SIZE(stats->pream); i++)
		len += scnprintf(buf + len, buf_len - len, "%7d ",
				 le32_to_cpu(stats->pream[i]));

	len += scnprintf(buf + len, buf_len - len, "\n\n%30s %10d\n",
			 "LDPC counts : ", le32_to_cpu(stats->ldpc));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n",
			 "RTS counts : ", le32_to_cpu(stats->rts_cnt));
	len += scnprintf(buf + len, buf_len - len, "%30s %10d\n\n",
			 "Ack RSSI : ", le32_to_cpu(stats->ack_rssi));

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_2_tidq_stats_fill(struct htt_10_x_tidq_stats *stats, char *buf,
					    int *length)
{
	unsigned int len = *length;
	int i;
	unsigned int buf_len = ATH10K_HTT_STATS_BUF_SIZE;

	if (le32_to_cpu(stats->wlan_dbg_tid_txq_status) == 1)
		len += scnprintf(buf + len, buf_len - len, "\n%40s\n",
				 "Could not read TIDQ stats from firmware");
	else {
		len += scnprintf(buf + len, buf_len - len, "\n");
		len += scnprintf(buf + len, buf_len - len, "%30s\n",
				 "ath10k TID QUEUE STATS PER H/W Q");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
					"=================");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
					"Frames queued to h/w Q");

		for (i = 0; i < DBG_STATS_MAX_HWQ_NUM; i++)
			len += scnprintf(buf + len, buf_len - len, "\tQ%d", i);

		len += scnprintf(buf + len, buf_len - len, "\n");

		for (i = 0; i < DBG_STATS_MAX_HWQ_NUM; i++)
			len += scnprintf(buf + len, buf_len - len, "\t%-3d",
					 le16_to_cpu(stats->txq_st.num_pkts_queued[i]));

		len += scnprintf(buf + len, buf_len - len, "\n");
		len += scnprintf(buf + len, buf_len - len, "%30s",
				 "TID Queue stats");
		len += scnprintf(buf + len, buf_len - len, "%30s\n\n",
				 "S/W queue stats <---> H/W queue stats");

		len += scnprintf(buf + len, buf_len - len, "%40s\n\n",
				 "------------------------------");

		for (i = 0; i < DBG_STATS_MAX_TID_NUM; i++)
			len += scnprintf(buf + len, buf_len - len,
					 "%20s\t%3d\t%3d\t\t%3d\n",
					 "TID", i,
					 le16_to_cpu(stats->txq_st.tid_sw_qdepth[i]),
					 le16_to_cpu(stats->txq_st.tid_hw_qdepth[i]));

		len += scnprintf(buf + len, buf_len - len, "%40s\n\n",
				 "------------------------------");
	}

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;

	*length = len;
}

static void ath10k_htt_10_2_stats_fill(struct ath10k *ar, char *buf,
				       int *length)
{
	unsigned long htt_stats_mask;
	u32 bit;
	mutex_lock(&ar->conf_mutex);
	htt_stats_mask = ar->debug.htt_stats_mask;

	spin_lock_bh(&ar->data_lock);
	for_each_set_bit(bit, &htt_stats_mask,
			 sizeof(htt_stats_mask) * BITS_PER_BYTE) {
		switch (BIT(bit)) {
		case HTT_DBG_STATS_WAL_PDEV_TXRX:
			ath10k_htt_10_2_txrx_debug_stats_fill(&ar->debug.htt_10_2.txrx_stats,
							      buf, length);
			break;
		case HTT_DBG_STATS_RX_RATE_INFO:
			ath10k_htt_10_2_rx_rate_info_stats_fill(&ar->debug.htt_10_2.rx_rate_info,
								buf, length);
			break;
		case HTT_DBG_STATS_TX_RATE_INFO:
			ath10k_htt_10_2_tx_rate_info_stats_fill(&ar->debug.htt_10_2.tx_rate_info,
								buf, length);
			break;
		case HTT_DBG_STATS_TIDQ:
			ath10k_htt_10_2_tidq_stats_fill(&ar->debug.htt_10_2.tidq, buf,
							length);
			break;
		default:
			ath10k_warn(ar, "HTT stats (%lu) not handled\n",
				    BIT(bit));
		}
	}
	spin_unlock_bh(&ar->data_lock);
	mutex_unlock(&ar->conf_mutex);
}

static ssize_t ath10k_read_dump_htt_stats(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char *buf;
	int length = 0;

	buf = vmalloc(ATH10K_HTT_STATS_BUF_SIZE);

	if (!buf)
		return -ENOMEM;

	if (ar->wmi.op_version <
	    ATH10K_FW_WMI_OP_VERSION_10_4)
		ath10k_htt_10_2_stats_fill(ar, buf, &length);
	else
		ath10k_htt_10_4_stats_fill(ar, buf, &length);

	count = simple_read_from_buffer(user_buf, count, ppos, buf, length);
	vfree(buf);
	return count;
}

static const struct file_operations fops_dump_htt_stats = {
	.read = ath10k_read_dump_htt_stats,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void ath10k_htt_debug_stats_init(struct ath10k *ar)
{
	debugfs_create_file("dump_htt_stats", S_IRUSR, ar->debug.debugfs_phy,
			    ar, &fops_dump_htt_stats);
}
