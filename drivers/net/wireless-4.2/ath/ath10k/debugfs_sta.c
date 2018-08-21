/*
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
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

#include "core.h"
#include "wmi-ops.h"
#include "wmi.h"
#include "debug.h"

static char ath10k_map_rate_code_number(u8 rate, u8 pream)
{
	u8 i;
	u8 legacy_rates[] = {TX_CCK_RATE_1_MBPS, TX_CCK_RATE_2_MBPS,
			     TX_CCK_RATE_5_5_MBPS, TX_CCK_RATE_11_MBPS,
			     TX_OFDM_RATE_6_MBPS, TX_OFDM_RATE_9_MBPS,
			     TX_OFDM_RATE_12_MBPS, TX_OFDM_RATE_18_MBPS,
			     TX_OFDM_RATE_24_MBPS, TX_OFDM_RATE_36_MBPS,
			     TX_OFDM_RATE_48_MBPS, TX_OFDM_RATE_54_MBPS};

	/* For CCK 5.5Mbps firmware sends rate as 6 */
	if (pream == WMI_RATE_PREAMBLE_CCK && rate == 6)
		rate = TX_CCK_RATE_5_5_MBPS;

	for (i = 0; i < LEGACY_RATE_NUM; i++) {
		if (rate == legacy_rates[i])
			break;
	}

	return i;
}

static void ath10k_fill_tx_bitrate(struct ieee80211_hw *hw,
				   struct ieee80211_sta *sta,
				   struct rate_info *txrate,
				   u8 rate, u8 sgi, u8 success, u8 failed,
				   u8 retries, bool skip_auto_rate)
{
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ieee80211_chanctx_conf *conf = NULL;
	struct ieee80211_tx_info info;

	memset(&info, 0, sizeof(info));
	info.status.rates[0].count = retries;

	switch (txrate->flags) {
	case WMI_RATE_PREAMBLE_OFDM:
		if (arsta->arvif && arsta->arvif->vif)
			conf = rcu_dereference(arsta->arvif->vif->chanctx_conf);
		if (conf && conf->def.chan->band == IEEE80211_BAND_5GHZ)
			info.status.rates[0].idx = txrate->mcs - 4;
		arsta->tx_stats.txrate.legacy = rate * 10;
		break;
	case WMI_RATE_PREAMBLE_CCK:
		info.status.rates[0].idx = txrate->mcs;
		if (rate == TX_CCK_RATE_5_5_MBPS)
			arsta->tx_stats.txrate.legacy = rate * 10 + 50;
		else
			arsta->tx_stats.txrate.legacy = rate * 10;
		if (sgi)
			info.status.rates[0].flags |=
				(IEEE80211_TX_RC_USE_SHORT_PREAMBLE |
				 IEEE80211_TX_RC_SHORT_GI);
		break;
	case WMI_RATE_PREAMBLE_HT:
		info.status.rates[0].idx =
				txrate->mcs + ((txrate->nss - 1) * 8);
		arsta->tx_stats.txrate.flags = RATE_INFO_FLAGS_MCS;
		arsta->tx_stats.txrate.mcs = txrate->mcs;
		if (sgi) {
			arsta->tx_stats.txrate.flags |=
				RATE_INFO_FLAGS_SHORT_GI;
			info.status.rates[0].flags |= IEEE80211_TX_RC_SHORT_GI;
		}
		info.status.rates[0].flags |= IEEE80211_TX_RC_MCS;
		arsta->tx_stats.txrate.nss = txrate->nss;
		arsta->tx_stats.txrate.bw = txrate->bw + RATE_INFO_BW_20;
		break;
	case WMI_RATE_PREAMBLE_VHT:
		ieee80211_rate_set_vht(&info.status.rates[0], txrate->mcs,
				       txrate->nss);
		arsta->tx_stats.txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
		arsta->tx_stats.txrate.mcs = txrate->mcs;
		if (sgi) {
			arsta->tx_stats.txrate.flags |=
					RATE_INFO_FLAGS_SHORT_GI;
			info.status.rates[0].flags |= IEEE80211_TX_RC_SHORT_GI;
		}
		info.status.rates[0].flags |= IEEE80211_TX_RC_VHT_MCS;
		arsta->tx_stats.txrate.nss = txrate->nss;
		arsta->tx_stats.txrate.bw = txrate->bw + RATE_INFO_BW_20;
		break;
	}

	switch (arsta->tx_stats.txrate.bw) {
	case RATE_INFO_BW_40:
		info.status.rates[0].flags |= IEEE80211_TX_RC_40_MHZ_WIDTH;
		break;
	case RATE_INFO_BW_80:
		info.status.rates[0].flags |= IEEE80211_TX_RC_80_MHZ_WIDTH;
		break;
	default:
		break;
	}

	if (success && !skip_auto_rate) {
		info.flags = IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status_noskb(hw, sta, &info);
	}
}

void ath10k_accumulate_per_peer_tx_stats(struct ath10k *ar,
					 struct ieee80211_sta *sta,
					 struct ath10k_peer_tx_stats
					 *peer_stats)
{
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	u8 pream, bw, mcs, nss, rate, gi;
	int idx;
	struct ath10k_tx_stats *tx_stats = &arsta->tx_stats;
	bool legacy_rate, skip_auto_rate;
	struct rate_info txrate;

	pream = ATH10K_HW_PREAMBLE(peer_stats->ratecode);
	legacy_rate = ((pream == WMI_RATE_PREAMBLE_CCK) ||
		       (pream == WMI_RATE_PREAMBLE_OFDM));

	gi = ATH10K_HW_GI(peer_stats->flags);
	skip_auto_rate = ATH10K_HW_DATA_PKT(peer_stats->flags);

	if (legacy_rate) {
		rate = ATH10K_HW_LEGACY_RATE(peer_stats->ratecode);
		mcs = ath10k_map_rate_code_number(rate, pream);
		if (mcs == LEGACY_RATE_NUM)
			return;

		tx_stats->succ_bytes_legacy_rates[mcs] +=
				(peer_stats->succ_bytes);
		tx_stats->succ_pkts_legacy_rates[mcs] +=
				(peer_stats->succ_pkts);
		tx_stats->fail_bytes_legacy_rates[mcs] +=
				(peer_stats->failed_bytes);
		tx_stats->fail_pkts_legacy_rates[mcs] +=
				(peer_stats->failed_pkts);
		tx_stats->retry_bytes_legacy_rates[mcs] +=
				(peer_stats->retry_bytes);
		tx_stats->retry_pkts_legacy_rates[mcs] +=
				(peer_stats->retry_pkts);
		tx_stats->ack_fails +=
				ATH10K_HW_BA_FAIL(peer_stats->flags);
	} else {
		bw = ATH10K_HW_BW(peer_stats->flags);
		nss = ATH10K_HW_NSS(peer_stats->ratecode) - 1;
		mcs = ATH10K_HW_MCS_RATE(peer_stats->ratecode);
		idx = mcs * 8 + 8 * 10 * (nss);
		idx += bw * 2 + gi;

		if (nss >= VHT_NSS_NUM || bw >= VHT_BW_NUM ||
		    mcs >= VHT_MCS_NUM)
			return;

		if (ATH10K_HW_AMPDU(peer_stats->flags)) {
			tx_stats->ba_fails +=
				ATH10K_HW_BA_FAIL(peer_stats->flags);
			tx_stats->ampdu_bytes_mcs[mcs] +=
					(peer_stats->succ_bytes) +
					(peer_stats->retry_bytes);
			tx_stats->ampdu_pkts_mcs[mcs] +=
					(peer_stats->succ_pkts +
					 peer_stats->retry_pkts);
			tx_stats->ampdu_bytes_bw[bw] +=
				(peer_stats->succ_bytes) +
				(peer_stats->retry_bytes);
			tx_stats->ampdu_bytes_nss[nss] +=
				(peer_stats->succ_bytes) +
				(peer_stats->retry_bytes);
			tx_stats->ampdu_bytes_gi[gi] +=
				(peer_stats->succ_bytes) +
				(peer_stats->retry_bytes);
			tx_stats->ampdu_bytes_rate_num[idx] +=
				(peer_stats->succ_bytes) +
				(peer_stats->retry_bytes);
			tx_stats->ampdu_pkts_bw[bw] +=
					(peer_stats->succ_pkts +
					 peer_stats->retry_pkts);
			tx_stats->ampdu_pkts_nss[nss] +=
					(peer_stats->succ_pkts +
					 peer_stats->retry_pkts);
			tx_stats->ampdu_pkts_gi[gi] +=
					(peer_stats->succ_pkts +
					 peer_stats->retry_pkts);
			tx_stats->ampdu_pkts_rate_num[idx] +=
					(peer_stats->succ_pkts +
					 peer_stats->retry_pkts);
		} else {
			tx_stats->ack_fails +=
				ATH10K_HW_BA_FAIL(peer_stats->flags);
		}
		tx_stats->succ_bytes_mcs[mcs] +=
				(peer_stats->succ_bytes);
		tx_stats->succ_pkts_mcs[mcs] +=
				(peer_stats->succ_pkts);
		tx_stats->fail_bytes_mcs[mcs] +=
				(peer_stats->failed_bytes);
		tx_stats->fail_pkts_mcs[mcs] +=
				(peer_stats->failed_pkts);
		tx_stats->retry_bytes_mcs[mcs] +=
				(peer_stats->retry_bytes);
		tx_stats->retry_pkts_mcs[mcs] +=
				(peer_stats->retry_pkts);
		tx_stats->succ_bytes_bw[bw] +=
			(peer_stats->succ_bytes);
		tx_stats->succ_bytes_nss[nss] +=
			(peer_stats->succ_bytes);
		tx_stats->succ_bytes_gi[gi] +=
			(peer_stats->succ_bytes);
		tx_stats->succ_bytes_rate_num[idx] +=
			(peer_stats->succ_bytes);
		tx_stats->succ_pkts_bw[bw] +=
			(peer_stats->succ_pkts);
		tx_stats->succ_pkts_nss[nss] +=
			(peer_stats->succ_pkts);
		tx_stats->succ_pkts_gi[gi] +=
			(peer_stats->succ_pkts);
		tx_stats->succ_pkts_rate_num[idx] +=
			(peer_stats->succ_pkts);
		tx_stats->fail_bytes_bw[bw] +=
			(peer_stats->failed_bytes);
		tx_stats->fail_bytes_nss[nss] +=
			(peer_stats->failed_bytes);
		tx_stats->fail_bytes_gi[gi] +=
			(peer_stats->failed_bytes);
		tx_stats->fail_bytes_rate_num[idx] +=
			(peer_stats->failed_bytes);
		tx_stats->fail_pkts_bw[bw] +=
			(peer_stats->failed_pkts);
		tx_stats->fail_pkts_nss[nss] +=
			(peer_stats->failed_pkts);
		tx_stats->fail_pkts_gi[gi] +=
			(peer_stats->failed_pkts);
		tx_stats->fail_pkts_rate_num[idx] +=
			(peer_stats->failed_pkts);
		tx_stats->retry_bytes_bw[bw] +=
			(peer_stats->retry_bytes);
		tx_stats->retry_bytes_nss[nss] +=
			(peer_stats->retry_bytes);
		tx_stats->retry_bytes_gi[gi] +=
			(peer_stats->retry_bytes);
		tx_stats->retry_bytes_rate_num[idx] +=
			(peer_stats->retry_bytes);
		tx_stats->retry_pkts_bw[bw] +=
			(peer_stats->retry_pkts);
		tx_stats->retry_pkts_nss[nss] +=
			(peer_stats->retry_pkts);
		tx_stats->retry_pkts_gi[gi] +=
			(peer_stats->retry_pkts);
		tx_stats->retry_pkts_rate_num[idx] +=
			(peer_stats->retry_pkts);
	}

	txrate.flags = pream;
	txrate.mcs = mcs;
	txrate.nss = nss + 1;
	txrate.bw = bw;
	ath10k_fill_tx_bitrate(ar->hw, sta, &txrate, rate, gi,
			       peer_stats->succ_pkts,
			       peer_stats->failed_pkts,
			       peer_stats->retry_pkts,
			       skip_auto_rate);
}

static void ath10k_sta_update_extd_stats_rx_duration(struct ath10k *ar,
						     struct list_head *head)
{
	struct ieee80211_sta *sta;
	struct ath10k_fw_extd_stats_peer *peer;
	struct ath10k_sta *arsta;

	rcu_read_lock();
	list_for_each_entry(peer, head, list) {
		sta = ieee80211_find_sta_by_ifaddr(ar->hw, peer->peer_macaddr,
						   NULL);
		if (!sta)
			continue;
		arsta = (struct ath10k_sta *)sta->drv_priv;
		arsta->rx_duration += (u64)peer->rx_duration;
	}
	rcu_read_unlock();
}

static void ath10k_sta_update_stats_rx_duration(struct ath10k *ar,
						struct list_head *head)
{	struct ieee80211_sta *sta;
	struct ath10k_fw_stats_peer *peer;
	struct ath10k_sta *arsta;

	rcu_read_lock();
	list_for_each_entry(peer, head, list) {
		sta = ieee80211_find_sta_by_ifaddr(ar->hw, peer->peer_macaddr,
						   NULL);
		if (!sta)
			continue;
		arsta = (struct ath10k_sta *)sta->drv_priv;
		arsta->rx_duration += (u64)peer->rx_duration;
	}
	rcu_read_unlock();
}

void ath10k_sta_update_rx_duration(struct ath10k *ar,
				   struct ath10k_fw_stats *stats)
{
	if (ar->wmi.op_version < ATH10K_FW_WMI_OP_VERSION_10_4)
		ath10k_sta_update_stats_rx_duration(ar, &stats->peers);
	else
		ath10k_sta_update_extd_stats_rx_duration(ar,
							 &stats->peers_extd);
}

static ssize_t ath10k_dbg_sta_read_aggr_mode(struct file *file,
					     char __user *user_buf,
					     size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[32];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "aggregation mode: %s\n",
			(arsta->aggr_mode == ATH10K_DBG_AGGR_MODE_AUTO) ?
			"auto" : "manual");
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_aggr_mode(struct file *file,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 aggr_mode;
	int ret;

	if (kstrtouint_from_user(user_buf, count, 0, &aggr_mode))
		return -EINVAL;

	if (aggr_mode >= ATH10K_DBG_AGGR_MODE_MAX)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (aggr_mode == arsta->aggr_mode)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_clear_resp(ar, arsta->arvif->vdev_id, sta->addr);
	if (ret) {
		ath10k_warn(ar, "failed to clear addba session ret: %d\n", ret);
		goto out;
	}

	arsta->aggr_mode = aggr_mode;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_aggr_mode = {
	.read = ath10k_dbg_sta_read_aggr_mode,
	.write = ath10k_dbg_sta_write_aggr_mode,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_addba(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, buf_size;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u", &tid, &buf_size);
	if (ret != 2)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_send(ar, arsta->arvif->vdev_id, sta->addr,
				    tid, buf_size);
	if (ret) {
		ath10k_warn(ar, "failed to send addba request: vdev_id %u peer %pM tid %u buf_size %u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, buf_size);
	}

	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_addba = {
	.write = ath10k_dbg_sta_write_addba,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_addba_resp(struct file *file,
					       const char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, status;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u", &tid, &status);
	if (ret != 2)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_addba_set_resp(ar, arsta->arvif->vdev_id, sta->addr,
					tid, status);
	if (ret) {
		ath10k_warn(ar, "failed to send addba response: vdev_id %u peer %pM tid %u status%u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, status);
	}
	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_addba_resp = {
	.write = ath10k_dbg_sta_write_addba_resp,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_delba(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u32 tid, initiator, reason;
	int ret;
	char buf[64];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u %u", &tid, &initiator, &reason);
	if (ret != 3)
		return -EINVAL;

	/* Valid TID values are 0 through 15 */
	if (tid > HTT_DATA_TX_EXT_TID_MGMT - 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if ((ar->state != ATH10K_STATE_ON) ||
	    (arsta->aggr_mode != ATH10K_DBG_AGGR_MODE_MANUAL)) {
		ret = count;
		goto out;
	}

	ret = ath10k_wmi_delba_send(ar, arsta->arvif->vdev_id, sta->addr,
				    tid, initiator, reason);
	if (ret) {
		ath10k_warn(ar, "failed to send delba: vdev_id %u peer %pM tid %u initiator %u reason %u\n",
			    arsta->arvif->vdev_id, sta->addr, tid, initiator,
			    reason);
	}
	ret = count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_delba = {
	.write = ath10k_dbg_sta_write_delba,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_rx_duration(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	char buf[100];
	int len = 0;

	len = scnprintf(buf, sizeof(buf),
			"%llu usecs\n", arsta->rx_duration);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_rx_duration = {
	.read = ath10k_dbg_sta_read_rx_duration,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define str(s) #s
#define STATS_OUTPUT_FORMAT(name) 					\
	do {								\
	len += scnprintf(buf + len, size - len, "%s: \n", str(name));	\
	len += scnprintf(buf + len, size - len, "MCS %s: ",		\
		(strstr(str(name), "pkts")) ? "packets" : "bytes");	\
	for (i = 0; i < VHT_MCS_NUM; i++)				\
		len += scnprintf(buf + len, size - len, "%llu, ",	\
				arsta->tx_stats.name## _mcs[i]);	\
	len += scnprintf(buf + len, size - len, "\n");			\
	len += scnprintf(buf + len, size - len,				\
			"BW %s:  20Mhz: %llu\t40Mhz: %llu\t80Mhz: %llu\t",\
		(strstr(str(name), "pkts")) ? "packets" : "bytes",	\
			arsta->tx_stats.name## _bw[0],			\
			arsta->tx_stats.name## _bw[1],			\
			arsta->tx_stats.name## _bw[2]);			\
	len += scnprintf(buf + len, size - len, "160Mhz: %llu\n",	\
			arsta->tx_stats.name## _bw[3]);			\
	len += scnprintf(buf + len, size - len,				\
			"NSS %s: 1x1: %llu\t2x2: %llu\t3x3: %llu\t",	\
		(strstr(str(name), "pkts")) ? "packets" : "bytes",	\
			arsta->tx_stats.name## _nss[0],			\
			arsta->tx_stats.name## _nss[1],			\
			arsta->tx_stats.name## _nss[2]);		\
	len += scnprintf(buf + len, size - len, "4x4: %llu\n",		\
			arsta->tx_stats.name## _nss[3]);		\
	len += scnprintf(buf + len, size - len, "GI %s:  LGI: %llu\t",	\
		(strstr(str(name), "pkts")) ? "packets" : "bytes",	\
			arsta->tx_stats.name## _gi[0]);			\
	len += scnprintf(buf + len, size - len, "SGI: %llu\n",		\
		arsta->tx_stats.name## _gi[1]);			\
	len += scnprintf(buf + len, size - len, "legacy rate %s: ",	\
		(strstr(str(name), "pkts")) ? "packets" : "bytes");	\
	len += scnprintf(buf + len, size - len,				\
			"\t1Mbps: %llu\t2Mbps: %llu\t",			\
			arsta->tx_stats.name## _legacy_rates[0],	\
			arsta->tx_stats.name## _legacy_rates[1]);	\
	len += scnprintf(buf + len, size - len,				\
			"5.5Mbps: %llu\t11Mbps: %llu\n",		\
			arsta->tx_stats.name## _legacy_rates[2],	\
			arsta->tx_stats.name## _legacy_rates[3]);	\
	len += scnprintf(buf + len, size - len,				\
			"\t\t\t6Mbps: %llu\t9Mbps: %llu\t",		\
			arsta->tx_stats.name## _legacy_rates[4],	\
			arsta->tx_stats.name## _legacy_rates[5]);	\
	len += scnprintf(buf + len, size - len,				\
			"12Mbps: %llu\t18Mbps: %llu\n",			\
			arsta->tx_stats.name## _legacy_rates[6],	\
			arsta->tx_stats.name## _legacy_rates[7]);	\
	len += scnprintf(buf + len, size - len,				\
			"\t\t\t24Mbps: %llu\t36Mbps: %llu\t",		\
			arsta->tx_stats.name## _legacy_rates[8],	\
			arsta->tx_stats.name## _legacy_rates[9]);	\
	len += scnprintf(buf + len, size - len,				\
			"48Mbps: %llu\t54Mbps: %llu\n",			\
			arsta->tx_stats.name## _legacy_rates[10],	\
			arsta->tx_stats.name## _legacy_rates[11]);	\
	len += scnprintf(buf + len, size - len, "Rate table %s :\n",	\
		(strstr(str(name), "pkts")) ? "packets" : "bytes");	\
	for (i = 0; i < VHT_RATE_NUM; i++) {				\
		len += scnprintf(buf + len, size - len, "\t%llu",	\
			arsta->tx_stats.name## _rate_num[i]);		\
		if (!((i + 1) % 8))					\
			len += scnprintf(buf + len, size - len, "\n");	\
	}								\
	len += scnprintf(buf + len, size - len, "\n");			\
	} while (0)

static ssize_t ath10k_dbg_sta_dump_tx_stats(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	char *buf;
	int len = 0, i, retval = 0, size = 24 * 1024;

	buf = kzalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	STATS_OUTPUT_FORMAT(succ_pkts);
	STATS_OUTPUT_FORMAT(succ_bytes);
	STATS_OUTPUT_FORMAT(ampdu_pkts);
	STATS_OUTPUT_FORMAT(ampdu_bytes);
	STATS_OUTPUT_FORMAT(fail_pkts);
	STATS_OUTPUT_FORMAT(fail_bytes);
	STATS_OUTPUT_FORMAT(retry_pkts);
	STATS_OUTPUT_FORMAT(retry_bytes);

	len += scnprintf(buf + len, size - len,
			 "\nTX duration:\t %llu usecs\n",
			 arsta->tx_stats.tx_duration);

	len += scnprintf(buf + len, size - len,
			"BA fails:\t %llu\n", arsta->tx_stats.ba_fails);
	len += scnprintf(buf + len, size - len,
			"ACK fails\n %llu\n", arsta->tx_stats.ack_fails);

	if (len > size)
		len = size;
	retval = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return retval;
}

static const struct file_operations fops_tx_stats = {
	.read = ath10k_dbg_sta_dump_tx_stats,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_tx_success_bytes(struct file *file,
						    char __user *user_buf,
						    size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	char buf[100];
	int len = 0, i;
	u64 total_succ_bytes;

	total_succ_bytes = arsta->tx_stats.succ_bytes_gi[0] +
		arsta->tx_stats.succ_bytes_gi[1];
	for (i = 0; i < LEGACY_RATE_NUM; i++)
		total_succ_bytes += arsta->tx_stats.succ_bytes_legacy_rates[i];

	len = scnprintf(buf, sizeof(buf),
			"%llu\n", total_succ_bytes);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_tx_success_bytes = {
	.read = ath10k_dbg_sta_read_tx_success_bytes,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_ampdu_subframe_count(struct file *file,
							char __user *user_buf,
							size_t count,
							loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[32];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len,
			"ampdu_subframe_count: %d\n",
			arsta->ampdu_subframe_count);
	mutex_unlock(&ar->conf_mutex);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_ampdu_subframe_count(struct file *file,
							 const char __user *user_buf,
							 size_t count,
							 loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u8 ampdu_subframe_count;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &ampdu_subframe_count))
		return -EINVAL;

	if (ampdu_subframe_count > ATH10K_AMPDU_SUBFRAME_COUNT_MAX ||
	    ampdu_subframe_count < ATH10K_AMPDU_SUBFRAME_COUNT_MIN)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if (ar->state != ATH10K_STATE_ON) {
		ret = -EBUSY;
		goto out;
	}

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					WMI_PEER_AMPDU, ampdu_subframe_count);
	if (ret) {
		ath10k_warn(ar, "failed to set ampdu subframe count for station"
			    " ret: %d\n", ret);
		goto out;
	}

	ret = count;
	arsta->ampdu_subframe_count = ampdu_subframe_count;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_set_ampdu_subframe_count = {
	.read = ath10k_dbg_sta_read_ampdu_subframe_count,
	.write = ath10k_dbg_sta_write_ampdu_subframe_count,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_tpc(struct file *file,
				       char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[20];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "tpc: %d dBm\n", arsta->tpc);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_tpc(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u8 tpc;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &tpc))
		return -EINVAL;

	if (tpc > ATH10K_TPC_MAX_VAL || tpc < ATH10K_TPC_MIN_VAL)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if (ar->state != ATH10K_STATE_ON) {
		ret = -EBUSY;
		goto out;
	}

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					WMI_PEER_USE_FIXED_PWR, tpc);
	if (ret) {
		ath10k_warn(ar, "failed to set tx power for station ret: %d\n",
			    ret);
		goto out;
	}

	ret = count;
	arsta->tpc = tpc;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_set_tpc = {
	.read = ath10k_dbg_sta_read_tpc,
	.write = ath10k_dbg_sta_write_tpc,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_write_cfr_capture(struct file *file,
						const char __user *user_buf,
						size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	struct wmi_peer_cfr_capture_conf_arg arg;
	u32 per_peer_cfr_status = 0, per_peer_cfr_bw  = 0;
	u32 per_peer_cfr_method = 0, per_peer_cfr_period = 0;
	int ret;
	char buf[64] = {0};

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto out;
	}

	ret = sscanf(buf, "%u %u %u %u", &per_peer_cfr_status, &per_peer_cfr_bw,
		     &per_peer_cfr_period, &per_peer_cfr_method);

	if (ret < 1) {
		ret = -EINVAL;
		goto out;
	}

	if (per_peer_cfr_status && ret != 4) {
		ret = -EINVAL;
		goto out;
	}

	if (per_peer_cfr_status == arsta->cfr_capture.cfr_enable &&
	    (per_peer_cfr_period &&
	    per_peer_cfr_period == arsta->cfr_capture.cfr_period) &&
	    per_peer_cfr_bw == arsta->cfr_capture.cfr_bandwidth &&
	    per_peer_cfr_method == arsta->cfr_capture.cfr_method) {
		ret = count;
		goto out;
	}

	if (per_peer_cfr_status > WMI_PEER_CFR_CAPTURE_ENABLE ||
	    per_peer_cfr_status < WMI_PEER_CFR_CAPTURE_DISABLE ||
	    per_peer_cfr_bw >= WMI_PEER_CFR_CAPTURE_BW_MAX  ||
	    per_peer_cfr_bw < WMI_PEER_CFR_CAPTURE_BW_20MHZ ||
	    per_peer_cfr_method < WMI_PEER_CFR_CAPTURE_METHOD_NULL_FRAME ||
	    per_peer_cfr_method >= WMI_PEER_CFR_CAPTURE_METHOD_MAX ||
	    per_peer_cfr_period < WMI_PEER_CFR_PERIODICITY_MIN ||
	    per_peer_cfr_period > WMI_PEER_CFR_PERIODICITY_MAX) {
		ret = -EINVAL;
		goto out;
	}

	/*TODO: Need rework when base time is configurable*/
	if (per_peer_cfr_period % 10) {
		ret = -EINVAL;
		goto out;
	}

	/*TODO:Need correction for 80+80 MHz*/
	if (per_peer_cfr_bw > sta->bandwidth) {
		ret = -EINVAL;
		goto out;
	}

	if (!per_peer_cfr_status) {
		per_peer_cfr_bw = arsta->cfr_capture.cfr_bandwidth;
		per_peer_cfr_period = arsta->cfr_capture.cfr_period;
		per_peer_cfr_method = arsta->cfr_capture.cfr_method;
	}

	arg.request = per_peer_cfr_status;
	arg.periodicity = per_peer_cfr_period;
	arg.bandwidth = per_peer_cfr_bw;
	arg.capture_method = per_peer_cfr_method;

	ret = ath10k_wmi_peer_set_cfr_capture_conf(ar, arsta->arvif->vdev_id,
						   sta->addr, &arg);
	if (ret) {
		ath10k_warn(ar, "failed to send cfr capture info: vdev_id %u peer %pM\n",
			    arsta->arvif->vdev_id, sta->addr);
		goto out;
	}

	ret = count;

	arsta->cfr_capture.cfr_enable = per_peer_cfr_status;
	arsta->cfr_capture.cfr_period = per_peer_cfr_period;
	arsta->cfr_capture.cfr_bandwidth = per_peer_cfr_bw;
	arsta->cfr_capture.cfr_method = per_peer_cfr_method;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_dbg_sta_read_cfr_capture(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[512];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "cfr_status: %s\n"
			"cfr_bandwidth: %dMHz\ncfr_period: %d ms\ncfr_method: %d\n",
			(arsta->cfr_capture.cfr_enable) ? "enabled" :
			"disabled", (arsta->cfr_capture.cfr_bandwidth == 0) ?
			20 : (arsta->cfr_capture.cfr_bandwidth == 1) ?
			40 : 80, arsta->cfr_capture.cfr_period,
			arsta->cfr_capture.cfr_method);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_cfr_capture = {
	.read = ath10k_dbg_sta_read_cfr_capture,
	.write = ath10k_dbg_sta_write_cfr_capture,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_peer_tid_log(struct file *file, char __user *user_buf,
						size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[50];
	int ret, len;

	mutex_lock(&ar->conf_mutex);

	len = 0;
	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto out;
	}

	/* This will enable the FW log message and dumped on the console*/

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					WMI_PEER_DEBUG, 1);
	if (ret) {
		len = scnprintf(buf, sizeof(buf) - len, "%s %d\n",
				"failed to set peer tid for station ret: ", ret);
		goto out;
	} else
		len = scnprintf(buf, sizeof(buf) - len, "\n %s \n\n",
				"dumping peer tid logs on the console");

out:
	mutex_unlock(&ar->conf_mutex);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_peer_tid_log = {
	.open = simple_open,
	.read = ath10k_dbg_sta_read_peer_tid_log,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_peer_ps_state(struct file *file,
						 char __user *user_buf,
						 size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[20];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	if(ar->ps_state_enable == 0)
		arsta->peer_ps_state = 2;
	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			arsta->peer_ps_state);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_peer_ps_state = {
	.open = simple_open,
	.read = ath10k_dbg_sta_read_peer_ps_state,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_dbg_sta_read_pspoll_sta_ko_enable(struct file *file,
							char __user *user_buf,
							size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	char buf[50];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "pspoll_sta_ko_enable: %d\n",
			arsta->pspoll_sta_ko_enable);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_dbg_sta_write_pspoll_sta_ko_enable(struct file *file,
							 const char __user *user_buf,
							 size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k *ar = arsta->arvif->ar;
	u8 pspoll_sta_ko_enable;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &pspoll_sta_ko_enable))
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if (ar->state != ATH10K_STATE_ON) {
		ret = -EBUSY;
		goto out;
	}

	ret = ath10k_wmi_peer_set_param(ar, arsta->arvif->vdev_id, sta->addr,
					WMI_PEER_PS_POLL_KICKOUT,
					pspoll_sta_ko_enable);
	if (ret) {
		ath10k_warn(ar, "failed to toggle pspoll sta kickout logic for"
			    " station ret: %d\n", ret);
		goto out;
	}

	ret = count;
	arsta->pspoll_sta_ko_enable = pspoll_sta_ko_enable;
out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_set_pspoll_sta_ko_enable = {
	.read = ath10k_dbg_sta_read_pspoll_sta_ko_enable,
	.write = ath10k_dbg_sta_write_pspoll_sta_ko_enable,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void ath10k_sta_add_debugfs(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta, struct dentry *dir)
{
	struct ath10k *ar = hw->priv;

	debugfs_create_file("aggr_mode", S_IRUGO | S_IWUSR, dir, sta,
			    &fops_aggr_mode);
	debugfs_create_file("addba", S_IWUSR, dir, sta, &fops_addba);
	debugfs_create_file("addba_resp", S_IWUSR, dir, sta, &fops_addba_resp);
	debugfs_create_file("delba", S_IWUSR, dir, sta, &fops_delba);
	debugfs_create_file("rx_duration", S_IRUGO, dir, sta,
			    &fops_rx_duration);
	debugfs_create_file("tpc", S_IRUGO | S_IWUSR, dir, sta,
			    &fops_set_tpc);
	debugfs_create_file("ampdu_subframe_count", S_IRUGO | S_IWUSR, dir, sta,
			    &fops_set_ampdu_subframe_count);
	debugfs_create_file("peer_tid_log", S_IRUSR, dir, sta, &fops_peer_tid_log);
	debugfs_create_file("tx_success_bytes", S_IRUGO, dir, sta,
			    &fops_tx_success_bytes);
	debugfs_create_file("peer_ps_state", S_IRUSR, dir, sta,
			    &fops_peer_ps_state);
	debugfs_create_file("tx_stats", 0444, dir, sta,
			    &fops_tx_stats);
	if (test_bit(WMI_SERVICE_CFR_CAPTURE_SUPPORT, ar->wmi.svc_map))
		debugfs_create_file("cfr_capture", 0644 , dir,
				    sta, &fops_cfr_capture);
	debugfs_create_file("pspoll_sta_ko_enable", S_IRUGO | S_IWUSR, dir, sta,
			    &fops_set_pspoll_sta_ko_enable);
}
