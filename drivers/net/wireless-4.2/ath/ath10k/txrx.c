/*
 * Copyright (c) 2005-2011 Atheros Communications Inc.
 * Copyright (c) 2011-2013 Qualcomm Atheros, Inc.
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
#include "txrx.h"
#include "htt.h"
#include "mac.h"
#include "debug.h"

static void ath10k_report_offchan_tx(struct ath10k *ar, struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	if (likely(!(info->flags & IEEE80211_TX_CTL_TX_OFFCHAN)))
		return;

	if (ath10k_mac_tx_frm_has_freq(ar))
		return;

	/* If the original wait_for_completion() timed out before
	 * {data,mgmt}_tx_completed() was called then we could complete
	 * offchan_tx_completed for a different skb. Prevent this by using
	 * offchan_tx_skb. */
	spin_lock_bh(&ar->data_lock);
	if (ar->offchan_tx_skb != skb) {
		ath10k_warn(ar, "completed old offchannel frame\n");
		goto out;
	}

	complete(&ar->offchan_tx_completed);
	ar->offchan_tx_skb = NULL; /* just for sanity */

	ath10k_dbg(ar, ATH10K_DBG_HTT, "completed offchannel skb %p\n", skb);
out:
	spin_unlock_bh(&ar->data_lock);
}

static inline u32 txdelay_time_to_ms(u32 val)
{
	u64 valns = ((u64)val << IEEE80211_TX_DELAY_SHIFT);

	do_div(valns, NSEC_PER_MSEC);
	return (u32)valns;
}

/* Returns transmit delay histogram stats bin to credit based on latency. */
static inline int txdelay_ms_to_bin(u32 latency)
{
	int top_bit_set;
	int bin_offset;

	/* The exponential (power-of-two) bucket range is determined by the high
	 * order bit set.  The first two 1ms bin (i.e. [0, 1) and [1, 2)) are
	 * returned directly.  All other bins are subdivided in half by
	 * calculating bin_offset based on the bit immediately to the right of
	 * the high order bit set.
	 */
	top_bit_set = fls(latency);
	if (top_bit_set < 2)
		return top_bit_set;
	if (top_bit_set > ATH10K_TX_DELAY_STATS_MAX_BIN)
		return ATH10K_TX_DELAY_STATS_MAX_BIN;
	bin_offset = (latency & (1 << (top_bit_set - 2))) ? 1 : 0;
	return (top_bit_set - 1) * 2 + bin_offset;
}

void ath10k_update_latency_stats(struct ath10k *ar, struct sk_buff *msdu,
				 u8 tid)
{
	u32	enqueue_time, now, bin;
	struct ieee80211_tx_info *info;

	now = ieee80211_txdelay_get_time();

	info = IEEE80211_SKB_CB(msdu);
	enqueue_time = info->tx_start_time;
	if (enqueue_time == 0)
		return;

	bin = txdelay_ms_to_bin(txdelay_time_to_ms(now - enqueue_time));
	ar->debug.tx_delay_stats[tid]->counts[bin]++;
}

/* Update airtime upon tx completion. It is called while holding txq_lock */
void ath10k_atf_tx_complete(struct ath10k *ar, struct sk_buff *skb)
{
	struct ieee80211_txq *txq;
	struct ath10k_skb_cb *skb_cb;
	struct ath10k_txq *artxq;
	struct atf_scheduler *atf;

	skb_cb = ATH10K_SKB_CB(skb);

	txq = skb_cb->txq;
	if (!txq)
		return;
	ar->airtime_inflight -= skb_cb->airtime_est;
	ar->atf_bytes_send += skb->len;
	artxq = (void *)txq->drv_priv;
	atf = &artxq->atf;

	atf->frames_inflight--;
	atf->bytes_send += skb->len;
	atf->airtime_inflight -= skb_cb->airtime_est;
	trace_ath10k_atf_tx_complete(ar, skb, skb_cb->airtime_est, txq);
}

int ath10k_txrx_tx_unref(struct ath10k_htt *htt,
			 const struct htt_tx_done *tx_done)
{
	struct ath10k *ar = htt->ar;
	struct device *dev = ar->dev;
	struct ieee80211_tx_info *info;
	struct ieee80211_txq *txq;
	struct ath10k_skb_cb *skb_cb;
	struct ath10k_txq *artxq;
	struct sk_buff *msdu;

	ath10k_dbg(ar, ATH10K_DBG_HTT,
		   "htt tx completion msdu_id %u discard %d no_ack %d success %d\n",
		   tx_done->msdu_id, !!tx_done->discard,
		   !!tx_done->no_ack, !!tx_done->success);

	if (tx_done->msdu_id >= htt->max_num_pending_tx) {
		ath10k_warn(ar, "warning: msdu_id %d too big, ignoring\n",
			    tx_done->msdu_id);
		return -EINVAL;
	}

	spin_lock_bh(&htt->tx_lock);
	msdu = idr_find(&htt->pending_tx, tx_done->msdu_id);
	if (!msdu) {
		ath10k_warn(ar, "received tx completion for invalid msdu_id: %d\n",
			    tx_done->msdu_id);
		spin_unlock_bh(&htt->tx_lock);
		return -ENOENT;
	}

	skb_cb = ATH10K_SKB_CB(msdu);
	txq = skb_cb->txq;

	if (txq) {
		artxq = (void *)txq->drv_priv;
		artxq->num_fw_queued--;
		ath10k_atf_tx_complete(htt->ar, msdu);
	}

	ath10k_htt_tx_free_msdu_id(htt, tx_done->msdu_id);
	ath10k_htt_tx_dec_pending(htt);
	if (htt->num_pending_tx == 0)
		wake_up(&htt->empty_tx_wq);
	spin_unlock_bh(&htt->tx_lock);

	dma_unmap_single(dev, skb_cb->paddr, msdu->len, DMA_TO_DEVICE);

	ath10k_report_offchan_tx(htt->ar, msdu);

	info = IEEE80211_SKB_CB(msdu);
	if (txq)
		ath10k_update_latency_stats(htt->ar, msdu, txq->tid);
	memset(&info->status, 0, sizeof(info->status));
	info->status.rates[0].idx = -1;
	trace_ath10k_txrx_tx_unref(ar, tx_done->msdu_id);

	if (tx_done->discard) {
		ieee80211_free_txskb(htt->ar->hw, msdu);
		return 0;
	}

	if (!(info->flags & IEEE80211_TX_CTL_NO_ACK))
		info->flags |= IEEE80211_TX_STAT_ACK;

	if (tx_done->no_ack)
		info->flags &= ~IEEE80211_TX_STAT_ACK;

	if (tx_done->success && (info->flags & IEEE80211_TX_CTL_NO_ACK))
		info->flags |= IEEE80211_TX_STAT_NOACK_TRANSMITTED;

	ieee80211_tx_status(htt->ar->hw, msdu);
	/* we do not own the msdu anymore */

	return 0;
}

struct ath10k_peer *ath10k_peer_find(struct ath10k *ar, int vdev_id,
				     const u8 *addr)
{
	struct ath10k_peer *peer;

	lockdep_assert_held(&ar->data_lock);

	list_for_each_entry(peer, &ar->peers, list) {
		if (peer->vdev_id != vdev_id)
			continue;
		if (memcmp(peer->addr, addr, ETH_ALEN))
			continue;

		return peer;
	}

	return NULL;
}

struct ath10k_peer *ath10k_peer_find_by_id(struct ath10k *ar, int peer_id)
{
	struct ath10k_peer *peer;

	lockdep_assert_held(&ar->data_lock);

	list_for_each_entry(peer, &ar->peers, list)
		if (test_bit(peer_id, peer->peer_ids))
			return peer;

	return NULL;
}

static int ath10k_wait_for_peer_common(struct ath10k *ar, int vdev_id,
				       const u8 *addr, bool expect_mapped)
{
	long time_left;

	time_left = wait_event_timeout(ar->peer_mapping_wq, ({
			bool mapped;

			spin_lock_bh(&ar->data_lock);
			mapped = !!ath10k_peer_find(ar, vdev_id, addr);
			spin_unlock_bh(&ar->data_lock);

			(mapped == expect_mapped ||
			 test_bit(ATH10K_FLAG_CRASH_FLUSH, &ar->dev_flags));
		}), 3*HZ);

	if (time_left == 0)
		return -ETIMEDOUT;

	return 0;
}

int ath10k_wait_for_peer_created(struct ath10k *ar, int vdev_id, const u8 *addr)
{
	return ath10k_wait_for_peer_common(ar, vdev_id, addr, true);
}

int ath10k_wait_for_peer_deleted(struct ath10k *ar, int vdev_id, const u8 *addr)
{
	return ath10k_wait_for_peer_common(ar, vdev_id, addr, false);
}

void ath10k_peer_map_event(struct ath10k_htt *htt,
			   struct htt_peer_map_event *ev)
{
	struct ath10k *ar = htt->ar;
	struct ath10k_peer *peer;

	spin_lock_bh(&ar->data_lock);
	peer = ath10k_peer_find(ar, ev->vdev_id, ev->addr);
	if (!peer) {
		peer = kzalloc(sizeof(*peer), GFP_ATOMIC);
		if (!peer)
			goto exit;

		peer->vdev_id = ev->vdev_id;
		ether_addr_copy(peer->addr, ev->addr);
		list_add(&peer->list, &ar->peers);
		wake_up(&ar->peer_mapping_wq);
	}

	ath10k_dbg(ar, ATH10K_DBG_HTT, "htt peer map vdev %d peer %pM id %d\n",
		   ev->vdev_id, ev->addr, ev->peer_id);

	ar->peer_map[ev->peer_id] = peer;
	set_bit(ev->peer_id, peer->peer_ids);
exit:
	spin_unlock_bh(&ar->data_lock);
}

void ath10k_peer_unmap_event(struct ath10k_htt *htt,
			     struct htt_peer_unmap_event *ev)
{
	struct ath10k *ar = htt->ar;
	struct ath10k_peer *peer;

	spin_lock_bh(&ar->data_lock);
	peer = ath10k_peer_find_by_id(ar, ev->peer_id);
	if (!peer) {
		ath10k_warn(ar, "peer-unmap-event: unknown peer id %d\n",
			    ev->peer_id);
		goto exit;
	}

	ath10k_dbg(ar, ATH10K_DBG_HTT, "htt peer unmap vdev %d peer %pM id %d\n",
		   peer->vdev_id, peer->addr, ev->peer_id);

	ar->peer_map[ev->peer_id] = NULL;
	clear_bit(ev->peer_id, peer->peer_ids);

	if (bitmap_empty(peer->peer_ids, ATH10K_MAX_NUM_PEER_IDS)) {
		list_del(&peer->list);
		kfree(peer);
		wake_up(&ar->peer_mapping_wq);
	}

exit:
	spin_unlock_bh(&ar->data_lock);
}
