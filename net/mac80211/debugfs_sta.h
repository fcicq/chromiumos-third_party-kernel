/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MAC80211_DEBUGFS_STA_H
#define __MAC80211_DEBUGFS_STA_H

#include "sta_info.h"

#ifdef CONFIG_MAC80211_DEBUGFS
void ieee80211_sta_debugfs_add(struct sta_info *sta);
void ieee80211_sta_debugfs_remove(struct sta_info *sta);
void ieee80211_rx_h_sta_stats(struct sta_info *sta, struct sk_buff *skb);
#else
static inline void ieee80211_sta_debugfs_add(struct sta_info *sta) {}
static inline void ieee80211_sta_debugfs_remove(struct sta_info *sta) {}
static inline void ieee80211_rx_h_sta_stats(struct sta_info *sta,
					    struct sk_buff *skb)
{
}
#endif

#endif /* __MAC80211_DEBUGFS_STA_H */
