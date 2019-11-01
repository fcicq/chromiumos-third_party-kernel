/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM vmscan

#if !defined(_TRACE_VMSCAN_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VMSCAN_H

#include <linux/types.h>
#include <linux/tracepoint.h>
#include <linux/mm.h>
#include <linux/memcontrol.h>
#include <trace/events/mmflags.h>

#define RECLAIM_WB_ANON		0x0001u
#define RECLAIM_WB_FILE		0x0002u
#define RECLAIM_WB_MIXED	0x0010u
#define RECLAIM_WB_SYNC		0x0004u /* Unused, all reclaim async */
#define RECLAIM_WB_ASYNC	0x0008u
#define RECLAIM_WB_LRU		(RECLAIM_WB_ANON|RECLAIM_WB_FILE)

#define show_reclaim_flags(flags)				\
	(flags) ? __print_flags(flags, "|",			\
		{RECLAIM_WB_ANON,	"RECLAIM_WB_ANON"},	\
		{RECLAIM_WB_FILE,	"RECLAIM_WB_FILE"},	\
		{RECLAIM_WB_MIXED,	"RECLAIM_WB_MIXED"},	\
		{RECLAIM_WB_SYNC,	"RECLAIM_WB_SYNC"},	\
		{RECLAIM_WB_ASYNC,	"RECLAIM_WB_ASYNC"}	\
		) : "RECLAIM_WB_NONE"

#define trace_reclaim_flags(page) ( \
	(page_is_file_cache(page) ? RECLAIM_WB_FILE : RECLAIM_WB_ANON) | \
	(RECLAIM_WB_ASYNC) \
	)

#define trace_shrink_flags(file) \
	( \
		(file ? RECLAIM_WB_FILE : RECLAIM_WB_ANON) | \
		(RECLAIM_WB_ASYNC) \
	)

TRACE_EVENT(mm_vmscan_kswapd_sleep,

	TP_PROTO(int nid),

	TP_ARGS(nid),

	TP_STRUCT__entry(
		__field(	int,	nid	)
	),

	TP_fast_assign(
		__entry->nid	= nid;
	),

	TP_printk("nid=%d", __entry->nid)
);

TRACE_EVENT(mm_vmscan_kswapd_wake,

	TP_PROTO(int nid, int zid, int order),

	TP_ARGS(nid, zid, order),

	TP_STRUCT__entry(
		__field(	int,	nid	)
		__field(	int,	zid	)
		__field(	int,	order	)
	),

	TP_fast_assign(
		__entry->nid	= nid;
		__entry->zid    = zid;
		__entry->order	= order;
	),

	TP_printk("nid=%d zid=%d order=%d", __entry->nid, __entry->zid, __entry->order)
);

TRACE_EVENT(mm_vmscan_wakeup_kswapd,

	TP_PROTO(int nid, int zid, int order, gfp_t gfp_flags),

	TP_ARGS(nid, zid, order, gfp_flags),

	TP_STRUCT__entry(
		__field(	int,	nid		)
		__field(	int,	zid		)
		__field(	int,	order		)
		__field(	gfp_t,	gfp_flags	)
	),

	TP_fast_assign(
		__entry->nid		= nid;
		__entry->zid		= zid;
		__entry->order		= order;
		__entry->gfp_flags	= gfp_flags;
	),

	TP_printk("nid=%d zid=%d order=%d gfp_flags=%s",
		__entry->nid,
		__entry->zid,
		__entry->order,
		show_gfp_flags(__entry->gfp_flags))
);

DECLARE_EVENT_CLASS(mm_vmscan_direct_reclaim_begin_template,

	TP_PROTO(int order, int may_writepage, gfp_t gfp_flags, int classzone_idx),

	TP_ARGS(order, may_writepage, gfp_flags, classzone_idx),

	TP_STRUCT__entry(
		__field(	int,	order		)
		__field(	int,	may_writepage	)
		__field(	gfp_t,	gfp_flags	)
		__field(	int,	classzone_idx	)
	),

	TP_fast_assign(
		__entry->order		= order;
		__entry->may_writepage	= may_writepage;
		__entry->gfp_flags	= gfp_flags;
		__entry->classzone_idx	= classzone_idx;
	),

	TP_printk("order=%d may_writepage=%d gfp_flags=%s classzone_idx=%d",
		__entry->order,
		__entry->may_writepage,
		show_gfp_flags(__entry->gfp_flags),
		__entry->classzone_idx)
);

DEFINE_EVENT(mm_vmscan_direct_reclaim_begin_template, mm_vmscan_direct_reclaim_begin,

	TP_PROTO(int order, int may_writepage, gfp_t gfp_flags, int classzone_idx),

	TP_ARGS(order, may_writepage, gfp_flags, classzone_idx)
);

#ifdef CONFIG_MEMCG
DEFINE_EVENT(mm_vmscan_direct_reclaim_begin_template, mm_vmscan_memcg_reclaim_begin,

	TP_PROTO(int order, int may_writepage, gfp_t gfp_flags, int classzone_idx),

	TP_ARGS(order, may_writepage, gfp_flags, classzone_idx)
);

DEFINE_EVENT(mm_vmscan_direct_reclaim_begin_template, mm_vmscan_memcg_softlimit_reclaim_begin,

	TP_PROTO(int order, int may_writepage, gfp_t gfp_flags, int classzone_idx),

	TP_ARGS(order, may_writepage, gfp_flags, classzone_idx)
);
#endif /* CONFIG_MEMCG */

DECLARE_EVENT_CLASS(mm_vmscan_direct_reclaim_end_template,

	TP_PROTO(unsigned long nr_reclaimed),

	TP_ARGS(nr_reclaimed),

	TP_STRUCT__entry(
		__field(	unsigned long,	nr_reclaimed	)
	),

	TP_fast_assign(
		__entry->nr_reclaimed	= nr_reclaimed;
	),

	TP_printk("nr_reclaimed=%lu", __entry->nr_reclaimed)
);

DEFINE_EVENT(mm_vmscan_direct_reclaim_end_template, mm_vmscan_direct_reclaim_end,

	TP_PROTO(unsigned long nr_reclaimed),

	TP_ARGS(nr_reclaimed)
);

#ifdef CONFIG_MEMCG
DEFINE_EVENT(mm_vmscan_direct_reclaim_end_template, mm_vmscan_memcg_reclaim_end,

	TP_PROTO(unsigned long nr_reclaimed),

	TP_ARGS(nr_reclaimed)
);

DEFINE_EVENT(mm_vmscan_direct_reclaim_end_template, mm_vmscan_memcg_softlimit_reclaim_end,

	TP_PROTO(unsigned long nr_reclaimed),

	TP_ARGS(nr_reclaimed)
);
#endif /* CONFIG_MEMCG */

TRACE_EVENT(mm_shrink_slab_start,
	TP_PROTO(struct shrinker *shr, struct shrink_control *sc,
		long nr_objects_to_shrink, unsigned long cache_items,
		unsigned long long delta, unsigned long total_scan,
		int priority),

	TP_ARGS(shr, sc, nr_objects_to_shrink, cache_items, delta, total_scan,
		priority),

	TP_STRUCT__entry(
		__field(struct shrinker *, shr)
		__field(void *, shrink)
		__field(int, nid)
		__field(long, nr_objects_to_shrink)
		__field(gfp_t, gfp_flags)
		__field(unsigned long, cache_items)
		__field(unsigned long long, delta)
		__field(unsigned long, total_scan)
		__field(int, priority)
	),

	TP_fast_assign(
		__entry->shr = shr;
		__entry->shrink = shr->scan_objects;
		__entry->nid = sc->nid;
		__entry->nr_objects_to_shrink = nr_objects_to_shrink;
		__entry->gfp_flags = sc->gfp_mask;
		__entry->cache_items = cache_items;
		__entry->delta = delta;
		__entry->total_scan = total_scan;
		__entry->priority = priority;
	),

	TP_printk("%pF %p: nid: %d objects to shrink %ld gfp_flags %s cache items %ld delta %lld total_scan %ld priority %d",
		__entry->shrink,
		__entry->shr,
		__entry->nid,
		__entry->nr_objects_to_shrink,
		show_gfp_flags(__entry->gfp_flags),
		__entry->cache_items,
		__entry->delta,
		__entry->total_scan,
		__entry->priority)
);

TRACE_EVENT(mm_shrink_slab_end,
	TP_PROTO(struct shrinker *shr, int nid, int shrinker_retval,
		long unused_scan_cnt, long new_scan_cnt, long total_scan),

	TP_ARGS(shr, nid, shrinker_retval, unused_scan_cnt, new_scan_cnt,
		total_scan),

	TP_STRUCT__entry(
		__field(struct shrinker *, shr)
		__field(int, nid)
		__field(void *, shrink)
		__field(long, unused_scan)
		__field(long, new_scan)
		__field(int, retval)
		__field(long, total_scan)
	),

	TP_fast_assign(
		__entry->shr = shr;
		__entry->nid = nid;
		__entry->shrink = shr->scan_objects;
		__entry->unused_scan = unused_scan_cnt;
		__entry->new_scan = new_scan_cnt;
		__entry->retval = shrinker_retval;
		__entry->total_scan = total_scan;
	),

	TP_printk("%pF %p: nid: %d unused scan count %ld new scan count %ld total_scan %ld last shrinker return val %d",
		__entry->shrink,
		__entry->shr,
		__entry->nid,
		__entry->unused_scan,
		__entry->new_scan,
		__entry->total_scan,
		__entry->retval)
);

TRACE_EVENT(mm_vmscan_lru_isolate,
	TP_PROTO(int classzone_idx,
		int order,
		unsigned long nr_requested,
		unsigned long nr_scanned,
		unsigned long nr_skipped,
		unsigned long nr_taken,
		isolate_mode_t isolate_mode,
		int lru),

	TP_ARGS(classzone_idx, order, nr_requested, nr_scanned, nr_skipped, nr_taken, isolate_mode, lru),

	TP_STRUCT__entry(
		__field(int, classzone_idx)
		__field(int, order)
		__field(unsigned long, nr_requested)
		__field(unsigned long, nr_scanned)
		__field(unsigned long, nr_skipped)
		__field(unsigned long, nr_taken)
		__field(isolate_mode_t, isolate_mode)
		__field(int, lru)
	),

	TP_fast_assign(
		__entry->classzone_idx = classzone_idx;
		__entry->order = order;
		__entry->nr_requested = nr_requested;
		__entry->nr_scanned = nr_scanned;
		__entry->nr_skipped = nr_skipped;
		__entry->nr_taken = nr_taken;
		__entry->isolate_mode = isolate_mode;
		__entry->lru = lru;
	),

	TP_printk("isolate_mode=%d classzone=%d order=%d nr_requested=%lu nr_scanned=%lu nr_skipped=%lu nr_taken=%lu lru=%s",
		__entry->isolate_mode,
		__entry->classzone_idx,
		__entry->order,
		__entry->nr_requested,
		__entry->nr_scanned,
		__entry->nr_skipped,
		__entry->nr_taken,
		__print_symbolic(__entry->lru, LRU_NAMES))
);

TRACE_EVENT(mm_vmscan_writepage,

	TP_PROTO(struct page *page),

	TP_ARGS(page),

	TP_STRUCT__entry(
		__field(unsigned long, pfn)
		__field(int, reclaim_flags)
	),

	TP_fast_assign(
		__entry->pfn = page_to_pfn(page);
		__entry->reclaim_flags = trace_reclaim_flags(page);
	),

	TP_printk("page=%p pfn=%lu flags=%s",
		pfn_to_page(__entry->pfn),
		__entry->pfn,
		show_reclaim_flags(__entry->reclaim_flags))
);

TRACE_EVENT(mm_vmscan_lru_shrink_inactive,

	TP_PROTO(int nid,
		unsigned long nr_scanned, unsigned long nr_reclaimed,
		struct reclaim_stat *stat, int priority, int file),

	TP_ARGS(nid, nr_scanned, nr_reclaimed, stat, priority, file),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(unsigned long, nr_scanned)
		__field(unsigned long, nr_reclaimed)
		__field(unsigned long, nr_dirty)
		__field(unsigned long, nr_writeback)
		__field(unsigned long, nr_congested)
		__field(unsigned long, nr_immediate)
		__field(unsigned long, nr_activate)
		__field(unsigned long, nr_ref_keep)
		__field(unsigned long, nr_unmap_fail)
		__field(int, priority)
		__field(int, reclaim_flags)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->nr_scanned = nr_scanned;
		__entry->nr_reclaimed = nr_reclaimed;
		__entry->nr_dirty = stat->nr_dirty;
		__entry->nr_writeback = stat->nr_writeback;
		__entry->nr_congested = stat->nr_congested;
		__entry->nr_immediate = stat->nr_immediate;
		__entry->nr_activate = stat->nr_activate;
		__entry->nr_ref_keep = stat->nr_ref_keep;
		__entry->nr_unmap_fail = stat->nr_unmap_fail;
		__entry->priority = priority;
		__entry->reclaim_flags = trace_shrink_flags(file);
	),

	TP_printk("nid=%d nr_scanned=%ld nr_reclaimed=%ld nr_dirty=%ld nr_writeback=%ld nr_congested=%ld nr_immediate=%ld nr_activate=%ld nr_ref_keep=%ld nr_unmap_fail=%ld priority=%d flags=%s",
		__entry->nid,
		__entry->nr_scanned, __entry->nr_reclaimed,
		__entry->nr_dirty, __entry->nr_writeback,
		__entry->nr_congested, __entry->nr_immediate,
		__entry->nr_activate, __entry->nr_ref_keep,
		__entry->nr_unmap_fail, __entry->priority,
		show_reclaim_flags(__entry->reclaim_flags))
);

TRACE_EVENT(mm_vmscan_lru_shrink_active,

	TP_PROTO(int nid, unsigned long nr_taken,
		unsigned long nr_active, unsigned long nr_deactivated,
		unsigned long nr_referenced, int priority, int file),

	TP_ARGS(nid, nr_taken, nr_active, nr_deactivated, nr_referenced, priority, file),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(unsigned long, nr_taken)
		__field(unsigned long, nr_active)
		__field(unsigned long, nr_deactivated)
		__field(unsigned long, nr_referenced)
		__field(int, priority)
		__field(int, reclaim_flags)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->nr_taken = nr_taken;
		__entry->nr_active = nr_active;
		__entry->nr_deactivated = nr_deactivated;
		__entry->nr_referenced = nr_referenced;
		__entry->priority = priority;
		__entry->reclaim_flags = trace_shrink_flags(file);
	),

	TP_printk("nid=%d nr_taken=%ld nr_active=%ld nr_deactivated=%ld nr_referenced=%ld priority=%d flags=%s",
		__entry->nid,
		__entry->nr_taken,
		__entry->nr_active, __entry->nr_deactivated, __entry->nr_referenced,
		__entry->priority,
		show_reclaim_flags(__entry->reclaim_flags))
);

TRACE_EVENT(mm_vmscan_inactive_list_is_low,

	TP_PROTO(int nid, int reclaim_idx,
		unsigned long total_inactive, unsigned long inactive,
		unsigned long total_active, unsigned long active,
		unsigned long ratio, int file),

	TP_ARGS(nid, reclaim_idx, total_inactive, inactive, total_active, active, ratio, file),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(int, reclaim_idx)
		__field(unsigned long, total_inactive)
		__field(unsigned long, inactive)
		__field(unsigned long, total_active)
		__field(unsigned long, active)
		__field(unsigned long, ratio)
		__field(int, reclaim_flags)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->reclaim_idx = reclaim_idx;
		__entry->total_inactive = total_inactive;
		__entry->inactive = inactive;
		__entry->total_active = total_active;
		__entry->active = active;
		__entry->ratio = ratio;
		__entry->reclaim_flags = trace_shrink_flags(file) & RECLAIM_WB_LRU;
	),

	TP_printk("nid=%d reclaim_idx=%d total_inactive=%ld inactive=%ld total_active=%ld active=%ld ratio=%ld flags=%s",
		__entry->nid,
		__entry->reclaim_idx,
		__entry->total_inactive, __entry->inactive,
		__entry->total_active, __entry->active,
		__entry->ratio,
		show_reclaim_flags(__entry->reclaim_flags))
);

TRACE_EVENT(kstaled_ring,
	TP_PROTO(int nid, const char *op, unsigned ring_head, unsigned tail_anon,
		 unsigned tail_file),

	TP_ARGS(nid, op, ring_head, tail_anon, tail_file),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(const char *, op)
		__field(unsigned, ring_head)
		__field(unsigned, tail_anon)
		__field(unsigned, tail_file)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->op = op;
		__entry->ring_head = ring_head;
		__entry->tail_anon = tail_anon;
		__entry->tail_file = tail_file;
	),

	TP_printk("node %4u; %10s; head %12u; tail anon %12u, file %12u",
		  __entry->nid, __entry->op, __entry->ring_head, __entry->tail_anon,
		  __entry->tail_file)
);

TRACE_EVENT(kstaled_aging,
	TP_PROTO(int nid, bool background, unsigned long hot),

	TP_ARGS(nid, background, hot),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(bool, background)
		__field(unsigned long, hot)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->background = background;
		__entry->hot = hot;
	),

	TP_printk("node %4u; %10s hot %12lu",
		  __entry->nid,
		  __entry->background ? "background" : "direct",
		  __entry->hot)
);

TRACE_EVENT(kstaled_reclaim,
	TP_PROTO(int nid, bool file, bool clean_only, unsigned span,
		 unsigned long scanned, unsigned long sorted,
		 unsigned long isolated, unsigned long reclaimed),

	TP_ARGS(nid, file, clean_only, span, scanned, sorted, isolated,
		reclaimed),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(bool, file)
		__field(bool, clean_only)
		__field(unsigned, span)
		__field(unsigned long, scanned)
		__field(unsigned long, sorted)
		__field(unsigned long, isolated)
		__field(unsigned long, reclaimed)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->file = file;
		__entry->clean_only = clean_only;
		__entry->span = span;
		__entry->scanned = scanned;
		__entry->sorted = sorted;
		__entry->isolated = isolated;
		__entry->reclaimed = reclaimed;
	),

	TP_printk("node %4u; %5s %s; span %4u; scanned %12lu; sorted %12lu; isolated %12lu; reclaimed %12lu",
		  __entry->nid,
		  __entry->clean_only ? "clean" : "",
		  __entry->file ? "file" : "anon",
		  __entry->span, __entry->scanned, __entry->sorted,
		  __entry->isolated, __entry->reclaimed)
);

TRACE_EVENT(kstaled_estimate,
	TP_PROTO(int nid, unsigned long total, unsigned long free,
		 unsigned long drop, unsigned long growth, unsigned span_anon,
		 unsigned span_file, bool walk_pmdp,
		 unsigned long nr_to_reclaim),

	TP_ARGS(nid, total, free, drop, growth, span_anon, span_file,
		walk_pmdp, nr_to_reclaim),

	TP_STRUCT__entry(
		__field(int, nid)
		__field(unsigned long, total)
		__field(unsigned long, free)
		__field(unsigned long, drop)
		__field(unsigned long, growth)
		__field(unsigned, span_anon)
		__field(unsigned, span_file)
		__field(bool, walk_pmdp)
		__field(unsigned long, nr_to_reclaim)
	),

	TP_fast_assign(
		__entry->nid = nid;
		__entry->total = total;
		__entry->free = free;
		__entry->drop = drop;
		__entry->growth = growth;
		__entry->span_anon = span_anon;
		__entry->span_file = span_file;
		__entry->walk_pmdp = walk_pmdp;
		__entry->nr_to_reclaim = nr_to_reclaim;
	),

	TP_printk("node %4u; total %12lu; free %12lu; drop %12lu; growth %12lu; span anon %4u, file %4u; PMD walk %d; reclaim target %12lu",
		  __entry->nid, __entry->total, __entry->free, __entry->drop,
		  __entry->growth, __entry->span_anon, __entry->span_file,
		  __entry->walk_pmdp, __entry->nr_to_reclaim)
);
#endif /* _TRACE_VMSCAN_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
