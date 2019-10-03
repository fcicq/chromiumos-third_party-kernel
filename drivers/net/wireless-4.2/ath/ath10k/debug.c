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

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/utsname.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/devcoredump.h>

#include "core.h"
#include "debug.h"
#include "hif.h"
#include "wmi-ops.h"
#include "mac.h"

/* ms */
#define ATH10K_DEBUG_HTT_STATS_INTERVAL 1000

#define ATH10K_FW_CRASH_DUMP_VERSION 1

/**
 * enum ath10k_fw_crash_dump_type - types of data in the dump file
 * @ATH10K_FW_CRASH_DUMP_REGDUMP: Register crash dump in binary format
 */
enum ath10k_fw_crash_dump_type {
	ATH10K_FW_CRASH_DUMP_REGISTERS = 0,

	ATH10K_FW_CRASH_DUMP_MAX,
};

struct ath10k_tlv_dump_data {
	/* see ath10k_fw_crash_dump_type above */
	__le32 type;

	/* in bytes */
	__le32 tlv_len;

	/* pad to 32-bit boundaries as needed */
	u8 tlv_data[];
} __packed;

struct ath10k_dump_file_data {
	/* dump file information */

	/* "ATH10K-FW-DUMP" */
	char df_magic[16];

	__le32 len;

	/* file dump version */
	__le32 version;

	/* some info we can get from ath10k struct that might help */

	u8 uuid[16];

	__le32 chip_id;

	/* 0 for now, in place for later hardware */
	__le32 bus_type;

	__le32 target_version;
	__le32 fw_version_major;
	__le32 fw_version_minor;
	__le32 fw_version_release;
	__le32 fw_version_build;
	__le32 phy_capability;
	__le32 hw_min_tx_power;
	__le32 hw_max_tx_power;
	__le32 ht_cap_info;
	__le32 vht_cap_info;
	__le32 num_rf_chains;

	/* firmware version string */
	char fw_ver[ETHTOOL_FWVERS_LEN];

	/* Kernel related information */

	/* time-of-day stamp */
	__le64 tv_sec;

	/* time-of-day stamp, nano-seconds */
	__le64 tv_nsec;

	/* LINUX_VERSION_CODE */
	__le32 kernel_ver_code;

	/* VERMAGIC_STRING */
	char kernel_ver[64];

	/* room for growth w/out changing binary format */
	u8 unused[128];

	/* struct ath10k_tlv_dump_data + more */
	u8 data[0];
} __packed;

void ath10k_info(struct ath10k *ar, const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	dev_info(ar->dev, "%pV", &vaf);
	trace_ath10k_log_info(ar, &vaf);
	va_end(args);
}
EXPORT_SYMBOL(ath10k_info);

void ath10k_debug_print_hwfw_info(struct ath10k *ar)
{
	char fw_features[128] = {};

	ath10k_core_get_fw_features_str(ar, fw_features, sizeof(fw_features));

	ath10k_info(ar, "%s target 0x%08x chip_id 0x%08x sub %04x:%04x",
		    ar->hw_params.name,
		    ar->target_version,
		    ar->chip_id,
		    ar->id.subsystem_vendor, ar->id.subsystem_device);

	ath10k_info(ar, "kconfig debug %d debugfs %d tracing %d dfs %d testmode %d\n",
		    config_enabled(CONFIG_ATH10K_DEBUG),
		    config_enabled(CONFIG_ATH10K_DEBUGFS),
		    config_enabled(CONFIG_ATH10K_TRACING),
		    config_enabled(CONFIG_ATH10K_DFS_CERTIFIED),
		    config_enabled(CONFIG_NL80211_TESTMODE));

	ath10k_info(ar, "firmware ver %s api %d features %s crc32 %08x\n",
		    ar->hw->wiphy->fw_version,
		    ar->fw_api,
		    fw_features,
		    crc32_le(0, ar->firmware->data, ar->firmware->size));
}

void ath10k_debug_print_board_info(struct ath10k *ar)
{
	char boardinfo[100];

	if (ar->id.bmi_ids_valid)
		scnprintf(boardinfo, sizeof(boardinfo), "%d:%d",
			  ar->id.bmi_chip_id, ar->id.bmi_board_id);
	else
		scnprintf(boardinfo, sizeof(boardinfo), "N/A");

	ath10k_info(ar, "board_file api %d bmi_id %s crc32 %08x",
		    ar->bd_api,
		    boardinfo,
		    crc32_le(0, ar->board->data, ar->board->size));
}

void ath10k_debug_print_boot_info(struct ath10k *ar)
{
	ath10k_info(ar, "htt-ver %d.%d wmi-op %d htt-op %d cal %s max-sta %d raw %d hwcrypto %d\n",
		    ar->htt.target_version_major,
		    ar->htt.target_version_minor,
		    ar->wmi.op_version,
		    ar->htt.op_version,
		    ath10k_cal_mode_str(ar->cal_mode),
		    ar->max_num_stations,
		    test_bit(ATH10K_FLAG_RAW_MODE, &ar->dev_flags),
		    !test_bit(ATH10K_FLAG_HW_CRYPTO_DISABLED, &ar->dev_flags));
}

void ath10k_print_driver_info(struct ath10k *ar)
{
	ath10k_debug_print_hwfw_info(ar);
	ath10k_debug_print_board_info(ar);
	ath10k_debug_print_boot_info(ar);
}
EXPORT_SYMBOL(ath10k_print_driver_info);

void ath10k_err(struct ath10k *ar, const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	dev_err(ar->dev, "%pV", &vaf);
	trace_ath10k_log_err(ar, &vaf);
	va_end(args);
}
EXPORT_SYMBOL(ath10k_err);

void ath10k_warn(struct ath10k *ar, const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	dev_warn_ratelimited(ar->dev, "%pV", &vaf);
	trace_ath10k_log_warn(ar, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(ath10k_warn);

#ifdef CONFIG_ATH10K_DEBUGFS

static ssize_t ath10k_read_wmi_services(struct file *file,
					char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	const char *name;
	ssize_t ret_cnt;
	bool enabled;
	int i;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&ar->conf_mutex);

	if (len > buf_len)
		len = buf_len;

	spin_lock_bh(&ar->data_lock);
	for (i = 0; i < WMI_SERVICE_MAX; i++) {
		enabled = test_bit(i, ar->wmi.svc_map);
		name = wmi_service_name(i);

		if (!name) {
			if (enabled)
				len += scnprintf(buf + len, buf_len - len,
						 "%-40s %s (bit %d)\n",
						 "unknown", "enabled", i);

			continue;
		}

		len += scnprintf(buf + len, buf_len - len,
				 "%-40s %s\n",
				 name, enabled ? "enabled" : "-");
	}
	spin_unlock_bh(&ar->data_lock);

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&ar->conf_mutex);

	kfree(buf);
	return ret_cnt;
}

static const struct file_operations fops_wmi_services = {
	.read = ath10k_read_wmi_services,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static void ath10k_fw_stats_pdevs_free(struct list_head *head)
{
	struct ath10k_fw_stats_pdev *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

static void ath10k_fw_stats_vdevs_free(struct list_head *head)
{
	struct ath10k_fw_stats_vdev *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

static void ath10k_fw_stats_peers_free(struct list_head *head)
{
	struct ath10k_fw_stats_peer *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

static void ath10k_fw_extd_stats_peers_free(struct list_head *head)
{
	struct ath10k_fw_extd_stats_peer *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

static void ath10k_debug_fw_stats_reset(struct ath10k *ar)
{
	spin_lock_bh(&ar->data_lock);
	ar->debug.fw_stats_done = false;
	ath10k_fw_stats_pdevs_free(&ar->debug.fw_stats.pdevs);
	ath10k_fw_stats_vdevs_free(&ar->debug.fw_stats.vdevs);
	ath10k_fw_stats_peers_free(&ar->debug.fw_stats.peers);
	ath10k_fw_extd_stats_peers_free(&ar->debug.fw_stats.peers_extd);
	spin_unlock_bh(&ar->data_lock);
}

void ath10k_debug_fw_stats_process(struct ath10k *ar, struct sk_buff *skb)
{
	struct ath10k_fw_stats stats = {};
	bool is_start, is_started, is_end;
	size_t num_peers;
	size_t num_vdevs;
	int ret;

	INIT_LIST_HEAD(&stats.pdevs);
	INIT_LIST_HEAD(&stats.vdevs);
	INIT_LIST_HEAD(&stats.peers);
	INIT_LIST_HEAD(&stats.peers_extd);

	spin_lock_bh(&ar->data_lock);
	ret = ath10k_wmi_pull_fw_stats(ar, skb, &stats);
	if (ret) {
		ath10k_warn(ar, "failed to pull fw stats: %d\n", ret);
		goto free;
	}

	/* Stat data may exceed htc-wmi buffer limit. In such case firmware
	 * splits the stats data and delivers it in a ping-pong fashion of
	 * request cmd-update event.
	 *
	 * However there is no explicit end-of-data. Instead start-of-data is
	 * used as an implicit one. This works as follows:
	 *  a) discard stat update events until one with pdev stats is
	 *     delivered - this skips session started at end of (b)
	 *  b) consume stat update events until another one with pdev stats is
	 *     delivered which is treated as end-of-data and is itself discarded
	 */
	if (ath10k_peer_stats_enabled(ar))
		ath10k_sta_update_rx_duration(ar, &stats);

	if (ar->debug.fw_stats_done) {
		if (!ath10k_peer_stats_enabled(ar))
			ath10k_warn(ar, "received unsolicited stats update event\n");

		goto free;
	}

	num_peers = ath10k_wmi_fw_stats_num_peers(&ar->debug.fw_stats.peers);
	num_vdevs = ath10k_wmi_fw_stats_num_vdevs(&ar->debug.fw_stats.vdevs);
	is_start = (list_empty(&ar->debug.fw_stats.pdevs) &&
		    !list_empty(&stats.pdevs));
	is_end = (!list_empty(&ar->debug.fw_stats.pdevs) &&
		  !list_empty(&stats.pdevs));

	if (is_start)
		list_splice_tail_init(&stats.pdevs, &ar->debug.fw_stats.pdevs);

	if (is_end)
		ar->debug.fw_stats_done = true;

	is_started = !list_empty(&ar->debug.fw_stats.pdevs);

	if (is_started && !is_end) {
		if (num_peers >= ATH10K_MAX_NUM_PEER_IDS) {
			/* Although this is unlikely impose a sane limit to
			 * prevent firmware from DoS-ing the host.
			 */
			ath10k_fw_stats_peers_free(&ar->debug.fw_stats.peers);
			ath10k_warn(ar, "dropping fw peer stats\n");
			goto free;
		}

		if (num_vdevs >= BITS_PER_LONG) {
			ath10k_fw_stats_vdevs_free(&ar->debug.fw_stats.vdevs);
			ath10k_warn(ar, "dropping fw vdev stats\n");
			goto free;
		}

		list_splice_tail_init(&stats.peers, &ar->debug.fw_stats.peers);
		list_splice_tail_init(&stats.peers_extd,
				      &ar->debug.fw_stats.peers_extd);
		list_splice_tail_init(&stats.vdevs, &ar->debug.fw_stats.vdevs);
	}

	complete(&ar->debug.fw_stats_complete);

free:
	/* In some cases lists have been spliced and cleared. Free up
	 * resources if that is not the case.
	 */
	ath10k_fw_stats_pdevs_free(&stats.pdevs);
	ath10k_fw_stats_vdevs_free(&stats.vdevs);
	ath10k_fw_stats_peers_free(&stats.peers);
	ath10k_fw_extd_stats_peers_free(&stats.peers_extd);

	spin_unlock_bh(&ar->data_lock);
}

static int ath10k_debug_fw_stats_request(struct ath10k *ar)
{
	unsigned long timeout, time_left;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	timeout = jiffies + msecs_to_jiffies(1 * HZ);

	ath10k_debug_fw_stats_reset(ar);

	for (;;) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		reinit_completion(&ar->debug.fw_stats_complete);

		ret = ath10k_wmi_request_stats(ar, ar->fw_stats_req_mask);
		if (ret) {
			ath10k_warn(ar, "could not request stats (%d)\n", ret);
			return ret;
		}

		time_left =
		wait_for_completion_timeout(&ar->debug.fw_stats_complete,
					    1 * HZ);
		if (!time_left)
			return -ETIMEDOUT;

		spin_lock_bh(&ar->data_lock);
		if (ar->debug.fw_stats_done) {
			spin_unlock_bh(&ar->data_lock);
			break;
		}
		spin_unlock_bh(&ar->data_lock);
	}

	return 0;
}

static int ath10k_fw_stats_open(struct inode *inode, struct file *file)
{
	struct ath10k *ar = inode->i_private;
	void *buf = NULL;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto err_unlock;
	}

	buf = vmalloc(ATH10K_FW_STATS_BUF_SIZE);
	if (!buf) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	ret = ath10k_debug_fw_stats_request(ar);
	if (ret) {
		ath10k_warn(ar, "failed to request fw stats: %d\n", ret);
		goto err_free;
	}

	ret = ath10k_wmi_fw_stats_fill(ar, &ar->debug.fw_stats, buf);
	if (ret) {
		ath10k_warn(ar, "failed to fill fw stats: %d\n", ret);
		goto err_free;
	}

	file->private_data = buf;

	mutex_unlock(&ar->conf_mutex);
	return 0;

err_free:
	vfree(buf);

err_unlock:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static int ath10k_fw_stats_release(struct inode *inode, struct file *file)
{
	vfree(file->private_data);

	return 0;
}

static ssize_t ath10k_fw_stats_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	const char *buf = file->private_data;
	unsigned int len = strlen(buf);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_fw_stats = {
	.open = ath10k_fw_stats_open,
	.release = ath10k_fw_stats_release,
	.read = ath10k_fw_stats_read,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_debug_fw_reset_stats_read(struct file *file,
						char __user *user_buf,
						size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret, len, buf_len;
	char *buf;

	buf_len = 500;
	buf = kmalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	spin_lock_bh(&ar->data_lock);

	len = 0;
	len += scnprintf(buf + len, buf_len - len,
			 "fw_crash_counter\t\t%d\n", ar->stats.fw_crash_counter);
	len += scnprintf(buf + len, buf_len - len,
			 "fw_warm_reset_counter\t\t%d\n",
			 ar->stats.fw_warm_reset_counter);
	len += scnprintf(buf + len, buf_len - len,
			 "fw_cold_reset_counter\t\t%d\n",
			 ar->stats.fw_cold_reset_counter);

	spin_unlock_bh(&ar->data_lock);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);

	return ret;
}

static const struct file_operations fops_fw_reset_stats = {
	.open = simple_open,
	.read = ath10k_debug_fw_reset_stats_read,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

/* This is a clean assert crash in firmware. */
static int ath10k_debug_fw_assert(struct ath10k *ar)
{
	struct wmi_vdev_install_key_cmd *cmd;
	struct sk_buff *skb;

	skb = ath10k_wmi_alloc_skb(ar, sizeof(*cmd) + 16);
	if (!skb)
		return -ENOMEM;

	cmd = (struct wmi_vdev_install_key_cmd *)skb->data;
	memset(cmd, 0, sizeof(*cmd));

	/* big enough number so that firmware asserts */
	cmd->vdev_id = __cpu_to_le32(0x7ffe);

	return ath10k_wmi_cmd_send(ar, skb,
				   ar->wmi.cmd->vdev_install_key_cmdid);
}

static ssize_t ath10k_read_simulate_fw_crash(struct file *file,
					     char __user *user_buf,
					     size_t count, loff_t *ppos)
{
	const char buf[] =
		"To simulate firmware crash write one of the keywords to this file:\n"
		"`soft` - this will send WMI_FORCE_FW_HANG_ASSERT to firmware if FW supports that command.\n"
		"`hard` - this will send to firmware command with illegal parameters causing firmware crash.\n"
		"`assert` - this will send special illegal parameter to firmware to cause assert failure and crash.\n"
		"`hw-restart` - this will simply queue hw restart without fw/hw actually crashing.\n";

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}

/* Simulate firmware crash:
 * 'soft': Call wmi command causing firmware hang. This firmware hang is
 * recoverable by warm firmware reset.
 * 'hard': Force firmware crash by setting any vdev parameter for not allowed
 * vdev id. This is hard firmware crash because it is recoverable only by cold
 * firmware reset.
 */
static ssize_t ath10k_write_simulate_fw_crash(struct file *file,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	int ret;

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = 0;

	/* drop the possible '\n' from the end */
	if (buf[count - 1] == '\n')
		buf[count - 1] = 0;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_RESTARTED) {
		ret = -ENETDOWN;
		goto exit;
	}

	if (!strcmp(buf, "soft")) {
		ath10k_info(ar, "simulating soft firmware crash\n");
		ret = ath10k_wmi_force_fw_hang(ar, WMI_FORCE_FW_HANG_ASSERT, 0);
	} else if (!strcmp(buf, "hard")) {
		ath10k_info(ar, "simulating hard firmware crash\n");
		/* 0x7fff is vdev id, and it is always out of range for all
		 * firmware variants in order to force a firmware crash.
		 */
		ret = ath10k_wmi_vdev_set_param(ar, 0x7fff,
						ar->wmi.vdev_param->rts_threshold,
						0);
	} else if (!strcmp(buf, "assert")) {
		ath10k_info(ar, "simulating firmware assert crash\n");
		ret = ath10k_debug_fw_assert(ar);
	} else if (!strcmp(buf, "hw-restart")) {
		ath10k_info(ar, "user requested hw restart\n");
		queue_work(ar->workqueue, &ar->restart_work);
		ret = 0;
	} else {
		ret = -EINVAL;
		goto exit;
	}

	if (ret) {
		ath10k_warn(ar, "failed to simulate firmware crash: %d\n", ret);
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_simulate_fw_crash = {
	.read = ath10k_read_simulate_fw_crash,
	.write = ath10k_write_simulate_fw_crash,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_chip_id(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned int len;
	char buf[50];

	len = scnprintf(buf, sizeof(buf), "0x%08x\n", ar->chip_id);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_chip_id = {
	.read = ath10k_read_chip_id,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

struct ath10k_fw_crash_data *
ath10k_debug_get_new_fw_crash_data(struct ath10k *ar)
{
	struct ath10k_fw_crash_data *crash_data = ar->debug.fw_crash_data;

	lockdep_assert_held(&ar->data_lock);

	crash_data->crashed_since_read = true;
	uuid_le_gen(&crash_data->uuid);
	getnstimeofday(&crash_data->timestamp);

	return crash_data;
}
EXPORT_SYMBOL(ath10k_debug_get_new_fw_crash_data);

static struct ath10k_dump_file_data *ath10k_build_dump_file(struct ath10k *ar,
							    bool mark_read)
{
	struct ath10k_fw_crash_data *crash_data = ar->debug.fw_crash_data;
	struct ath10k_dump_file_data *dump_data;
	struct ath10k_tlv_dump_data *dump_tlv;
	int hdr_len = sizeof(*dump_data);
	unsigned int len, sofar = 0;
	unsigned char *buf;

	len = hdr_len;
	len += sizeof(*dump_tlv) + sizeof(crash_data->registers);

	sofar += hdr_len;

	/* This is going to get big when we start dumping FW RAM and such,
	 * so go ahead and use vmalloc.
	 */
	buf = vzalloc(len);
	if (!buf)
		return NULL;

	spin_lock_bh(&ar->data_lock);

	if (!crash_data->crashed_since_read) {
		spin_unlock_bh(&ar->data_lock);
		vfree(buf);
		return NULL;
	}

	dump_data = (struct ath10k_dump_file_data *)(buf);
	strlcpy(dump_data->df_magic, "ATH10K-FW-DUMP",
		sizeof(dump_data->df_magic));
	dump_data->len = cpu_to_le32(len);

	dump_data->version = cpu_to_le32(ATH10K_FW_CRASH_DUMP_VERSION);

	memcpy(dump_data->uuid, &crash_data->uuid, sizeof(dump_data->uuid));
	dump_data->chip_id = cpu_to_le32(ar->chip_id);
	dump_data->bus_type = cpu_to_le32(0);
	dump_data->target_version = cpu_to_le32(ar->target_version);
	dump_data->fw_version_major = cpu_to_le32(ar->fw_version_major);
	dump_data->fw_version_minor = cpu_to_le32(ar->fw_version_minor);
	dump_data->fw_version_release = cpu_to_le32(ar->fw_version_release);
	dump_data->fw_version_build = cpu_to_le32(ar->fw_version_build);
	dump_data->phy_capability = cpu_to_le32(ar->phy_capability);
	dump_data->hw_min_tx_power = cpu_to_le32(ar->hw_min_tx_power);
	dump_data->hw_max_tx_power = cpu_to_le32(ar->hw_max_tx_power);
	dump_data->ht_cap_info = cpu_to_le32(ar->ht_cap_info);
	dump_data->vht_cap_info = cpu_to_le32(ar->vht_cap_info);
	dump_data->num_rf_chains = cpu_to_le32(ar->num_rf_chains);

	strlcpy(dump_data->fw_ver, ar->hw->wiphy->fw_version,
		sizeof(dump_data->fw_ver));

	dump_data->kernel_ver_code = 0;
	strlcpy(dump_data->kernel_ver, init_utsname()->release,
		sizeof(dump_data->kernel_ver));

	dump_data->tv_sec = cpu_to_le64(crash_data->timestamp.tv_sec);
	dump_data->tv_nsec = cpu_to_le64(crash_data->timestamp.tv_nsec);

	/* Gather crash-dump */
	dump_tlv = (struct ath10k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH10K_FW_CRASH_DUMP_REGISTERS);
	dump_tlv->tlv_len = cpu_to_le32(sizeof(crash_data->registers));
	memcpy(dump_tlv->tlv_data, &crash_data->registers,
	       sizeof(crash_data->registers));
	sofar += sizeof(*dump_tlv) + sizeof(crash_data->registers);

	ar->debug.fw_crash_data->crashed_since_read = !mark_read;

	spin_unlock_bh(&ar->data_lock);

	return dump_data;
}

int ath10k_debug_fw_devcoredump(struct ath10k *ar)
{
	struct ath10k_dump_file_data *dump;
	void *dump_ptr;
	u32 dump_len;

	/* To keep the dump file available also for debugfs don't mark the
	 * file read, only debugfs should do that.
	 */
	dump = ath10k_build_dump_file(ar, false);
	if (!dump) {
		ath10k_warn(ar, "no crash dump data found for devcoredump");
		return -ENODATA;
	}

	/* Make a copy of the dump file for dev_coredumpv() as during the
	 * transition period we need to own the original file. Once
	 * fw_crash_dump debugfs file is removed no need to have a copy
	 * anymore.
	 */
	dump_len = le32_to_cpu(dump->len);
	dump_ptr = vzalloc(dump_len);

	if (!dump_ptr)
		return -ENOMEM;

	memcpy(dump_ptr, dump, dump_len);

	dev_coredumpv(ar->dev, dump_ptr, dump_len, GFP_KERNEL);

	return 0;
}

static int ath10k_fw_crash_dump_open(struct inode *inode, struct file *file)
{
	struct ath10k *ar = inode->i_private;
	struct ath10k_dump_file_data *dump;

	ath10k_warn(ar, "fw_crash_dump debugfs file is deprecated, please use /sys/class/devcoredump instead.");

	dump = ath10k_build_dump_file(ar, true);
	if (!dump)
		return -ENODATA;

	file->private_data = dump;

	return 0;
}

static ssize_t ath10k_fw_crash_dump_read(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct ath10k_dump_file_data *dump_file = file->private_data;

	return simple_read_from_buffer(user_buf, count, ppos,
				       dump_file,
				       le32_to_cpu(dump_file->len));
}

static int ath10k_fw_crash_dump_release(struct inode *inode,
					struct file *file)
{
	vfree(file->private_data);

	return 0;
}

static const struct file_operations fops_fw_crash_dump = {
	.open = ath10k_fw_crash_dump_open,
	.read = ath10k_fw_crash_dump_read,
	.release = ath10k_fw_crash_dump_release,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_reg_addr_read(struct file *file,
				    char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 buf[32];
	unsigned int len = 0;
	u32 reg_addr;

	mutex_lock(&ar->conf_mutex);
	reg_addr = ar->debug.reg_addr;
	mutex_unlock(&ar->conf_mutex);

	len += scnprintf(buf + len, sizeof(buf) - len, "0x%x\n", reg_addr);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_reg_addr_write(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 reg_addr;
	int ret;

	ret = kstrtou32_from_user(user_buf, count, 0, &reg_addr);
	if (ret)
		return ret;

	if (!IS_ALIGNED(reg_addr, 4))
		return -EFAULT;

	mutex_lock(&ar->conf_mutex);
	ar->debug.reg_addr = reg_addr;
	mutex_unlock(&ar->conf_mutex);

	return count;
}

static const struct file_operations fops_reg_addr = {
	.read = ath10k_reg_addr_read,
	.write = ath10k_reg_addr_write,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_reg_value_read(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 buf[48];
	unsigned int len;
	u32 reg_addr, reg_val;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_UTF) {
		ret = -ENETDOWN;
		goto exit;
	}

	reg_addr = ar->debug.reg_addr;

	reg_val = ath10k_hif_read32(ar, reg_addr);
	len = scnprintf(buf, sizeof(buf), "0x%08x:0x%08x\n", reg_addr, reg_val);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_reg_value_write(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 reg_addr, reg_val;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_UTF) {
		ret = -ENETDOWN;
		goto exit;
	}

	reg_addr = ar->debug.reg_addr;

	ret = kstrtou32_from_user(user_buf, count, 0, &reg_val);
	if (ret)
		goto exit;

	ath10k_hif_write32(ar, reg_addr, reg_val);

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static const struct file_operations fops_reg_value = {
	.read = ath10k_reg_value_read,
	.write = ath10k_reg_value_write,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_mem_value_read(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 *buf;
	int ret;

	if (*ppos < 0)
		return -EINVAL;

	if (!count)
		return 0;

	mutex_lock(&ar->conf_mutex);

	buf = vmalloc(count);
	if (!buf) {
		ret = -ENOMEM;
		goto exit;
	}

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_UTF) {
		ret = -ENETDOWN;
		goto exit;
	}

	ret = ath10k_hif_diag_read(ar, *ppos, buf, count);
	if (ret) {
		ath10k_warn(ar, "failed to read address 0x%08x via diagnose window fnrom debugfs: %d\n",
			    (u32)(*ppos), ret);
		goto exit;
	}

	ret = copy_to_user(user_buf, buf, count);
	if (ret) {
		ret = -EFAULT;
		goto exit;
	}

	count -= ret;
	*ppos += count;
	ret = count;

exit:
	vfree(buf);
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_mem_value_write(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 *buf;
	int ret;

	if (*ppos < 0)
		return -EINVAL;

	if (!count)
		return 0;

	mutex_lock(&ar->conf_mutex);

	buf = vmalloc(count);
	if (!buf) {
		ret = -ENOMEM;
		goto exit;
	}

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_UTF) {
		ret = -ENETDOWN;
		goto exit;
	}

	ret = copy_from_user(buf, user_buf, count);
	if (ret) {
		ret = -EFAULT;
		goto exit;
	}

	ret = ath10k_hif_diag_write(ar, *ppos, buf, count);
	if (ret) {
		ath10k_warn(ar, "failed to write address 0x%08x via diagnose window from debugfs: %d\n",
			    (u32)(*ppos), ret);
		goto exit;
	}

	*ppos += count;
	ret = count;

exit:
	vfree(buf);
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static const struct file_operations fops_mem_value = {
	.read = ath10k_mem_value_read,
	.write = ath10k_mem_value_write,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int ath10k_debug_htt_stats_req(struct ath10k *ar)
{
	u64 cookie;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	if (ar->debug.htt_stats_mask == 0)
		/* htt stats are disabled */
		return 0;

	if (ar->state != ATH10K_STATE_ON)
		return 0;

	cookie = get_jiffies_64();

	ret = ath10k_htt_h2t_stats_req(&ar->htt, ar->debug.htt_stats_mask,
				       cookie);
	if (ret) {
		ath10k_warn(ar, "failed to send htt stats request: %d\n", ret);
		return ret;
	}

	queue_delayed_work(ar->workqueue, &ar->debug.htt_stats_dwork,
			   msecs_to_jiffies(ATH10K_DEBUG_HTT_STATS_INTERVAL));

	return 0;
}

static void ath10k_debug_htt_stats_dwork(struct work_struct *work)
{
	struct ath10k *ar = container_of(work, struct ath10k,
					 debug.htt_stats_dwork.work);

	mutex_lock(&ar->conf_mutex);

	ath10k_debug_htt_stats_req(ar);

	mutex_unlock(&ar->conf_mutex);
}

static ssize_t ath10k_read_htt_stats_mask(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	unsigned int len;

	len = scnprintf(buf, sizeof(buf), "%lu\n", ar->debug.htt_stats_mask);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_write_htt_stats_mask(struct file *file,
					   const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned long mask;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &mask);
	if (ret)
		return ret;

	/* max 17 bit masks (for now) */
	if (mask > 0x1ffff)
		return -E2BIG;

	mutex_lock(&ar->conf_mutex);

	ar->debug.htt_stats_mask = mask;

	ret = ath10k_debug_htt_stats_req(ar);
	if (ret)
		goto out;

	ret = count;

out:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static const struct file_operations fops_htt_stats_mask = {
	.read = ath10k_read_htt_stats_mask,
	.write = ath10k_write_htt_stats_mask,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_warm_hw_reset(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	size_t buf_size;
	int ret;
	bool val;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	buf[buf_size] = '\0';

	if (strtobool(buf, &val) != 0)
		return -EINVAL;

	if (!val)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto exit;
	}

	if ((ar->wmi.op_version !=
	     ATH10K_FW_WMI_OP_VERSION_10_4) &&
	     (!test_bit(WMI_SERVICE_RESET_CHIP, ar->wmi.svc_map))) {
		ath10k_warn(ar, "failed to map wmi service\n");
		ret = -ENOTSUPP;
		goto exit;
	}

	ret = ath10k_wmi_pdev_set_param(ar, ar->wmi.pdev_param->pdev_reset,
					WMI_RST_MODE_WARM_RESET);

	if (ret) {
		ath10k_warn(ar, "failed to warm hw reset: %d\n", ret);
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static const struct file_operations fops_warm_hw_reset = {
	.write = ath10k_write_warm_hw_reset,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_reset_htt_stats(struct file *file,
					    const char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned long reset;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &reset);
	if (ret)
		return ret;

	if (reset == 0 || reset > 0x1ffff)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	ar->debug.reset_htt_stats = reset;

	ret = ath10k_debug_htt_stats_req(ar);
	if (ret)
		goto out;

	ret = count;

out:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static const struct file_operations fops_reset_htt_stats = {
	.write = ath10k_write_reset_htt_stats,
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_htt_max_amsdu_ampdu(struct file *file,
					       char __user *user_buf,
					       size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[64];
	u8 amsdu, ampdu;
	unsigned int len;

	mutex_lock(&ar->conf_mutex);

	amsdu = ar->htt.max_num_amsdu;
	ampdu = ar->htt.max_num_ampdu;
	mutex_unlock(&ar->conf_mutex);

	len = scnprintf(buf, sizeof(buf), "%u %u\n", amsdu, ampdu);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_write_htt_max_amsdu_ampdu(struct file *file,
						const char __user *user_buf,
						size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int res;
	char buf[64];
	unsigned int amsdu, ampdu;

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = 0;

	res = sscanf(buf, "%u %u", &amsdu, &ampdu);

	if (res != 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	res = ath10k_htt_h2t_aggr_cfg_msg(&ar->htt, ampdu, amsdu);
	if (res)
		goto out;

	res = count;
	ar->htt.max_num_amsdu = amsdu;
	ar->htt.max_num_ampdu = ampdu;

out:
	mutex_unlock(&ar->conf_mutex);
	return res;
}

static const struct file_operations fops_htt_max_amsdu_ampdu = {
	.read = ath10k_read_htt_max_amsdu_ampdu,
	.write = ath10k_write_htt_max_amsdu_ampdu,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_fw_dbglog(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned int len;
	char buf[96];

	len = scnprintf(buf, sizeof(buf), "0x%16llx %u\n",
			ar->debug.fw_dbglog_mask, ar->debug.fw_dbglog_level);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_write_fw_dbglog(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret;
	char buf[96];
	unsigned int log_level;
	u64 mask;

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = 0;

	ret = sscanf(buf, "%llx %u", &mask, &log_level);

	if (!ret)
		return -EINVAL;

	if (ret == 1)
		/* default if user did not specify */
		log_level = ATH10K_DBGLOG_LEVEL_WARN;

	mutex_lock(&ar->conf_mutex);

	ar->debug.fw_dbglog_mask = mask;
	ar->debug.fw_dbglog_level = log_level;

	if (ar->state == ATH10K_STATE_ON) {
		ret = ath10k_wmi_dbglog_cfg(ar, ar->debug.fw_dbglog_mask,
					    ar->debug.fw_dbglog_level);
		if (ret) {
			ath10k_warn(ar, "dbglog cfg failed from debugfs: %d\n",
				    ret);
			goto exit;
		}
	}

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

/* TODO:  Would be nice to always support ethtool stats, would need to
 * move the stats storage out of ath10k_debug, or always have ath10k_debug
 * struct available..
 */

/* This generally cooresponds to the debugfs fw_stats file */
static const char ath10k_gstrings_stats[][ETH_GSTRING_LEN] = {
	"tx_pkts_nic",
	"tx_bytes_nic",
	"rx_pkts_nic",
	"rx_bytes_nic",
	"d_noise_floor",
	"d_cycle_count",
	"d_phy_error",
	"d_rts_bad",
	"d_rts_good",
	"d_tx_power", /* in .5 dbM I think */
	"d_rx_crc_err", /* fcs_bad */
	"d_no_beacon",
	"d_tx_mpdus_queued",
	"d_tx_msdu_queued",
	"d_tx_msdu_dropped",
	"d_local_enqued",
	"d_local_freed",
	"d_tx_ppdu_hw_queued",
	"d_tx_ppdu_reaped",
	"d_tx_fifo_underrun",
	"d_tx_ppdu_abort",
	"d_tx_mpdu_requed",
	"d_tx_excessive_retries",
	"d_tx_hw_rate",
	"d_tx_dropped_sw_retries",
	"d_tx_illegal_rate",
	"d_tx_continuous_xretries",
	"d_tx_timeout",
	"d_tx_mpdu_txop_limit",
	"d_pdev_resets",
	"d_rx_mid_ppdu_route_change",
	"d_rx_status",
	"d_rx_extra_frags_ring0",
	"d_rx_extra_frags_ring1",
	"d_rx_extra_frags_ring2",
	"d_rx_extra_frags_ring3",
	"d_rx_msdu_htt",
	"d_rx_mpdu_htt",
	"d_rx_msdu_stack",
	"d_rx_mpdu_stack",
	"d_rx_phy_err",
	"d_rx_phy_err_drops",
	"d_rx_mpdu_errors", /* FCS, MIC, ENC */
	"d_fw_crash_count",
	"d_fw_warm_reset_count",
	"d_fw_cold_reset_count",
};

#define ATH10K_SSTATS_LEN ARRAY_SIZE(ath10k_gstrings_stats)

void ath10k_debug_get_et_strings(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 u32 sset, u8 *data)
{
	if (sset == ETH_SS_STATS)
		memcpy(data, *ath10k_gstrings_stats,
		       sizeof(ath10k_gstrings_stats));
}

int ath10k_debug_get_et_sset_count(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif, int sset)
{
	if (sset == ETH_SS_STATS)
		return ATH10K_SSTATS_LEN;

	return 0;
}

void ath10k_debug_get_et_stats(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ethtool_stats *stats, u64 *data)
{
	struct ath10k *ar = hw->priv;
	static const struct ath10k_fw_stats_pdev zero_stats = {};
	const struct ath10k_fw_stats_pdev *pdev_stats;
	int i = 0, ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state == ATH10K_STATE_ON) {
		ret = ath10k_debug_fw_stats_request(ar);
		if (ret) {
			/* just print a warning and try to use older results */
			ath10k_warn(ar,
				    "failed to get fw stats for ethtool: %d\n",
				    ret);
		}
	}

	pdev_stats = list_first_entry_or_null(&ar->debug.fw_stats.pdevs,
					      struct ath10k_fw_stats_pdev,
					      list);
	if (!pdev_stats) {
		/* no results available so just return zeroes */
		pdev_stats = &zero_stats;
	}

	spin_lock_bh(&ar->data_lock);

	data[i++] = pdev_stats->hw_reaped; /* ppdu reaped */
	data[i++] = 0; /* tx bytes */
	data[i++] = pdev_stats->htt_mpdus;
	data[i++] = 0; /* rx bytes */
	data[i++] = pdev_stats->ch_noise_floor;
	data[i++] = pdev_stats->cycle_count;
	data[i++] = pdev_stats->phy_err_count;
	data[i++] = pdev_stats->rts_bad;
	data[i++] = pdev_stats->rts_good;
	data[i++] = pdev_stats->chan_tx_power;
	data[i++] = pdev_stats->fcs_bad;
	data[i++] = pdev_stats->no_beacons;
	data[i++] = pdev_stats->mpdu_enqued;
	data[i++] = pdev_stats->msdu_enqued;
	data[i++] = pdev_stats->wmm_drop;
	data[i++] = pdev_stats->local_enqued;
	data[i++] = pdev_stats->local_freed;
	data[i++] = pdev_stats->hw_queued;
	data[i++] = pdev_stats->hw_reaped;
	data[i++] = pdev_stats->underrun;
	data[i++] = pdev_stats->tx_abort;
	data[i++] = pdev_stats->mpdus_requed;
	data[i++] = pdev_stats->tx_ko;
	data[i++] = pdev_stats->data_rc;
	data[i++] = pdev_stats->sw_retry_failure;
	data[i++] = pdev_stats->illgl_rate_phy_err;
	data[i++] = pdev_stats->pdev_cont_xretry;
	data[i++] = pdev_stats->pdev_tx_timeout;
	data[i++] = pdev_stats->txop_ovf;
	data[i++] = pdev_stats->pdev_resets;
	data[i++] = pdev_stats->mid_ppdu_route_change;
	data[i++] = pdev_stats->status_rcvd;
	data[i++] = pdev_stats->r0_frags;
	data[i++] = pdev_stats->r1_frags;
	data[i++] = pdev_stats->r2_frags;
	data[i++] = pdev_stats->r3_frags;
	data[i++] = pdev_stats->htt_msdus;
	data[i++] = pdev_stats->htt_mpdus;
	data[i++] = pdev_stats->loc_msdus;
	data[i++] = pdev_stats->loc_mpdus;
	data[i++] = pdev_stats->phy_errs;
	data[i++] = pdev_stats->phy_err_drop;
	data[i++] = pdev_stats->mpdu_errs;
	data[i++] = ar->stats.fw_crash_counter;
	data[i++] = ar->stats.fw_warm_reset_counter;
	data[i++] = ar->stats.fw_cold_reset_counter;

	spin_unlock_bh(&ar->data_lock);

	mutex_unlock(&ar->conf_mutex);

	WARN_ON(i != ATH10K_SSTATS_LEN);
}

static const struct file_operations fops_fw_dbglog = {
	.read = ath10k_read_fw_dbglog,
	.write = ath10k_write_fw_dbglog,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int ath10k_debug_cal_data_open(struct inode *inode, struct file *file)
{
	struct ath10k *ar = inode->i_private;
	void *buf;
	u32 hi_addr;
	__le32 addr;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_UTF) {
		ret = -ENETDOWN;
		goto err;
	}

	buf = vmalloc(ar->hw_params.cal_data_len);
	if (!buf) {
		ret = -ENOMEM;
		goto err;
	}

	hi_addr = host_interest_item_address(HI_ITEM(hi_board_data));

	ret = ath10k_hif_diag_read(ar, hi_addr, &addr, sizeof(addr));
	if (ret) {
		ath10k_warn(ar, "failed to read hi_board_data address: %d\n", ret);
		goto err_vfree;
	}

	ret = ath10k_hif_diag_read(ar, le32_to_cpu(addr), buf,
				   ar->hw_params.cal_data_len);
	if (ret) {
		ath10k_warn(ar, "failed to read calibration data: %d\n", ret);
		goto err_vfree;
	}

	file->private_data = buf;

	mutex_unlock(&ar->conf_mutex);

	return 0;

err_vfree:
	vfree(buf);

err:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_debug_cal_data_read(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	void *buf = file->private_data;

	return simple_read_from_buffer(user_buf, count, ppos,
				       buf, ar->hw_params.cal_data_len);
}

static int ath10k_debug_cal_data_release(struct inode *inode,
					 struct file *file)
{
	vfree(file->private_data);

	return 0;
}

static ssize_t ath10k_write_ani_enable(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret;
	u8 enable;

	if (kstrtou8_from_user(user_buf, count, 0, &enable))
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->ani_enabled == enable) {
		ret = count;
		goto exit;
	}

	ret = ath10k_wmi_pdev_set_param(ar, ar->wmi.pdev_param->ani_enable,
					enable);
	if (ret) {
		ath10k_warn(ar, "ani_enable failed from debugfs: %d\n", ret);
		goto exit;
	}
	ar->ani_enabled = enable;

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_read_ani_enable(struct file *file, char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[32];

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->ani_enabled);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_ani_enable = {
	.read = ath10k_read_ani_enable,
	.write = ath10k_write_ani_enable,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static const struct file_operations fops_cal_data = {
	.open = ath10k_debug_cal_data_open,
	.read = ath10k_debug_cal_data_read,
	.release = ath10k_debug_cal_data_release,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_nf_cal_period(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned int len;
	char buf[32];

	len = scnprintf(buf, sizeof(buf), "%d\n",
			ar->debug.nf_cal_period);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ath10k_write_nf_cal_period(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned long period;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &period);
	if (ret)
		return ret;

	if (period > WMI_PDEV_PARAM_CAL_PERIOD_MAX)
		return -EINVAL;

	/* there's no way to switch back to the firmware default */
	if (period == 0)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	ar->debug.nf_cal_period = period;

	if (ar->state != ATH10K_STATE_ON) {
		/* firmware is not running, nothing else to do */
		ret = count;
		goto exit;
	}

	ret = ath10k_wmi_pdev_set_param(ar, ar->wmi.pdev_param->cal_period,
					ar->debug.nf_cal_period);
	if (ret) {
		ath10k_warn(ar, "cal period cfg failed from debugfs: %d\n",
			    ret);
		goto exit;
	}

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static const struct file_operations fops_nf_cal_period = {
	.read = ath10k_read_nf_cal_period,
	.write = ath10k_write_nf_cal_period,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define ATH10K_TPC_CONFIG_BUF_SIZE	(1024 * 1024)

static int ath10k_debug_tpc_stats_request(struct ath10k *ar)
{
	int ret;
	unsigned long time_left;

	lockdep_assert_held(&ar->conf_mutex);

	reinit_completion(&ar->debug.tpc_complete);

	ret = ath10k_wmi_pdev_get_tpc_config(ar, WMI_TPC_CONFIG_PARAM);
	if (ret) {
		ath10k_warn(ar, "failed to request tpc config: %d\n", ret);
		return ret;
	}

	time_left = wait_for_completion_timeout(&ar->debug.tpc_complete,
						1 * HZ);
	if (time_left == 0)
		return -ETIMEDOUT;

	return 0;
}

void ath10k_debug_tpc_stats_process(struct ath10k *ar,
				    struct ath10k_tpc_stats *tpc_stats)
{
	spin_lock_bh(&ar->data_lock);

	kfree(ar->debug.tpc_stats);
	ar->debug.tpc_stats = tpc_stats;
	complete(&ar->debug.tpc_complete);

	spin_unlock_bh(&ar->data_lock);
}

static void ath10k_tpc_stats_print(struct ath10k_tpc_stats *tpc_stats,
				   unsigned int j, char *buf, unsigned int *len)
{
	unsigned int i, buf_len;
	static const char table_str[][5] = { "CDD",
					     "STBC",
					     "TXBF" };
	static const char pream_str[][6] = { "CCK",
					     "OFDM",
					     "HT20",
					     "HT40",
					     "VHT20",
					     "VHT40",
					     "VHT80",
					     "HTCUP" };

	buf_len = ATH10K_TPC_CONFIG_BUF_SIZE;
	*len += scnprintf(buf + *len, buf_len - *len,
			  "********************************\n");
	*len += scnprintf(buf + *len, buf_len - *len,
			  "******************* %s POWER TABLE ****************\n",
			  table_str[j]);
	*len += scnprintf(buf + *len, buf_len - *len,
			  "********************************\n");
	*len += scnprintf(buf + *len, buf_len - *len,
			  "No.  Preamble Rate_code tpc_value1 tpc_value2 tpc_value3\n");

	for (i = 0; i < tpc_stats->rate_max; i++) {
		*len += scnprintf(buf + *len, buf_len - *len,
				  "%8d %s 0x%2x %s\n", i,
				  pream_str[tpc_stats->tpc_table[j].pream_idx[i]],
				  tpc_stats->tpc_table[j].rate_code[i],
				  tpc_stats->tpc_table[j].tpc_value[i]);
	}

	*len += scnprintf(buf + *len, buf_len - *len,
			  "***********************************\n");
}

static void ath10k_tpc_stats_fill(struct ath10k *ar,
				  struct ath10k_tpc_stats *tpc_stats,
				  char *buf)
{
	unsigned int len, j, buf_len;

	len = 0;
	buf_len = ATH10K_TPC_CONFIG_BUF_SIZE;

	spin_lock_bh(&ar->data_lock);

	if (!tpc_stats) {
		ath10k_warn(ar, "failed to get tpc stats\n");
		goto unlock;
	}

	len += scnprintf(buf + len, buf_len - len, "\n");
	len += scnprintf(buf + len, buf_len - len,
			 "*************************************\n");
	len += scnprintf(buf + len, buf_len - len,
			 "TPC config for channel %4d mode %d\n",
			 tpc_stats->chan_freq,
			 tpc_stats->phy_mode);
	len += scnprintf(buf + len, buf_len - len,
			 "*************************************\n");
	len += scnprintf(buf + len, buf_len - len,
			 "CTL		=  0x%2x Reg. Domain		= %2d\n",
			 tpc_stats->ctl,
			 tpc_stats->reg_domain);
	len += scnprintf(buf + len, buf_len - len,
			 "Antenna Gain	= %2d Reg. Max Antenna Gain	=  %2d\n",
			 tpc_stats->twice_antenna_gain,
			 tpc_stats->twice_antenna_reduction);
	len += scnprintf(buf + len, buf_len - len,
			 "Power Limit	= %2d Reg. Max Power		= %2d\n",
			 tpc_stats->power_limit,
			 tpc_stats->twice_max_rd_power / 2);
	len += scnprintf(buf + len, buf_len - len,
			 "Num tx chains	= %2d Num supported rates	= %2d\n",
			 tpc_stats->num_tx_chain,
			 tpc_stats->rate_max);

	for (j = 0; j < WMI_TPC_FLAG; j++) {
		switch (j) {
		case WMI_TPC_TABLE_TYPE_CDD:
			if (tpc_stats->flag[j] == ATH10K_TPC_TABLE_TYPE_FLAG) {
				len += scnprintf(buf + len, buf_len - len,
						 "CDD not supported\n");
				break;
			}

			ath10k_tpc_stats_print(tpc_stats, j, buf, &len);
			break;
		case WMI_TPC_TABLE_TYPE_STBC:
			if (tpc_stats->flag[j] == ATH10K_TPC_TABLE_TYPE_FLAG) {
				len += scnprintf(buf + len, buf_len - len,
						 "STBC not supported\n");
				break;
			}

			ath10k_tpc_stats_print(tpc_stats, j, buf, &len);
			break;
		case WMI_TPC_TABLE_TYPE_TXBF:
			if (tpc_stats->flag[j] == ATH10K_TPC_TABLE_TYPE_FLAG) {
				len += scnprintf(buf + len, buf_len - len,
						 "TXBF not supported\n***************************\n");
				break;
			}

			ath10k_tpc_stats_print(tpc_stats, j, buf, &len);
			break;
		default:
			len += scnprintf(buf + len, buf_len - len,
					 "Invalid Type\n");
			break;
		}
	}

unlock:
	spin_unlock_bh(&ar->data_lock);

	if (len >= buf_len)
		buf[len - 1] = 0;
	else
		buf[len] = 0;
}

static int ath10k_tpc_stats_open(struct inode *inode, struct file *file)
{
	struct ath10k *ar = inode->i_private;
	void *buf = NULL;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto err_unlock;
	}

	buf = vmalloc(ATH10K_TPC_CONFIG_BUF_SIZE);
	if (!buf) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	ret = ath10k_debug_tpc_stats_request(ar);
	if (ret) {
		ath10k_warn(ar, "failed to request tpc config stats: %d\n",
			    ret);
		goto err_free;
	}

	ath10k_tpc_stats_fill(ar, ar->debug.tpc_stats, buf);
	file->private_data = buf;

	mutex_unlock(&ar->conf_mutex);
	return 0;

err_free:
	vfree(buf);

err_unlock:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static int ath10k_tpc_stats_release(struct inode *inode, struct file *file)
{
	vfree(file->private_data);

	return 0;
}

static ssize_t ath10k_tpc_stats_read(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	const char *buf = file->private_data;
	unsigned int len = strlen(buf);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_tpc_stats = {
	.open = ath10k_tpc_stats_open,
	.release = ath10k_tpc_stats_release,
	.read = ath10k_tpc_stats_read,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_atf_enable(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret;
	u8 enable;

	if (kstrtou8_from_user(user_buf, count, 0, &enable))
		return -EINVAL;
	mutex_lock(&ar->conf_mutex);
	spin_lock_bh(&ar->txqs_lock);
	if (ar->atf_enabled == enable) {
		ret = count;
		goto exit;
	}

	ar->atf_enabled = enable;
	ret = count;
exit:
	spin_unlock_bh(&ar->txqs_lock);
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_read_atf_enable(struct file *file, char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[32];

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->atf_enabled);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_atf_enable = {
	.read = ath10k_read_atf_enable,
	.write = ath10k_write_atf_enable,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_atf_param(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret, len;
	char buf[500];
	u32 temp, tid;

	if (count > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[sizeof(buf) - 1] = '\0';
	len = strlen(buf);

	if (len > 0 && buf[len - 1] == '\n')
		buf[len - 1] = 0;

	ret = count;
	mutex_lock(&ar->conf_mutex);
	spin_lock_bh(&ar->txqs_lock);

	if (sscanf(buf, "interval %u", &temp) == 1) {
		ar->atf_sch_interval =  temp;
		goto exit;
	} else if (sscanf(buf, "max_inflight_airtime %u", &temp) == 1) {
		ar->airtime_inflight_max = temp;
		goto exit;
	} else if (sscanf(buf, "release_limit %u", &temp) == 1) {
		ar->atf_release_limit = temp;
		goto exit;
	} else if (sscanf(buf, "quantum %u %u", &tid, &temp) == 2) {
		if (tid < IEEE80211_NUM_TIDS)
			ar->atf_quantum[tid] =  temp;
		goto exit;
	} else if (sscanf(buf, "quantum_mesh %u %u", &tid, &temp) == 2) {
		if (tid < IEEE80211_NUM_TIDS)
			ar->atf_quantum_mesh[tid] = temp;
		goto exit;
	} else if (sscanf(buf, "txq_limit_min %u %u", &tid, &temp) == 2) {
		if (tid < IEEE80211_NUM_TIDS)
			ar->atf_txq_limit_min[tid] = temp;
		goto exit;
	} else if (sscanf(buf, "txq_limit_max %u %u", &tid, &temp) == 2) {
		if (tid < IEEE80211_NUM_TIDS)
			ar->atf_txq_limit_max[tid] = temp;
		goto exit;
	} else {
		ret = -EINVAL;
	}

exit:
	spin_unlock_bh(&ar->txqs_lock);
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_read_atf_param(struct file *file,
				     char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[500];

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf),
			"interval %u\n"
			"max_inflight_airtime %u\n"
			"release_limit %u\n"
			"quantum %u %u %u %u %u %u %u %u\n"
			"quantum_mesh %u %u %u %u %u %u %u %u\n"
			"txq_limit_min %u %u %u %u %u %u %u %u\n"
			"txq_limit_max %u %u %u %u %u %u %u %u\n",
			ar->atf_sch_interval,
			ar->airtime_inflight_max,
			ar->atf_release_limit,
			ar->atf_quantum[0], ar->atf_quantum[1],
			ar->atf_quantum[2], ar->atf_quantum[3],
			ar->atf_quantum[4], ar->atf_quantum[5],
			ar->atf_quantum[6], ar->atf_quantum[7],
			ar->atf_quantum_mesh[0], ar->atf_quantum_mesh[1],
			ar->atf_quantum_mesh[2], ar->atf_quantum_mesh[3],
			ar->atf_quantum_mesh[4], ar->atf_quantum_mesh[5],
			ar->atf_quantum_mesh[6], ar->atf_quantum_mesh[7],
			ar->atf_txq_limit_min[0], ar->atf_txq_limit_min[1],
			ar->atf_txq_limit_min[2], ar->atf_txq_limit_min[3],
			ar->atf_txq_limit_min[4], ar->atf_txq_limit_min[5],
			ar->atf_txq_limit_min[6], ar->atf_txq_limit_min[7],
			ar->atf_txq_limit_max[0], ar->atf_txq_limit_max[1],
			ar->atf_txq_limit_max[2], ar->atf_txq_limit_max[3],
			ar->atf_txq_limit_max[4], ar->atf_txq_limit_max[5],
			ar->atf_txq_limit_max[6], ar->atf_txq_limit_max[7]);

	mutex_unlock(&ar->conf_mutex);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_atf_param = {
	.read = ath10k_read_atf_param,
	.write = ath10k_write_atf_param,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_read_atf_stats(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	struct ieee80211_txq *txq;
	struct ath10k_txq *artxq;
	struct atf_scheduler *atf;
	int retval;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len = scnprintf(buf, buf_len, "Atf_codel enable:%d\n"
			"Total frames pending: %u, airtime: %u bytes_sent: %u\n",
			ath10k_atf_scheduler_enabled(ar),
			ar->htt.num_pending_tx, ar->airtime_inflight,
			ar->atf_bytes_send_last_interval);
	spin_lock_bh(&ar->txqs_lock);
	rcu_read_lock();
	list_for_each_entry(artxq, &ar->txqs, list) {
		txq = container_of((void *)artxq, struct ieee80211_txq,
				   drv_priv);
		atf = &artxq->atf;
		if (txq->sta)
			len += scnprintf(buf + len, buf_len - len, "STA: %pM\n",
					 txq->sta->addr);
		len += scnprintf(buf + len, buf_len - len, "tid: %d deficit: %d"
				 " frames: %u, airtime %d bytes_sent: %u\n",
				 txq->tid, atf->deficit, atf->frames_inflight,
				 atf->airtime_inflight,
				 atf->bytes_send_last_interval);
	}
	rcu_read_unlock();
	spin_unlock_bh(&ar->txqs_lock);

	retval = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return retval;
}

static const struct file_operations fops_atf_stats = {
	.read = ath10k_read_atf_stats,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

int ath10k_debug_start(struct ath10k *ar)
{
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	ret = ath10k_debug_htt_stats_req(ar);
	if (ret)
		/* continue normally anyway, this isn't serious */
		ath10k_warn(ar, "failed to start htt stats workqueue: %d\n",
			    ret);

	if (ar->debug.fw_dbglog_mask) {
		ret = ath10k_wmi_dbglog_cfg(ar, ar->debug.fw_dbglog_mask,
					    ATH10K_DBGLOG_LEVEL_WARN);
		if (ret)
			/* not serious */
			ath10k_warn(ar, "failed to enable dbglog during start: %d",
				    ret);
	}

	if (ar->debug.pktlog_filter) {
		ret = ath10k_wmi_pdev_pktlog_enable(ar,
						    ar->debug.pktlog_filter);
		if (ret)
			/* not serious */
			ath10k_warn(ar,
				    "failed to enable pktlog filter %x: %d\n",
				    ar->debug.pktlog_filter, ret);
	} else {
		ret = ath10k_wmi_pdev_pktlog_disable(ar);
		if (ret)
			/* not serious */
			ath10k_warn(ar, "failed to disable pktlog: %d\n", ret);
	}

	if (ar->debug.nf_cal_period) {
		ret = ath10k_wmi_pdev_set_param(ar,
						ar->wmi.pdev_param->cal_period,
						ar->debug.nf_cal_period);
		if (ret)
			/* not serious */
			ath10k_warn(ar, "cal period cfg failed from debug start: %d\n",
				    ret);
	}

	return ret;
}

void ath10k_debug_stop(struct ath10k *ar)
{
	lockdep_assert_held(&ar->conf_mutex);

	/* Must not use _sync to avoid deadlock, we do that in
	 * ath10k_debug_destroy(). The check for htt_stats_mask is to avoid
	 * warning from del_timer(). */
	if (ar->debug.htt_stats_mask != 0)
		cancel_delayed_work(&ar->debug.htt_stats_dwork);

#ifdef CONFIG_ATH10K_SMART_ANTENNA
	if (!ar->smart_ant_info.enabled)
		ath10k_wmi_pdev_pktlog_disable(ar);
#else
	ath10k_wmi_pdev_pktlog_disable(ar);
#endif
}

static ssize_t ath10k_write_simulate_radar(struct file *file,
					   const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;

	ieee80211_radar_detected(ar->hw);

	return count;
}

static const struct file_operations fops_simulate_radar = {
	.write = ath10k_write_simulate_radar,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

#define ATH10K_DFS_STAT(s, p) (\
	len += scnprintf(buf + len, size - len, "%-28s : %10u\n", s, \
			 ar->debug.dfs_stats.p))

#define ATH10K_DFS_POOL_STAT(s, p) (\
	len += scnprintf(buf + len, size - len, "%-28s : %10u\n", s, \
			 ar->debug.dfs_pool_stats.p))

static ssize_t ath10k_read_dfs_stats(struct file *file, char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	int retval = 0, len = 0;
	const int size = 8000;
	struct ath10k *ar = file->private_data;
	char *buf;

	buf = kzalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	if (!ar->dfs_detector) {
		len += scnprintf(buf + len, size - len, "DFS not enabled\n");
		goto exit;
	}

	ar->debug.dfs_pool_stats =
			ar->dfs_detector->get_stats(ar->dfs_detector);

	len += scnprintf(buf + len, size - len, "Pulse detector statistics:\n");

	ATH10K_DFS_STAT("reported phy errors", phy_errors);
	ATH10K_DFS_STAT("pulse events reported", pulses_total);
	ATH10K_DFS_STAT("DFS pulses detected", pulses_detected);
	ATH10K_DFS_STAT("DFS pulses discarded", pulses_discarded);
	ATH10K_DFS_STAT("Radars detected", radar_detected);

	len += scnprintf(buf + len, size - len, "Global Pool statistics:\n");
	ATH10K_DFS_POOL_STAT("Pool references", pool_reference);
	ATH10K_DFS_POOL_STAT("Pulses allocated", pulse_allocated);
	ATH10K_DFS_POOL_STAT("Pulses alloc error", pulse_alloc_error);
	ATH10K_DFS_POOL_STAT("Pulses in use", pulse_used);
	ATH10K_DFS_POOL_STAT("Seqs. allocated", pseq_allocated);
	ATH10K_DFS_POOL_STAT("Seqs. alloc error", pseq_alloc_error);
	ATH10K_DFS_POOL_STAT("Seqs. in use", pseq_used);

exit:
	if (len > size)
		len = size;

	retval = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return retval;
}

static const struct file_operations fops_dfs_stats = {
	.read = ath10k_read_dfs_stats,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_pktlog_filter(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 filter;
	int ret;

	if (kstrtouint_from_user(ubuf, count, 0, &filter))
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

#ifdef CONFIG_ATH10K_SMART_ANTENNA
	if (ar->smart_ant_info.enabled)
		filter |= ATH10K_PKTLOG_SMART_ANT;
#endif
	if (ar->state != ATH10K_STATE_ON) {
		ar->debug.pktlog_filter = filter;
		ret = count;
		goto out;
	}

	if (filter == ar->debug.pktlog_filter) {
		ret = count;
		goto out;
	}

	if (filter) {
		ret = ath10k_wmi_pdev_pktlog_enable(ar, filter);
		if (ret) {
			ath10k_warn(ar, "failed to enable pktlog filter %x: %d\n",
				    ar->debug.pktlog_filter, ret);
			goto out;
		}
	} else {
#ifdef CONFIG_ATH10K_SMART_ANTENNA
		if (ar->smart_ant_info.enabled) {
			ret = ath10k_wmi_pdev_pktlog_enable(
					ar,
					ATH10K_PKTLOG_SMART_ANT);
			if (ret)
				goto out;
		} else {
			ret = ath10k_wmi_pdev_pktlog_disable(ar);
			if (ret) {
				ath10k_warn(ar, "failed to disable pktlog: %d\n",
					    ret);
				goto out;
			}
		}
#else
		ret = ath10k_wmi_pdev_pktlog_disable(ar);
		if (ret) {
			ath10k_warn(ar, "failed to disable pktlog: %d\n", ret);
			goto out;
		}
#endif
	}

	ar->debug.pktlog_filter = filter;
	ret = count;

out:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_read_pktlog_filter(struct file *file, char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	char buf[32];
	struct ath10k *ar = file->private_data;
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "%08x\n",
			ar->debug.pktlog_filter);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_pktlog_filter = {
	.read = ath10k_read_pktlog_filter,
	.write = ath10k_write_pktlog_filter,
	.open = simple_open
};

static ssize_t ath10k_write_quiet_period(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 period;

	if (kstrtouint_from_user(ubuf, count, 0, &period))
		return -EINVAL;

	if (period < ATH10K_QUIET_PERIOD_MIN) {
		ath10k_warn(ar, "Quiet period %u can not be lesser than 25ms\n",
			    period);
		return -EINVAL;
	}
	mutex_lock(&ar->conf_mutex);
	ar->thermal.quiet_period = period;
	ath10k_thermal_set_throttling(ar);
	mutex_unlock(&ar->conf_mutex);

	return count;
}

static ssize_t ath10k_read_quiet_period(struct file *file, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	char buf[32];
	struct ath10k *ar = file->private_data;
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->thermal.quiet_period);
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_quiet_period = {
	.read = ath10k_read_quiet_period,
	.write = ath10k_write_quiet_period,
	.open = simple_open
};

#define ATH10K_DEFAULT_WLAN_PRIORITY_OVER_BT 0x38

static ssize_t ath10k_write_btcoex(struct file *file,
				   const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	size_t buf_size;
	int ret;
	bool val;
	u32 pdev_param;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, ubuf, buf_size))
		return -EFAULT;

	buf[buf_size] = '\0';

	if (strtobool(buf, &val) != 0)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_RESTARTED) {
		ret = -ENETDOWN;
		goto exit;
	}

	if (!(test_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags) ^ val)) {
		ret = count;
		goto exit;
	}

	pdev_param = ar->wmi.pdev_param->enable_btcoex;
	if (pdev_param != WMI_PDEV_PARAM_UNSUPPORTED) {
		ret = ath10k_wmi_pdev_set_param(ar, pdev_param, val);
		if (ret) {
			ath10k_warn(ar, "failed to enable btcoex: %d\n", ret);
			goto exit;
		}
	} else {
		ath10k_info(ar, "restarting firmware due to btcoex change");
		queue_work(ar->workqueue, &ar->restart_work);
	}

	if (val)
		set_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags);
	else
		clear_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags);

	ar->debug.wlan_traffic_priority =
					ATH10K_DEFAULT_WLAN_PRIORITY_OVER_BT;

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_read_btcoex(struct file *file, char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	char buf[32];
	struct ath10k *ar = file->private_data;
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			test_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags));
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_btcoex = {
	.read = ath10k_read_btcoex,
	.write = ath10k_write_btcoex,
	.open = simple_open
};

static ssize_t ath10k_write_btcoex_priority(struct file *file,
					    const char __user *ubuf,
					    size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 wlan_traffic_priority;
	int ret;

	if (kstrtou32_from_user(ubuf, count, 0, &wlan_traffic_priority))
		return -EINVAL;

	if (wlan_traffic_priority > 0x3f)
		return -E2BIG;

	mutex_lock(&ar->conf_mutex);

	if (!(test_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags))) {
		ret = -EOPNOTSUPP;
		goto exit;
	}

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_RESTARTED) {
		ret = -ENETDOWN;
		goto exit;
	}

	ret = ath10k_wmi_set_coex_param(ar, wlan_traffic_priority);

	if (ret) {
		ath10k_warn(ar, "failed to set wlan priority %d\n", ret);
		goto exit;
	}

	ar->debug.wlan_traffic_priority = wlan_traffic_priority;

	ret = count;
exit:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

#define ATH10K_BE_TRAFFIC_OVER_BT      BIT(0)
#define ATH10K_BK_TRAFFIC_OVER_BT      BIT(1)
#define ATH10K_VI_TRAFFIC_OVER_BT      BIT(2)
#define ATH10K_VO_TRAFFIC_OVER_BT      BIT(3)
#define ATH10K_BEACON_TRAFFIC_OVER_BT  BIT(4)
#define ATH10K_MGMT_TRAFFIC_OVER_BT    BIT(5)

static ssize_t ath10k_read_btcoex_priority(struct file *file, char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	char buf[100] = "";
	struct ath10k *ar = file->private_data;

	mutex_lock(&ar->conf_mutex);

	if (!(test_bit(ATH10K_FLAG_BTCOEX, &ar->dev_flags))) {
		strcpy (buf, "BTCOEX is disabled\n");
		goto exit;
	}

	if (!ar->debug.wlan_traffic_priority) {
		strcpy(buf, "BT has higher priority than any of the WLAN frames");
	} else {
		if (ar->debug.wlan_traffic_priority &
				ATH10K_BE_TRAFFIC_OVER_BT)
			strcat(buf, "Best Effort\n");
		if (ar->debug.wlan_traffic_priority &
				ATH10K_BK_TRAFFIC_OVER_BT)
			strcat(buf, "Background\n");
		if (ar->debug.wlan_traffic_priority &
				ATH10K_VI_TRAFFIC_OVER_BT)
			strcat(buf, "Video\n");
		if (ar->debug.wlan_traffic_priority &
				ATH10K_VO_TRAFFIC_OVER_BT)
			strcat(buf, "Voice\n");
		if (ar->debug.wlan_traffic_priority &
				ATH10K_BEACON_TRAFFIC_OVER_BT)
			strcat(buf, "Beacon\n");
		if (ar->debug.wlan_traffic_priority &
				ATH10K_MGMT_TRAFFIC_OVER_BT)
			strcat(buf, "Mgmt\n");
	}
exit:
	mutex_unlock(&ar->conf_mutex);
	return simple_read_from_buffer(ubuf, count, ppos, buf, strlen(buf));
}

static const struct file_operations fops_btcoex_priority = {
	.read = ath10k_read_btcoex_priority,
	.write = ath10k_write_btcoex_priority,
	.open = simple_open
};

static ssize_t ath10k_write_cfr_enable(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u8 cfr_enable;
	u32 param;
	int ret;

	if (kstrtou8_from_user(user_buf, count, 0, &cfr_enable))
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto exit;
	}

	if (cfr_enable > 1 || cfr_enable < 0) {
		ret = -EINVAL;
		goto exit;
	}

	if (ar->cfr_enable == cfr_enable) {
		ret = count;
		goto exit;
	}

	param = ar->wmi.pdev_param->enable_cfr_capture;
	ret = ath10k_wmi_pdev_set_param(ar, param, cfr_enable);
	if (ret) {
		ath10k_warn(ar, "CFR capture enable failed from debugfs: %d\n",
			    ret);
		goto exit;
	}

	ar->cfr_enable = cfr_enable;

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_read_cfr_enable(struct file *file, char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[32];

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->cfr_enable);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_cfr_enable = {
	.read = ath10k_read_cfr_enable,
	.write = ath10k_write_cfr_enable,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_disable_peer_stats(struct file *file,
					       const char __user *ubuf,
					       size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	size_t buf_size;
	int ret;
	bool val;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, ubuf, buf_size))
		return -EFAULT;

	buf[buf_size] = '\0';

	if (strtobool(buf, &val) != 0)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_RESTARTED) {
		ret = -ENETDOWN;
		goto exit;
	}

	if (!(test_bit(ATH10K_FLAG_PEER_STATS_DISABLED, &ar->dev_flags) ^ val)) {
		ret = count;
		goto exit;
	}

	if (val)
		set_bit(ATH10K_FLAG_PEER_STATS_DISABLED, &ar->dev_flags);
	else
		clear_bit(ATH10K_FLAG_PEER_STATS_DISABLED, &ar->dev_flags);

	ath10k_info(ar, "restarting firmware due to Peer stats change");

	queue_work(ar->workqueue, &ar->restart_work);
	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_read_disable_peer_stats(struct file *file,
					      char __user *ubuf,
					      size_t count, loff_t *ppos)

{
	char buf[32];
	struct ath10k *ar = file->private_data;
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			test_bit(ATH10K_FLAG_PEER_STATS_DISABLED,
				 &ar->dev_flags));
	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_disable_peer_stats = {
	.read = ath10k_read_disable_peer_stats,
	.write = ath10k_write_disable_peer_stats,
	.open = simple_open
};

static ssize_t ath10k_debug_fw_checksums_read(struct file *file,
					      char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	unsigned int len = 0, buf_len = 4096;
	ssize_t ret_cnt;
	char *buf;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&ar->conf_mutex);

	len += scnprintf(buf + len, buf_len - len,
			 "firmware-N.bin\t\t%08x\n",
			 crc32_le(0, ar->firmware->data, ar->firmware->size));
	len += scnprintf(buf + len, buf_len - len,
			 "athwlan\t\t\t%08x\n",
			 crc32_le(0, ar->firmware_data, ar->firmware_len));
	len += scnprintf(buf + len, buf_len - len,
			 "otp\t\t\t%08x\n",
			 crc32_le(0, ar->otp_data, ar->otp_len));
	len += scnprintf(buf + len, buf_len - len,
			 "codeswap\t\t%08x\n",
			 crc32_le(0, ar->swap.firmware_codeswap_data,
				  ar->swap.firmware_codeswap_len));
	len += scnprintf(buf + len, buf_len - len,
			 "board-N.bin\t\t%08x\n",
			 crc32_le(0, ar->board->data, ar->board->size));
	len += scnprintf(buf + len, buf_len - len,
			 "board\t\t\t%08x\n",
			 crc32_le(0, ar->board_data, ar->board_len));

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&ar->conf_mutex);

	kfree(buf);
	return ret_cnt;
}

static const struct file_operations fops_fw_checksums = {
	.read = ath10k_debug_fw_checksums_read,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_adjacent_wlan_interfrc(struct file *file,
						   const char __user *ubuf,
						   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 flag;

	if (kstrtouint_from_user(ubuf, count, 0, &flag))
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	if (flag != ar->wlan_interfrc_mask) {
		ar->wlan_interfrc_mask = flag;
		queue_work(ar->workqueue, &ar->restart_work);
	}
	mutex_unlock(&ar->conf_mutex);

	return count;
}

#define ATH10K_WLAN_INTFRC_REPORT_SIZE	256

static ssize_t ath10k_read_adjacent_wlan_interfrc(struct file *file,
						  char __user *ubuf,
						  size_t count, loff_t *ppos)
{
	char buf[ATH10K_WLAN_INTFRC_REPORT_SIZE];
	struct ath10k *ar = file->private_data;
	unsigned int len = 0;
	unsigned int buf_len = ATH10K_WLAN_INTFRC_REPORT_SIZE;
	u32 interfrc_5g, interfrc_2g;

	mutex_lock(&ar->conf_mutex);

	len += scnprintf(buf, buf_len,
			 "INTERFRC DETECT FOR SPEC SCAN: %s\n",
			 ar->wlan_interfrc_mask & ATH10K_SPECTRAL_INTERFRC ?
			 "Enable" : "Disable");

	spin_lock_bh(&ar->data_lock);
	interfrc_5g = ar->spectral.interfrc_5g;
	interfrc_2g = ar->spectral.interfrc_2g;
	spin_unlock_bh(&ar->data_lock);

	len += scnprintf(buf + len, buf_len - len, "5G INTERFRC: %d\n",
			 interfrc_5g);
	len += scnprintf(buf + len, buf_len - len, "2G INTERFRC: %d\n",
			 interfrc_2g);
	len += scnprintf(buf + len, buf_len - len,
			 "INTERFRC DETECT FOR SURVEY SCAN: %s\n",
			 ar->wlan_interfrc_mask & ATH10K_SURVEY_INTERFRC ?
			 "Enable" : "Disable");

	mutex_unlock(&ar->conf_mutex);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations fops_adjacent_wlan_interfrc = {
	.read = ath10k_read_adjacent_wlan_interfrc,
	.write = ath10k_write_adjacent_wlan_interfrc,
	.open = simple_open
};

static ssize_t ath10k_write_ps_state_enable(struct file *file,
					    const char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int ret;
	u32 param;
	u8 ps_state_enable;

	if (kstrtou8_from_user(user_buf, count, 0, &ps_state_enable))
		return -EINVAL;

	if (ps_state_enable > 1 || ps_state_enable < 0)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->ps_state_enable == ps_state_enable) {
		ret = count;
		goto exit;
	}

	param = ar->wmi.pdev_param->peer_sta_ps_statechg_enable;
	ret = ath10k_wmi_pdev_set_param(ar, param, ps_state_enable);
	if (ret) {
		ath10k_warn(ar, "ps_state_enable_enable failed from debugfs: %d\n",
			    ret);
		goto exit;
	}
	ar->ps_state_enable = ps_state_enable;

	ret = count;

exit:
	mutex_unlock(&ar->conf_mutex);

	return ret;
}

static ssize_t ath10k_read_ps_state_enable(struct file *file,
					   char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[32];

	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->ps_state_enable);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_ps_state_enable = {
	.read = ath10k_read_ps_state_enable,
	.write = ath10k_write_ps_state_enable,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int ath10k_debug_tpc_stats_final_request(struct ath10k *ar)
{
	int ret;
	unsigned long time_left;

	lockdep_assert_held(&ar->conf_mutex);

	reinit_completion(&ar->debug.tpc_complete);

	ret = ath10k_wmi_pdev_get_tpc_table_cmdid(ar, WMI_TPC_CONFIG_PARAM);
	if (ret) {
		ath10k_warn(ar, "failed to request tpc table cmdid: %d\n", ret);
		return ret;
	}

	time_left = wait_for_completion_timeout(&ar->debug.tpc_complete,
						1 * HZ);
	if (time_left == 0)
		return -ETIMEDOUT;

	return 0;
}

static int ath10k_tpc_stats_final_open(struct inode *inode, struct file *file)
{
	struct ath10k *ar = inode->i_private;
	void *buf = NULL;
	int ret;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON) {
		ret = -ENETDOWN;
		goto err_unlock;
	}

	buf = vmalloc(ATH10K_TPC_CONFIG_BUF_SIZE);
	if (!buf) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	ret = ath10k_debug_tpc_stats_final_request(ar);
	if (ret) {
		ath10k_warn(ar, "failed to request tpc stats final: %d\n",
			    ret);
		goto err_free;
	}

	ath10k_tpc_stats_fill(ar, ar->debug.tpc_stats, buf);
	file->private_data = buf;

	mutex_unlock(&ar->conf_mutex);
	return 0;

err_free:
	vfree(buf);

err_unlock:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static int ath10k_tpc_stats_final_release(struct inode *inode,
					  struct file *file)
{
	vfree(file->private_data);

	return 0;
}

static ssize_t ath10k_tpc_stats_final_read(struct file *file,
					   char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	const char *buf = file->private_data;
	unsigned int len = strlen(buf);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_tpc_stats_final = {
	.open = ath10k_tpc_stats_final_open,
	.release = ath10k_tpc_stats_final_release,
	.read = ath10k_tpc_stats_final_read,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_burst_dur(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	u32 dur[4];
	int ret;
	int ac;
	char buf[128];

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	/* make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = 0;

	ret = sscanf(buf, "%u %u %u %u", &dur[0], &dur[1], &dur[2], &dur[3]);

	if (!ret)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);
	for (ac = 0; ac < 4; ac++) {
		if ((dur[ac] != BURST_ZERO) && (dur[ac] < MIN_BURST_DUR || dur[ac] > MAX_BURST_DUR)) {
			mutex_unlock(&ar->conf_mutex);
			return -EINVAL;
		}
		ar->burst_dur[ac] = dur[ac];

		if (ar->state != ATH10K_STATE_ON)
			continue;

		ret = ath10k_wmi_pdev_set_param(ar, ar->wmi.pdev_param->aggr_burst,
						(SM(ac, ATH10K_AGGR_BURST_AC) |
						SM(dur[ac], ATH10K_AGGR_BURST_DUR)));
		if (ret) {
			ath10k_warn(ar, "failed to set aggr burst duration for ac %d: %d\n", ac, ret);
			goto exit;
		}
	}

	ret = count;

exit:
        mutex_unlock(&ar->conf_mutex);
        return ret;
}

static ssize_t ath10k_read_burst_dur(struct file *file,
				     char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int len = 0;
	char buf[128];

	len = scnprintf(buf, sizeof(buf) - len, "%u %u %u %u\n",
			ar->burst_dur[0], ar->burst_dur[1],
			ar->burst_dur[2], ar->burst_dur[3]);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_burst_dur = {
        .read = ath10k_read_burst_dur,
        .write = ath10k_write_burst_dur,
        .open = simple_open,
        .owner = THIS_MODULE,
        .llseek = default_llseek,
};

int ath10k_debug_create(struct ath10k *ar)
{
	size_t tx_delay_stats_size;
	struct ath10k_tx_delay_stats *pbuf;
	int i;
	ar->debug.fw_crash_data = vzalloc(sizeof(*ar->debug.fw_crash_data));
	if (!ar->debug.fw_crash_data)
		return -ENOMEM;

	INIT_LIST_HEAD(&ar->debug.fw_stats.pdevs);
	INIT_LIST_HEAD(&ar->debug.fw_stats.vdevs);
	INIT_LIST_HEAD(&ar->debug.fw_stats.peers);
	INIT_LIST_HEAD(&ar->debug.fw_stats.peers_extd);

	tx_delay_stats_size = sizeof(struct ath10k_tx_delay_stats) *
				     ARRAY_SIZE(ar->debug.tx_delay_stats);
	pbuf = kzalloc(tx_delay_stats_size, GFP_KERNEL);
	if (!pbuf)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(ar->debug.tx_delay_stats); i++) {
		ar->debug.tx_delay_stats[i] = pbuf;
		pbuf++;
	}

	memset(ar->debug.ftmr_enabled, -1, sizeof(ar->debug.ftmr_enabled));

	return 0;
}

void ath10k_debug_destroy(struct ath10k *ar)
{
	vfree(ar->debug.fw_crash_data);
	ar->debug.fw_crash_data = NULL;

	ath10k_debug_fw_stats_reset(ar);

	kfree(ar->debug.tpc_stats);
	kfree(ar->debug.tx_delay_stats[0]);
}

/* TX delay stats class names ordered by TID. */
static const char ath10k_tx_delay_stats_names[][7] = {
	"AC_BE",
	"AC_BK",
	"AC_BK+",
	"AC_BE+",
	"AC_VI",
	"AC_VI+",
	"AC_VO",
	"AC_VO+",
	"TID8",
	"TID9",
	"TID10",
	"TID11",
	"TID12",
	"TID13",
	"TID14",
	"TID15",
};

#define ATH10K_TX_DELAY_STATS_NAMES_SIZE ARRAY_SIZE(ath10k_tx_delay_stats_names)

/* Returns start time of the transmit delay histogram stats bin. */
static inline int ath10k_tx_delay_bin_to_ms(int bin)
{
	int bin_ms;

	/* The first two bins span 1ms (i.e. [0, 1), and [1, 2)) are returned
	 * directly. All other power-of-two bucket ranges are subdivided into
	 * two bins, with the even numbered bin covering the first half of the
	 * range and the odd numbered bin offset by 1/2 of the range.
	 */
	if (bin < 2)
		return bin;
	bin_ms = 1 << (bin / 2);
	if (bin % 2)
		bin_ms += bin_ms >> 1;
	return bin_ms;
}

static ssize_t ath10k_tx_delay_stats_dump(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	ssize_t ret_cnt;
	struct ath10k_tx_delay_stats *stats;
	u32 tid, bin, total, counter, target, index;
	u32 percentile[] = {5, 10, 25, 50, 75, 90, 95, 99};

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
			 "Frames pending in driver: %d, max: %d,\n",
			 ar->htt.num_pending_tx, ar->atf_max_num_pending_tx);

	BUILD_BUG_ON(ATH10K_TX_DELAY_STATS_NAMES_SIZE != IEEE80211_NUM_TIDS);
	for (tid = 0; tid < IEEE80211_NUM_TIDS; tid++) {
		stats = ar->debug.tx_delay_stats[tid];
		total = 0;
		for (bin = 0; bin <= ATH10K_TX_DELAY_STATS_MAX_BIN; bin++)
			total += stats->counts[bin];

		/* Skip class that has no activity to make output more
		 * concise.
		 */
		if (total == 0)
			continue;
		len += scnprintf(buf + len, buf_len - len,
				 "TX latency stats for %s: %d frames\n",
				 ath10k_tx_delay_stats_names[tid], total);
		for (counter = 0, bin = 0, index = 0;
		     index < ARRAY_SIZE(percentile); index++) {
			target = total * percentile[index] / 100;
			while ((counter <  target) &&
			       (bin <= ATH10K_TX_DELAY_STATS_MAX_BIN)) {
				counter += stats->counts[bin];
				bin++;
			}
			len += scnprintf(buf + len, buf_len - len,
					 "P%d:\t<%dms\n", percentile[index],
					 ath10k_tx_delay_bin_to_ms(bin));
		}
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t ath10k_tx_delay_stats_clear(struct file *file,
					   const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int val, ret;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret)
		return ret;
	if (val != 0)
		return -EINVAL;

	memset(ar->debug.tx_delay_stats[0], 0,
	       sizeof(struct ath10k_tx_delay_stats) *
		   ARRAY_SIZE(ar->debug.tx_delay_stats));
	ar->atf_max_num_pending_tx = 0;
	return count;
}

static const struct file_operations fops_tx_delay_stats = {
	.read = ath10k_tx_delay_stats_dump,
	.write = ath10k_tx_delay_stats_clear,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_tx_delay_histo_dump(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k_tx_delay_stats *stats = file->private_data;
	struct ath10k_tx_delay_stats stats_local;
	char *buf;
	unsigned int len = 0, buf_len = 4096, i;
	ssize_t ret_cnt;

	memcpy(&stats_local, stats, sizeof(struct ath10k_tx_delay_stats));
	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "TX delay histogram(ms)\n");
	for (i = 0; i < ATH10K_TX_DELAY_STATS_MAX_BIN; i++) {
		len += scnprintf(buf + len, buf_len - len, "[%4u - %4u):%8u ",
				 ath10k_tx_delay_bin_to_ms(i),
				 ath10k_tx_delay_bin_to_ms(i + 1),
				 stats_local.counts[i]);
		if (i % 5 == 4)
			len += scnprintf(buf + len, buf_len - len, "\n");
	}
	len += scnprintf(buf + len, buf_len - len, "[%4d -  inf):%8u ",
			 ath10k_tx_delay_bin_to_ms(i), stats_local.counts[i]);
	len += scnprintf(buf + len, buf_len - len, "\n");

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret_cnt;
}

static ssize_t ath10k_tx_delay_histo_reset(struct file *file,
					   const char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct ath10k_tx_delay_stats *stats = file->private_data;
	int val, ret;

	ret = kstrtoint_from_user(user_buf, count, 0, &val);
	if (ret)
		return ret;
	if (val != 0)
		return -EINVAL;
	memset(stats, 0, sizeof(struct ath10k_tx_delay_stats));
	return count;
}

static const struct file_operations fops_tx_delay_histo = {
	.read = ath10k_tx_delay_histo_dump,
	.write = ath10k_tx_delay_histo_reset,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

void ath10k_txdelay_debugfs_register(struct ath10k *ar)
{
	struct dentry *tx_delay_histo_dir;
	int i;

	tx_delay_histo_dir = debugfs_create_dir("tx_delay_histogram",
						ar->debug.debugfs_phy);
	if (IS_ERR_OR_NULL(tx_delay_histo_dir)) {
		dev_err(ar->dev, "Failed to create debugfs dir %s\n",
			"tx_delay_stats");
		return;
	}
	for (i = 0; i < ATH10K_TX_DELAY_STATS_NAMES_SIZE; i++) {
		debugfs_create_file(ath10k_tx_delay_stats_names[i],
				    S_IRUGO | S_IWUSR, tx_delay_histo_dir,
				    ar->debug.tx_delay_stats[i],
				    &fops_tx_delay_histo);
	}

	debugfs_create_file("stats", S_IRUGO | S_IWUSR,
			    tx_delay_histo_dir, ar,
			    &fops_tx_delay_stats);

	return;
}

static ssize_t ath10k_write_ftm_resp(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	struct ath10k_vif *arvif;
	unsigned int vdev_id, enable;
	int ret;
	char buf[64] = {0};

	simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);

	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf(buf, "%u %u", &vdev_id, &enable);
	if (ret != 2)
		return -EINVAL;

	mutex_lock(&ar->conf_mutex);

	if (ar->state != ATH10K_STATE_ON &&
	    ar->state != ATH10K_STATE_RESTARTED) {
		ret = -ENETDOWN;
		goto exit;
	}

	arvif = ath10k_get_arvif(ar, vdev_id);
	if (!arvif) {
		ret = -EINVAL;
		goto exit;
	}

	/* Disabling the FTMR while there are active sessions is not allowed
	 * as this makes FW wait for all active sessions to run to completion
	 */
	if (!enable && arvif->is_up) {
		ret = -EPERM;
		ath10k_warn(ar,
			    "FTMR disable not allowed as vdev: %d is up: %d\n",
			    vdev_id, ret);
		goto exit;
	}

	if (enable == ar->debug.ftmr_enabled[vdev_id]) {
		ret = count;
		goto exit;
	}

	ret = ath10k_wmi_vdev_set_param(ar, arvif->vdev_id,
					ar->wmi.vdev_param->rtt_responder_role,
					enable);

	if (ret && ret != -EOPNOTSUPP) {
		ath10k_warn(ar, "failed to set ftm_resp for vdev: %d: %d\n",
			    vdev_id, ret);
		goto exit;
	}

	ar->debug.ftmr_enabled[vdev_id] = enable;
	ret = count;
exit:
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

static ssize_t ath10k_read_ftm_resp(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	struct ath10k_vif *arvif;
	ssize_t ret_cnt;
	int len = 0, buf_len = 4096;
	char *buf;

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len, "%17s vdev_id status\n",
			 "mac");

	mutex_lock(&ar->conf_mutex);
	list_for_each_entry(arvif, &ar->arvifs, list)
		len += scnprintf(buf + len, buf_len - len, "%pM %7u %6u\n",
				 arvif->vif->addr, arvif->vdev_id,
				 ar->debug.ftmr_enabled[arvif->vdev_id]);

	mutex_unlock(&ar->conf_mutex);

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret_cnt;
}

static const struct file_operations fops_ftm_resp = {
	.read = ath10k_read_ftm_resp,
	.write = ath10k_write_ftm_resp,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static ssize_t ath10k_write_rts_threshold(struct file *file,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	int rts_threshold, ret = 0;
	struct ath10k_vif *arvif;
	struct ieee80211_vif *vif;

	ret = kstrtoint_from_user(user_buf, count, 0, &rts_threshold);
	if (ret)
		return ret;

	mutex_lock(&ar->conf_mutex);
	list_for_each_entry(arvif, &ar->arvifs, list) {
		vif = arvif->vif;
		if (vif->type != NL80211_IFTYPE_MESH_POINT) {
			ar->hw->wiphy->rts_threshold = rts_threshold;
			ret = ath10k_wmi_vdev_set_param(ar, arvif->vdev_id,
							ar->wmi.vdev_param->rts_threshold,
							ar->hw->wiphy->rts_threshold);
			if (ret) {
				ath10k_warn(ar,
					    "Failed to set rts threshold for vdev %d: %d\n",
					    arvif->vdev_id, ret);
				break;
			}
		}
	}

	mutex_unlock(&ar->conf_mutex);
	return count;
}

static ssize_t ath10k_read_rts_threshold(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct ath10k *ar = file->private_data;
	char buf[32];
	int len = 0;

	mutex_lock(&ar->conf_mutex);
	len = scnprintf(buf, sizeof(buf) - len, "%d\n",
			ar->hw->wiphy->rts_threshold);
	mutex_unlock(&ar->conf_mutex);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations fops_rts_threshold = {
	.read = ath10k_read_rts_threshold,
	.write = ath10k_write_rts_threshold,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

int ath10k_debug_register(struct ath10k *ar)
{
	ar->debug.debugfs_phy = debugfs_create_dir("ath10k",
						   ar->hw->wiphy->debugfsdir);
	if (IS_ERR_OR_NULL(ar->debug.debugfs_phy)) {
		if (IS_ERR(ar->debug.debugfs_phy))
			return PTR_ERR(ar->debug.debugfs_phy);

		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&ar->debug.htt_stats_dwork,
			  ath10k_debug_htt_stats_dwork);

	init_completion(&ar->debug.tpc_complete);
	init_completion(&ar->debug.fw_stats_complete);

	debugfs_create_file("fw_stats", S_IRUSR, ar->debug.debugfs_phy, ar,
			    &fops_fw_stats);

	debugfs_create_file("fw_reset_stats", S_IRUSR, ar->debug.debugfs_phy,
			    ar, &fops_fw_reset_stats);

	debugfs_create_file("wmi_services", S_IRUSR, ar->debug.debugfs_phy, ar,
			    &fops_wmi_services);

	debugfs_create_file("simulate_fw_crash", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_simulate_fw_crash);

	debugfs_create_file("fw_crash_dump", S_IRUSR, ar->debug.debugfs_phy,
			    ar, &fops_fw_crash_dump);

	debugfs_create_file("reg_addr", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_reg_addr);

	debugfs_create_file("reg_value", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_reg_value);

	debugfs_create_file("mem_value", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_mem_value);

	debugfs_create_file("chip_id", S_IRUSR, ar->debug.debugfs_phy,
			    ar, &fops_chip_id);

	debugfs_create_file("htt_stats_mask", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_htt_stats_mask);

	debugfs_create_file("warm_hw_reset", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_warm_hw_reset);
	debugfs_create_file("burst_dur", S_IRUSR | S_IWUSR,
                            ar->debug.debugfs_phy, ar, &fops_burst_dur);

	debugfs_create_file("reset_htt_stats", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_reset_htt_stats);

	debugfs_create_file("htt_max_amsdu_ampdu", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar,
			    &fops_htt_max_amsdu_ampdu);

	debugfs_create_file("fw_dbglog", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_fw_dbglog);

	debugfs_create_file("cal_data", S_IRUSR, ar->debug.debugfs_phy,
			    ar, &fops_cal_data);

	debugfs_create_file("ani_enable", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_ani_enable);

	debugfs_create_file("nf_cal_period", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_nf_cal_period);

	if (config_enabled(CONFIG_ATH10K_DFS_CERTIFIED)) {
		debugfs_create_file("dfs_simulate_radar", S_IWUSR,
				    ar->debug.debugfs_phy, ar,
				    &fops_simulate_radar);

		debugfs_create_bool("dfs_block_radar_events", S_IWUSR,
				    ar->debug.debugfs_phy,
				    &ar->dfs_block_radar_events);

		debugfs_create_file("dfs_stats", S_IRUSR,
				    ar->debug.debugfs_phy, ar,
				    &fops_dfs_stats);
	}

	debugfs_create_file("pktlog_filter", S_IRUGO | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_pktlog_filter);

	debugfs_create_file("quiet_period", S_IRUGO | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_quiet_period);

	debugfs_create_file("tpc_stats", S_IRUSR,
			    ar->debug.debugfs_phy, ar, &fops_tpc_stats);

	if (test_bit(WMI_SERVICE_COEX_GPIO, ar->wmi.svc_map)) {
		debugfs_create_file("btcoex", S_IRUGO | S_IWUSR,
				    ar->debug.debugfs_phy, ar, &fops_btcoex);
		if (test_bit(WMI_SERVICE_BTCOEX, ar->wmi.svc_map))
			debugfs_create_file("btcoex_priority",
				S_IRUGO | S_IWUSR,
				ar->debug.debugfs_phy, ar, &fops_btcoex_priority);
	}

	if (test_bit(WMI_SERVICE_PEER_STATS, ar->wmi.svc_map))
		debugfs_create_file("disable_peer_stats", S_IRUGO | S_IWUSR,
				    ar->debug.debugfs_phy, ar,
				    &fops_disable_peer_stats);

	debugfs_create_file("fw_checksums", S_IRUSR,
			    ar->debug.debugfs_phy, ar, &fops_fw_checksums);

	debugfs_create_file("adjacent_wlan_interfrc", S_IRUGO | S_IWUSR,
			    ar->debug.debugfs_phy, ar,
			    &fops_adjacent_wlan_interfrc);

	debugfs_create_file("ps_state_enable", S_IRUSR | S_IWUSR,
			    ar->debug.debugfs_phy, ar, &fops_ps_state_enable);

	debugfs_create_file("atf_enable", 0600,
			    ar->debug.debugfs_phy, ar, &fops_atf_enable);

	debugfs_create_file("atf_param", 0600,
			    ar->debug.debugfs_phy, ar, &fops_atf_param);

	debugfs_create_file("atf_stats", 0600, ar->debug.debugfs_phy, ar,
			    &fops_atf_stats);

	debugfs_create_file("tpc_stats_final", S_IRUSR,
			    ar->debug.debugfs_phy, ar, &fops_tpc_stats_final);

#ifdef CONFIG_ATH10K_SMART_ANTENNA
	ath10k_smart_ant_debugfs_init(ar);
#endif
	ath10k_txdelay_debugfs_register(ar);

	if (test_bit(WMI_SERVICE_CFR_CAPTURE_SUPPORT, ar->wmi.svc_map))
		debugfs_create_file("periodic_cfr_enable", 0600,
				    ar->debug.debugfs_phy,
				    ar, &fops_cfr_enable);

	debugfs_create_file("ftm_resp", 0600,
			    ar->debug.debugfs_phy, ar, &fops_ftm_resp);

	debugfs_create_file("rts_threshold", 0600,
			    ar->debug.debugfs_phy, ar, &fops_rts_threshold);
	return 0;
}

void ath10k_debug_unregister(struct ath10k *ar)
{
	cancel_delayed_work_sync(&ar->debug.htt_stats_dwork);
}

#endif /* CONFIG_ATH10K_DEBUGFS */

#ifdef CONFIG_ATH10K_DEBUG
void __ath10k_dbg(struct ath10k *ar, enum ath10k_debug_mask mask,
		  const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	if (ath10k_debug_mask & mask)
		dev_printk(KERN_DEBUG, ar->dev, "%pV", &vaf);

	trace_ath10k_log_dbg(ar, mask, &vaf);

	va_end(args);
}
EXPORT_SYMBOL(__ath10k_dbg);

void ath10k_dbg_dump(struct ath10k *ar,
		     enum ath10k_debug_mask mask,
		     const char *msg, const char *prefix,
		     const void *buf, size_t len)
{
	char linebuf[256];
	unsigned int linebuflen;
	const void *ptr;

	if (ath10k_debug_mask & mask) {
		if (msg)
			ath10k_dbg(ar, mask, "%s\n", msg);

		for (ptr = buf; (ptr - buf) < len; ptr += 16) {
			linebuflen = 0;
			linebuflen += scnprintf(linebuf + linebuflen,
						sizeof(linebuf) - linebuflen,
						"%s%08x: ",
						(prefix ? prefix : ""),
						(unsigned int)(ptr - buf));
			hex_dump_to_buffer(ptr, len - (ptr - buf), 16, 1,
					   linebuf + linebuflen,
					   sizeof(linebuf) - linebuflen, true);
			dev_printk(KERN_DEBUG, ar->dev, "%s\n", linebuf);
		}
	}

	/* tracing code doesn't like null strings :/ */
	trace_ath10k_log_dbg_dump(ar, msg ? msg : "", prefix ? prefix : "",
				  buf, len);
}
EXPORT_SYMBOL(ath10k_dbg_dump);

#endif /* CONFIG_ATH10K_DEBUG */
