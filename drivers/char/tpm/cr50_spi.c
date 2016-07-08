/*
 * Copyright (C) 2016 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published by
 * the Free Software Foundation.
 *
 * This device driver implements a TCG PTP FIFO compliant interface over SPI
 * for Cr50 devices.
 * It is based on cr50_spi driver by Peter Huewe and Christophe Ricard.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/freezer.h>

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/tpm.h>
#include "tpm.h"
#include "tpm_tis_core.h"

#define MAX_SPI_FRAMESIZE	64

/* Cr50 default timing constants:
 * - can go to sleep not earlier than after CR50_SLEEP_DELAY_MSEC
 * - needs up to CR50_WAKE_START_DELAY_MSEC to wake after sleep
 * - requires at least CR50_ACCESS_DELAY_MSEC between transactions
 */
#define CR50_DFT_SLEEP_DELAY_MSEC		1000
#define CR50_DFT_WAKE_START_DELAY_MSEC		60
#define CR50_DFT_ACCESS_DELAY_MSEC		2

#define TPM_CR50_FW_VER(l)			(0x0F90 | ((l) << 12))
#define TPM_CR50_MAX_FW_VER_LEN			64

struct cr50_spi_phy {
	struct tpm_tis_data priv;
	struct spi_device *spi_device;

	struct mutex time_track_mutex;
	unsigned long last_access_jiffies;
	unsigned long wake_after_jiffies;

	unsigned long access_delay_jiffies;
	unsigned long sleep_delay_jiffies;
	unsigned int wake_start_delay_msec;
};

static inline struct cr50_spi_phy *to_cr50_spi_phy(struct tpm_tis_data *data)
{
	return container_of(data, struct cr50_spi_phy, priv);
}

/* Cr50 needs to have at least some delay between consecutive
 * transactions. Make sure we wait.
 */
static inline void cr50_ensure_access_delay(struct cr50_spi_phy *phy)
{
	/* Note: There is a small chance, if Cr50 is not accessed in a few days,
	 * that time_in_range will not provide the correct result after the wrap
	 * around for jiffies. In this case, we'll have an unneeded short delay,
	 * which is fine.
	 */
	unsigned long allowed_access =
		phy->last_access_jiffies + phy->access_delay_jiffies;
	unsigned long time_now = jiffies;

	if (time_in_range_open(time_now,
			       phy->last_access_jiffies, allowed_access))
		mdelay(jiffies_to_msecs(allowed_access - time_now));
}

/* Cr50 might go to sleep if there is no SPI activity for some time and
 * miss the first few bits/bytes on the bus. In such case, wake it up
 * by asserting CS and give it time to start up.
 */
static inline bool cr50_needs_waking(struct cr50_spi_phy *phy)
{
	/* Note: There is a small chance, if Cr50 is not accessed in a few days,
	 * that time_in_range will not provide the correct result after the wrap
	 * around for jiffies. In this case, we'll probably timeout or read
	 * incorrect value from TPM_STS and just retry the operation.
	 */
	return !time_in_range_open(jiffies,
				   phy->last_access_jiffies,
				   phy->wake_after_jiffies);
}

static inline void cr50_wake_if_needed(struct cr50_spi_phy *phy)
{
	if (cr50_needs_waking(phy)) {
		/* assert CS, wait 1 msec, deassert CS */
		struct spi_transfer spi_cs_wake = { .delay_usecs = 1000 };

		spi_sync_transfer(phy->spi_device, &spi_cs_wake, 1);
		/* wait for it to fully wake */
		msleep(phy->wake_start_delay_msec);
	}
	/* Reset the time when we need to wake Cr50 again */
	phy->wake_after_jiffies = jiffies + phy->sleep_delay_jiffies;
}

/* Flow control: clock the bus and wait for cr50 to set LSB before
 * sending/receiving data. TCG PTP spec allows it to happen during
 * the last byte of header, but cr50 never does that in practice,
 * and earlier versions had a bug when it was set too early, so don't
 * check for it during header transfer.
 */
static int cr50_spi_flow_control(struct cr50_spi_phy *phy)
{
	unsigned long timeout_jiffies =
		jiffies + msecs_to_jiffies(TPM_RETRY * TPM_TIMEOUT_RETRY);
	struct spi_message m;
	int ret;
	u8 rx = 0;
	struct spi_transfer spi_xfer = {
		.rx_buf = &rx,
		.len = 1,
		.cs_change = 1,
	};

	do {
		spi_message_init(&m);
		spi_message_add_tail(&spi_xfer, &m);
		ret = spi_sync_locked(phy->spi_device, &m);
		if (ret < 0)
			return ret;
		if (time_after(jiffies, timeout_jiffies))
			return -EBUSY;
	} while (!(rx & 0x01));
	return 0;
}

static int cr50_spi_xfer_bytes(struct tpm_tis_data *data, u32 addr,
			       u16 len, u8 *buf, bool do_write)
{
	struct cr50_spi_phy *phy = to_cr50_spi_phy(data);
	struct spi_message m;
	u8 tx_buf[4];
	u8 rx_buf[4];
	struct spi_transfer spi_xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = 4,
		.cs_change = 1,
	};
	int ret;

	if (len > MAX_SPI_FRAMESIZE)
		return -EINVAL;

	/* Do this outside of spi_bus_lock in case cr50 is not the
	 * only device on that spi bus.
	 */
	mutex_lock(&phy->time_track_mutex);
	cr50_ensure_access_delay(phy);
	cr50_wake_if_needed(phy);
	mutex_unlock(&phy->time_track_mutex);

	tx_buf[0] = (do_write ? 0x00 : 0x80) | (len - 1);
	tx_buf[1] = 0xD4;
	tx_buf[2] = (addr >> 8) & 0xFF;
	tx_buf[3] = addr & 0xFF;

	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);

	spi_bus_lock(phy->spi_device->master);
	ret = spi_sync_locked(phy->spi_device, &m);
	if (ret < 0)
		goto exit;

	ret = cr50_spi_flow_control(phy);
	if (ret < 0)
		goto exit;

	spi_xfer.cs_change = 0;
	spi_xfer.len = len;
	if (do_write) {
		spi_xfer.tx_buf = buf;
		spi_xfer.rx_buf = NULL;
	} else {
		spi_xfer.tx_buf = NULL;
		spi_xfer.rx_buf = buf;
	}

	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	ret = spi_sync_locked(phy->spi_device, &m);

exit:
	spi_bus_unlock(phy->spi_device->master);

	mutex_lock(&phy->time_track_mutex);
	phy->last_access_jiffies = jiffies;
	mutex_unlock(&phy->time_track_mutex);

	return ret;
}

static int cr50_spi_read_bytes(struct tpm_tis_data *data, u32 addr,
			       u16 len, u8 *result)
{
	return cr50_spi_xfer_bytes(data, addr, len, result, false);
}

static int cr50_spi_write_bytes(struct tpm_tis_data *data, u32 addr,
				u16 len, u8 *value)
{
	return cr50_spi_xfer_bytes(data, addr, len, value, true);
}

static int cr50_spi_read16(struct tpm_tis_data *data, u32 addr, u16 *result)
{
	int rc;

	rc = data->phy_ops->read_bytes(data, addr, sizeof(u16), (u8 *)result);
	if (!rc)
		*result = le16_to_cpu(*result);
	return rc;
}

static int cr50_spi_read32(struct tpm_tis_data *data, u32 addr, u32 *result)
{
	int rc;

	rc = data->phy_ops->read_bytes(data, addr, sizeof(u32), (u8 *)result);
	if (!rc)
		*result = le32_to_cpu(*result);
	return rc;
}

static int cr50_spi_write32(struct tpm_tis_data *data, u32 addr, u32 value)
{
	value = cpu_to_le32(value);
	return data->phy_ops->write_bytes(data, addr, sizeof(u32),
					   (u8 *)&value);
}

static void cr50_get_fw_version(struct tpm_tis_data *data, char *fw_ver)
{
	int i, len = 0;
	char fw_ver_block[4];

	/* Write anything to TPM_CR50_FW_VER to start from the beg of string */
	tpm_tis_write8(data, TPM_CR50_FW_VER(data->locality), 0);

	/* Read the string, 4 bytes at a time, until we get '\0' */
	do {
		tpm_tis_read_bytes(data, TPM_CR50_FW_VER(data->locality), 4,
				   fw_ver_block);
		for (i = 0; i < 4 && fw_ver_block[i]; )
			fw_ver[len++] = fw_ver_block[i++];
	} while (i == 4 && len < TPM_CR50_MAX_FW_VER_LEN);
	fw_ver[len] = 0;
}

static const struct tpm_tis_phy_ops cr50_spi_phy_ops = {
	.read_bytes = cr50_spi_read_bytes,
	.write_bytes = cr50_spi_write_bytes,
	.read16 = cr50_spi_read16,
	.read32 = cr50_spi_read32,
	.write32 = cr50_spi_write32,
	.max_xfer_size = MAX_SPI_FRAMESIZE,
};

static int cr50_of_property_read_u32_optional(struct spi_device *dev,
					      const char *name,
					      u32 default_value,
					      u32 *value)
{
	struct device_node *np = dev->dev.of_node;
	int rc;

	if (of_find_property(np, name, NULL)) {
		rc = of_property_read_u32(np, name, value);
		if (rc < 0) {
			dev_err(&dev->dev,
				"invalid '%s' property (%d)\n",
				name, rc);
			return rc;
		}
	} else {
		*value = default_value;
	}

	dev_dbg(&dev->dev, "%s = %u\n", name, *value);
	return 0;
}

static int cr50_spi_probe(struct spi_device *dev)
{
	char fw_ver[TPM_CR50_MAX_FW_VER_LEN + 1];
	struct cr50_spi_phy *phy;
	int rc;
	u32 value;

	phy = devm_kzalloc(&dev->dev, sizeof(struct cr50_spi_phy),
			   GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->spi_device = dev;

	/* Cr50 timing properties.
	 */
	rc = cr50_of_property_read_u32_optional(dev, "access-delay-ms",
						CR50_DFT_ACCESS_DELAY_MSEC,
						&value);
	if (rc < 0)
		return rc;
	phy->access_delay_jiffies = msecs_to_jiffies(value);

	rc = cr50_of_property_read_u32_optional(dev, "sleep-delay-ms",
						CR50_DFT_SLEEP_DELAY_MSEC,
						&value);
	if (rc < 0)
		return rc;
	phy->sleep_delay_jiffies = msecs_to_jiffies(value);

	rc = cr50_of_property_read_u32_optional(dev, "wake-start-delay-ms",
						CR50_DFT_WAKE_START_DELAY_MSEC,
						&value);
	if (rc < 0)
		return rc;
	phy->wake_start_delay_msec = value;

	mutex_init(&phy->time_track_mutex);
	phy->wake_after_jiffies = jiffies;
	phy->last_access_jiffies = jiffies;

	rc = tpm_tis_core_init(&dev->dev, &phy->priv, -1, &cr50_spi_phy_ops,
			       NULL);
	if (rc < 0)
		return rc;

	cr50_get_fw_version(&phy->priv, fw_ver);
	dev_info(&dev->dev, "Cr50 firmware version: %s\n", fw_ver);

	return 0;
}

static SIMPLE_DEV_PM_OPS(cr50_pm, tpm_pm_suspend, tpm_tis_resume);

static int cr50_spi_remove(struct spi_device *dev)
{
	struct tpm_chip *chip = spi_get_drvdata(dev);

	tpm_chip_unregister(chip);
	tpm_tis_remove(chip);
	return 0;
}

static const struct spi_device_id cr50_spi_id[] = {
	{"cr50", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, cr50_spi_id);

static const struct of_device_id of_cr50_spi_match[] = {
	{ .compatible = "google,cr50", },
	{}
};
MODULE_DEVICE_TABLE(of, of_cr50_spi_match);

static struct spi_driver cr50_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cr50_spi",
		.pm = &cr50_pm,
		.of_match_table = of_match_ptr(of_cr50_spi_match),
	},
	.probe = cr50_spi_probe,
	.remove = cr50_spi_remove,
	.id_table = cr50_spi_id,
};
module_spi_driver(cr50_spi_driver);

MODULE_DESCRIPTION("Cr50 TCG PTP FIFO SPI driver");
MODULE_LICENSE("GPL v2");
