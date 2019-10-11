/*
 * Copyright (C) 2009 Felix Fietkau <nbd@nbd.name>
 * Copyright (C) 2011-2012 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (c) 2015, 2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2016 John Crispin <john@phrozen.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <net/dsa.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/if_bridge.h>
#include <linux/delay.h>
#include <linux/mdio.h>
#include <linux/etherdevice.h>
#include <linux/debugfs.h>

#include "qca8k.h"

#define MIB_DESC(_s, _o, _n)	\
	{			\
		.size = (_s),	\
		.offset = (_o),	\
		.name = (_n),	\
	}

static const struct qca8k_mib_desc ar8327_mib[] = {
	MIB_DESC(1, 0x00, "RxBroad"),
	MIB_DESC(1, 0x04, "RxPause"),
	MIB_DESC(1, 0x08, "RxMulti"),
	MIB_DESC(1, 0x0c, "RxFcsErr"),
	MIB_DESC(1, 0x10, "RxAlignErr"),
	MIB_DESC(1, 0x14, "RxRunt"),
	MIB_DESC(1, 0x18, "RxFragment"),
	MIB_DESC(1, 0x1c, "Rx64Byte"),
	MIB_DESC(1, 0x20, "Rx128Byte"),
	MIB_DESC(1, 0x24, "Rx256Byte"),
	MIB_DESC(1, 0x28, "Rx512Byte"),
	MIB_DESC(1, 0x2c, "Rx1024Byte"),
	MIB_DESC(1, 0x30, "Rx1518Byte"),
	MIB_DESC(1, 0x34, "RxMaxByte"),
	MIB_DESC(1, 0x38, "RxTooLong"),
	MIB_DESC(2, 0x3c, "RxGoodByte"),
	MIB_DESC(2, 0x44, "RxBadByte"),
	MIB_DESC(1, 0x4c, "RxOverFlow"),
	MIB_DESC(1, 0x50, "Filtered"),
	MIB_DESC(1, 0x54, "TxBroad"),
	MIB_DESC(1, 0x58, "TxPause"),
	MIB_DESC(1, 0x5c, "TxMulti"),
	MIB_DESC(1, 0x60, "TxUnderRun"),
	MIB_DESC(1, 0x64, "Tx64Byte"),
	MIB_DESC(1, 0x68, "Tx128Byte"),
	MIB_DESC(1, 0x6c, "Tx256Byte"),
	MIB_DESC(1, 0x70, "Tx512Byte"),
	MIB_DESC(1, 0x74, "Tx1024Byte"),
	MIB_DESC(1, 0x78, "Tx1518Byte"),
	MIB_DESC(1, 0x7c, "TxMaxByte"),
	MIB_DESC(1, 0x80, "TxOverSize"),
	MIB_DESC(2, 0x84, "TxByte"),
	MIB_DESC(1, 0x8c, "TxCollision"),
	MIB_DESC(1, 0x90, "TxAbortCol"),
	MIB_DESC(1, 0x94, "TxMultiCol"),
	MIB_DESC(1, 0x98, "TxSingleCol"),
	MIB_DESC(1, 0x9c, "TxExcDefer"),
	MIB_DESC(1, 0xa0, "TxDefer"),
	MIB_DESC(1, 0xa4, "TxLateCol"),
};

/* The 32bit switch registers are accessed indirectly. To achieve this we need
 * to set the page of the register. Track the last page that was set to reduce
 * mdio writes
 */
static u16 qca8k_current_page = 0xffff;

static void
qca8k_split_addr(u32 regaddr, u16 *r1, u16 *r2, u16 *page)
{
	regaddr >>= 1;
	*r1 = regaddr & 0x1e;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0x3ff;
}

static u32
qca8k_mii_read32(struct mii_bus *bus, int phy_id, u32 regnum)
{
	u32 val;
	int ret;

	ret = bus->read(bus, phy_id, regnum);
	if (ret >= 0) {
		val = ret;
		ret = bus->read(bus, phy_id, regnum + 1);
		val |= ret << 16;
	}

	if (ret < 0) {
		dev_err_ratelimited(&bus->dev,
				    "failed to read qca8k 32bit register\n");
		return ret;
	}

	return val;
}

static void
qca8k_mii_write32(struct mii_bus *bus, int phy_id, u32 regnum, u32 val)
{
	u16 lo, hi;
	int ret;

	lo = val & 0xffff;
	hi = (u16)(val >> 16);

	ret = bus->write(bus, phy_id, regnum, lo);
	if (ret >= 0)
		ret = bus->write(bus, phy_id, regnum + 1, hi);
	if (ret < 0)
		dev_err_ratelimited(&bus->dev,
				    "failed to write qca8k 32bit register\n");
}

static void
qca8k_set_page(struct mii_bus *bus, u16 page)
{
	if (page == qca8k_current_page)
		return;

	if (bus->write(bus, 0x18, 0, page) < 0)
		dev_err_ratelimited(&bus->dev,
				    "failed to set qca8k page\n");
	qca8k_current_page = page;
}

static u32
qca8k_read(struct qca8k_priv *priv, u32 reg)
{
	u16 r1, r2, page;
	u32 val;

	qca8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&priv->bus->mdio_lock, MDIO_MUTEX_NESTED);

	qca8k_set_page(priv->bus, page);
	val = qca8k_mii_read32(priv->bus, 0x10 | r2, r1);

	mutex_unlock(&priv->bus->mdio_lock);

	return val;
}

static void
qca8k_write(struct qca8k_priv *priv, u32 reg, u32 val)
{
	u16 r1, r2, page;

	qca8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&priv->bus->mdio_lock, MDIO_MUTEX_NESTED);

	qca8k_set_page(priv->bus, page);
	qca8k_mii_write32(priv->bus, 0x10 | r2, r1, val);

	mutex_unlock(&priv->bus->mdio_lock);
}

static u32
qca8k_rmw(struct qca8k_priv *priv, u32 reg, u32 mask, u32 val)
{
	u16 r1, r2, page;
	u32 ret;

	qca8k_split_addr(reg, &r1, &r2, &page);

	mutex_lock_nested(&priv->bus->mdio_lock, MDIO_MUTEX_NESTED);

	qca8k_set_page(priv->bus, page);
	ret = qca8k_mii_read32(priv->bus, 0x10 | r2, r1);
	ret &= ~mask;
	ret |= val;
	qca8k_mii_write32(priv->bus, 0x10 | r2, r1, ret);

	mutex_unlock(&priv->bus->mdio_lock);

	return ret;
}

static void
qca8k_reg_set(struct qca8k_priv *priv, u32 reg, u32 val)
{
	qca8k_rmw(priv, reg, 0, val);
}

static void
qca8k_reg_clear(struct qca8k_priv *priv, u32 reg, u32 val)
{
	qca8k_rmw(priv, reg, val, 0);
}

static int
qca8k_regmap_read(void *ctx, uint32_t reg, uint32_t *val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ctx;

	*val = qca8k_read(priv, reg);

	return 0;
}

static int
qca8k_regmap_write(void *ctx, uint32_t reg, uint32_t val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ctx;

	qca8k_write(priv, reg, val);

	return 0;
}

static const struct regmap_range qca8k_readable_ranges[] = {
	regmap_reg_range(0x0000, 0x00e4), /* Global control */
	regmap_reg_range(0x0100, 0x0168), /* EEE control */
	regmap_reg_range(0x0200, 0x0270), /* Parser control */
	regmap_reg_range(0x0400, 0x0454), /* ACL */
	regmap_reg_range(0x0600, 0x0718), /* Lookup */
	regmap_reg_range(0x0800, 0x0b70), /* QM */
	regmap_reg_range(0x0c00, 0x0c80), /* PKT */
	regmap_reg_range(0x0e00, 0x0e98), /* L3 */
	regmap_reg_range(0x1000, 0x10ac), /* MIB - Port0 */
	regmap_reg_range(0x1100, 0x11ac), /* MIB - Port1 */
	regmap_reg_range(0x1200, 0x12ac), /* MIB - Port2 */
	regmap_reg_range(0x1300, 0x13ac), /* MIB - Port3 */
	regmap_reg_range(0x1400, 0x14ac), /* MIB - Port4 */
	regmap_reg_range(0x1500, 0x15ac), /* MIB - Port5 */
	regmap_reg_range(0x1600, 0x16ac), /* MIB - Port6 */

};

static const struct regmap_access_table qca8k_readable_table = {
	.yes_ranges = qca8k_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(qca8k_readable_ranges),
};

static struct regmap_config qca8k_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x16ac, /* end MIB - Port6 range */
	.reg_read = qca8k_regmap_read,
	.reg_write = qca8k_regmap_write,
	.rd_table = &qca8k_readable_table,
};

static int
qca8k_busy_wait(struct qca8k_priv *priv, u32 reg, u32 mask)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(20);

	/* loop until the busy flag has cleared */
	do {
		u32 val = qca8k_read(priv, reg);
		int busy = val & mask;

		if (!busy)
			break;
		cond_resched();
	} while (!time_after_eq(jiffies, timeout));

	return time_after_eq(jiffies, timeout);
}

static void
qca8k_fdb_read(struct qca8k_priv *priv, struct qca8k_fdb *fdb)
{
	u32 reg[4];
	int i;

	/* load the ARL table into an array */
	for (i = 0; i < 4; i++)
		reg[i] = qca8k_read(priv, QCA8K_REG_ATU_DATA0 + (i * 4));

	/* vid - 83:72 */
	fdb->vid = (reg[2] >> QCA8K_ATU_VID_S) & QCA8K_ATU_VID_M;
	/* aging - 67:64 */
	fdb->aging = reg[2] & QCA8K_ATU_STATUS_M;
	/* portmask - 54:48 */
	fdb->port_mask = (reg[1] >> QCA8K_ATU_PORT_S) & QCA8K_ATU_PORT_M;
	/* mac - 47:0 */
	fdb->mac[0] = (reg[1] >> QCA8K_ATU_ADDR0_S) & 0xff;
	fdb->mac[1] = reg[1] & 0xff;
	fdb->mac[2] = (reg[0] >> QCA8K_ATU_ADDR2_S) & 0xff;
	fdb->mac[3] = (reg[0] >> QCA8K_ATU_ADDR3_S) & 0xff;
	fdb->mac[4] = (reg[0] >> QCA8K_ATU_ADDR4_S) & 0xff;
	fdb->mac[5] = reg[0] & 0xff;
}

static void
qca8k_fdb_write(struct qca8k_priv *priv, u16 vid, u8 port_mask, const u8 *mac,
		u8 aging)
{
	u32 reg[3] = { 0 };
	int i;

	/* vid - 83:72 */
	reg[2] = (vid & QCA8K_ATU_VID_M) << QCA8K_ATU_VID_S;
	/* aging - 67:64 */
	reg[2] |= aging & QCA8K_ATU_STATUS_M;
	/* portmask - 54:48 */
	reg[1] = (port_mask & QCA8K_ATU_PORT_M) << QCA8K_ATU_PORT_S;
	/* mac - 47:0 */
	reg[1] |= mac[0] << QCA8K_ATU_ADDR0_S;
	reg[1] |= mac[1];
	reg[0] |= mac[2] << QCA8K_ATU_ADDR2_S;
	reg[0] |= mac[3] << QCA8K_ATU_ADDR3_S;
	reg[0] |= mac[4] << QCA8K_ATU_ADDR4_S;
	reg[0] |= mac[5];

	/* load the array into the ARL table */
	for (i = 0; i < 3; i++)
		qca8k_write(priv, QCA8K_REG_ATU_DATA0 + (i * 4), reg[i]);
}

static int
qca8k_fdb_access(struct qca8k_priv *priv, enum qca8k_fdb_cmd cmd, int port)
{
	u32 reg;

	/* Set the command and FDB index */
	reg = QCA8K_ATU_FUNC_BUSY;
	reg |= cmd;
	if (port >= 0) {
		reg |= QCA8K_ATU_FUNC_PORT_EN;
		reg |= (port & QCA8K_ATU_FUNC_PORT_M) << QCA8K_ATU_FUNC_PORT_S;
	}

	/* Write the function register triggering the table access */
	qca8k_write(priv, QCA8K_REG_ATU_FUNC, reg);

	/* wait for completion */
	if (qca8k_busy_wait(priv, QCA8K_REG_ATU_FUNC, QCA8K_ATU_FUNC_BUSY))
		return -1;

	/* Check for table full violation when adding an entry */
	if (cmd == QCA8K_FDB_LOAD) {
		reg = qca8k_read(priv, QCA8K_REG_ATU_FUNC);
		if (reg & QCA8K_ATU_FUNC_FULL)
			return -1;
	}

	return 0;
}

static int
qca8k_fdb_next(struct qca8k_priv *priv, struct qca8k_fdb *fdb, int port)
{
	int ret;

	qca8k_fdb_write(priv, fdb->vid, fdb->port_mask, fdb->mac, fdb->aging);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_NEXT, port);
	if (ret >= 0)
		qca8k_fdb_read(priv, fdb);

	return ret;
}

static int
qca8k_fdb_add(struct qca8k_priv *priv, const u8 *mac, u16 port_mask,
	      u16 vid, u8 aging)
{
	int ret;

	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_write(priv, vid, port_mask, mac, aging);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_LOAD, -1);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int
qca8k_fdb_del(struct qca8k_priv *priv, const u8 *mac, u16 port_mask, u16 vid)
{
	int ret;

	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_write(priv, vid, port_mask, mac, 0);
	ret = qca8k_fdb_access(priv, QCA8K_FDB_PURGE, -1);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static void
qca8k_fdb_flush(struct qca8k_priv *priv)
{
	mutex_lock(&priv->reg_mutex);
	qca8k_fdb_access(priv, QCA8K_FDB_FLUSH, -1);
	mutex_unlock(&priv->reg_mutex);
}

static void
qca8k_mib_init(struct qca8k_priv *priv)
{
	mutex_lock(&priv->reg_mutex);
	qca8k_reg_set(priv, QCA8K_REG_MIB, QCA8K_MIB_FLUSH | QCA8K_MIB_BUSY);
	qca8k_busy_wait(priv, QCA8K_REG_MIB, QCA8K_MIB_BUSY);
	qca8k_reg_set(priv, QCA8K_REG_MIB, QCA8K_MIB_CPU_KEEP);
	qca8k_write(priv, QCA8K_REG_MODULE_EN, QCA8K_MODULE_EN_MIB);
	mutex_unlock(&priv->reg_mutex);
}

static int
qca8k_set_pad_ctrl(struct qca8k_priv *priv, int port, int mode)
{
	u32 reg, val;

	switch (port) {
	case 0:
		reg = QCA8K_REG_PORT0_PAD_CTRL;
		break;
	case 6:
		reg = QCA8K_REG_PORT6_PAD_CTRL;
		break;
	default:
		pr_err("Can't set PAD_CTRL on port %d\n", port);
		return -EINVAL;
	}

	/* Configure a port to be directly connected to an external
	 * PHY or MAC.
	 */
	switch (mode) {
	case PHY_INTERFACE_MODE_RGMII:
		/* RGMII mode means no delay so don't enable the delay */
		val = QCA8K_PORT_PAD_RGMII_EN;
		qca8k_write(priv, reg, val);
		break;
	case PHY_INTERFACE_MODE_RGMII_ID:
		/* RGMII_ID needs internal delay. This is enabled through
		 * PORT5_PAD_CTRL for all ports, rather than individual port
		 * registers
		 */
		qca8k_write(priv, reg,
			    QCA8K_PORT_PAD_RGMII_EN |
			    QCA8K_PORT_PAD_RGMII_TX_DELAY(QCA8K_MAX_DELAY) |
			    QCA8K_PORT_PAD_RGMII_RX_DELAY(QCA8K_MAX_DELAY));
		qca8k_write(priv, QCA8K_REG_PORT5_PAD_CTRL,
			    QCA8K_PORT_PAD_RGMII_RX_DELAY_EN);
		break;
	case PHY_INTERFACE_MODE_SGMII:
		qca8k_write(priv, reg, QCA8K_PORT_PAD_SGMII_EN);
		break;
	default:
		pr_err("xMII mode %d not supported\n", mode);
		return -EINVAL;
	}

	return 0;
}

static void
qca8k_port_set_status(struct qca8k_priv *priv, int port, int enable)
{
	u32 mask = QCA8K_PORT_STATUS_TXMAC | QCA8K_PORT_STATUS_RXMAC;

	if (enable)
		qca8k_reg_set(priv, QCA8K_REG_PORT_STATUS(port), mask);
	else
		qca8k_reg_clear(priv, QCA8K_REG_PORT_STATUS(port), mask);
}

static int
qca8k_phy_read(struct dsa_switch *ds, int phy, int regnum)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	return mdiobus_read(priv->bus, phy, regnum);
}

static int
qca8k_phy_write(struct dsa_switch *ds, int phy, int regnum, u16 val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	return mdiobus_write(priv->bus, phy, regnum, val);
}

static int
qca8k_phy_dbg_read(struct dsa_switch *ds, int phy, int regnum)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct mii_bus *bus = priv->bus;
	int val;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy, QCA8K_MII_DBG_ADDR, regnum);
	val = bus->read(bus, phy, QCA8K_MII_DBG_DATA);
	mutex_unlock(&bus->mdio_lock);

	return val;
}

static int
qca8k_phy_dbg_write(struct dsa_switch *ds, int phy,
		    int regnum, u16 val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct mii_bus *bus = priv->bus;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy, QCA8K_MII_DBG_ADDR, regnum);
	bus->write(bus, phy, QCA8K_MII_DBG_DATA, val);
	mutex_unlock(&bus->mdio_lock);

	return 0;
}

static int
qca8k_phy_mmd_write(struct dsa_switch *ds, int phy,
		    int regnum, u16 val)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct mii_bus *bus = priv->bus;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy, QCA8K_MII_MMD_ADDR, regnum);
	bus->write(bus, phy, QCA8K_MII_MMD_DATA, val);
	mutex_unlock(&bus->mdio_lock);

	return 0;
}

static int
qca8k_phy_mmd_read(struct dsa_switch *ds, int phy,
		   int regnum)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct mii_bus *bus = priv->bus;
	int val;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy, QCA8K_MII_MMD_ADDR, regnum);
	val = bus->read(bus, phy, QCA8K_MII_MMD_DATA);
	mutex_unlock(&bus->mdio_lock);

	return val;
}

static int
qca8k_qm_error_check(struct qca8k_priv *priv)
{
	u32 value, qm_err_int;

	value = qca8k_read(priv, QCA8K_GLOBAL_INT1);
	qm_err_int = value & QCA8K_QM_ERR_INT;

	if (qm_err_int)
		return 1;

	qca8k_write(priv, QCA8K_REG_QM_DEBUG_ADDR, 0x0);
	value = qca8k_read(priv, QCA8K_REG_QM_DEBUG_VALUE);

	return value;
}

static void
qca8k_phy_powerdown(struct qca8k_priv *priv)
{
	int i;
	u16 phy_val;
	struct mii_bus *bus;

	bus = priv->bus;

	for (i = 0; i < QCA8K_NUM_PHYS; i++) {
		mdiobus_write(bus, i, MII_BMCR, BMCR_PDOWN);

		phy_val = qca8k_phy_dbg_read(priv->ds, i,
					     QCA8K_PHY_DEBUG_GREEN);
		phy_val &= (~(QCA8K_PHY_GATE_CLK_IN1000));
		qca8k_phy_dbg_write(priv->ds, i, QCA8K_PHY_DEBUG_GREEN,
				    phy_val);

		/* PHY will stop the tx clock for a while when link is down
		 *	1. bit13 = 0, speed up link down tx_clk
		 *	2. bit10 = 0, speed up speed mode change to 2'b10 tx_clk
		 */
		phy_val = qca8k_phy_dbg_read(priv->ds, i,
					     QCA8K_PHY_DEBUG_HIB_CTRL);
		phy_val &= ~(QCA8K_PHY_HIB_CTRL_SEL_RST_80U |
				QCA8K_PHY_HIB_CTRL_EN_ANY_CHANGE);
		qca8k_phy_dbg_write(priv->ds, i,
				    QCA8K_PHY_DEBUG_HIB_CTRL, phy_val);
	}
}

static void
qca8k_hw_soft_reset(struct qca8k_priv *priv)
{
	u32 value = 0;

	value = qca8k_read(priv, QCA8K_REG_MASK_CTRL);
	value |= QCA8K_CTRL_RESET;
	qca8k_write(priv, QCA8K_REG_MASK_CTRL, value);
	/*Need wait reset done*/
	do {
		usleep_range(10, 20);
		value = qca8k_read(priv, QCA8K_REG_MASK_CTRL);
	} while (value & QCA8K_CTRL_RESET);
	do {
		usleep_range(10, 20);
		value = qca8k_read(priv, QCA8K_GLOBAL_INT0);
	} while ((value & QCA8K_GLOBAL_INITIALIZED_STATUS) !=
			QCA8K_GLOBAL_INITIALIZED_STATUS);
}

static int
qca8k_phy_poll_reset(struct mii_bus *bus)
{
	unsigned int sleep_msecs = 20;
	int ret, elapsed, i;

	for (elapsed = sleep_msecs; elapsed <= 600;
	      elapsed += sleep_msecs) {
		msleep(sleep_msecs);
		for (i = 0; i < QCA8K_NUM_PHYS; i++) {
			ret = mdiobus_read(bus, i, MII_BMCR);
			if (ret < 0)
				return ret;
			if (ret & BMCR_RESET)
				break;
			if (i == QCA8K_NUM_PHYS - 1) {
				usleep_range(1000, 2000);
				return 0;
			}
		}
	}
	return -ETIMEDOUT;
}

static void
qca8k_phy_fixup(struct qca8k_priv *priv, int phy_addr)
{
	qca8k_phy_mmd_write(priv->ds, phy_addr, 0x7, 0x3c);
	qca8k_phy_mmd_write(priv->ds, phy_addr, 0x4007, 0x0);
	qca8k_phy_mmd_write(priv->ds, phy_addr, 0x3, 0x800d);
	qca8k_phy_mmd_write(priv->ds, phy_addr, 0x4003, 0x803f);

	qca8k_phy_dbg_write(priv->ds, phy_addr, 0x3d, 0x6860);
	qca8k_phy_dbg_write(priv->ds, phy_addr, 0x5, 0x2c46);
	qca8k_phy_dbg_write(priv->ds, phy_addr, 0x3c, 0x6000);
}

static void
qca8k_phy_init(struct qca8k_priv *priv)
{
	int i;
	struct mii_bus *bus;
	u32 adv = ADVERTISE_ALL | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

	bus = priv->bus;
	for (i = 0; i < QCA8K_NUM_PHYS; i++) {
		/* phy fixup */
		qca8k_phy_fixup(priv, i);

		/* initialize the port itself */
		mdiobus_write(bus, i, MII_ADVERTISE, adv);

		mdiobus_write(bus, i, MII_CTRL1000,
			      ADVERTISE_1000FULL | BIT(10));

		mdiobus_write(bus, i, MII_BMCR, BMCR_RESET | BMCR_ANENABLE);
	}

	qca8k_phy_poll_reset(bus);
}

static int
qca8k_hw_init(struct dsa_switch *ds)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int ret, i, phy_mode = -1;
	u32 mask;

	/* Initialize CPU port pad mode (xMII type, delays...) */
	phy_mode = of_get_phy_mode(ds->dst->cpu_dp->dn);
	if (phy_mode < 0) {
		pr_err("Can't find phy-mode for master device\n");
		return phy_mode;
	}
	ret = qca8k_set_pad_ctrl(priv, QCA8K_CPU_PORT, phy_mode);
	if (ret < 0)
		return ret;

	/* Enable CPU Port, force it to maximum bandwidth and full-duplex */
	mask = QCA8K_PORT_STATUS_SPEED_1000 | QCA8K_PORT_STATUS_TXFLOW |
	       QCA8K_PORT_STATUS_RXFLOW | QCA8K_PORT_STATUS_DUPLEX;
	qca8k_write(priv, QCA8K_REG_PORT_STATUS(QCA8K_CPU_PORT), mask);
	qca8k_reg_set(priv, QCA8K_REG_GLOBAL_FW_CTRL0,
		      QCA8K_GLOBAL_FW_CTRL0_CPU_PORT_EN);
	qca8k_port_set_status(priv, QCA8K_CPU_PORT, 1);
	priv->port_sts[QCA8K_CPU_PORT].enabled = 1;

	/* Disable AZ */
	qca8k_write(priv, QCA8K_REG_EEE_CTRL, 0);

	/* Enable MIB counters */
	qca8k_mib_init(priv);

	/* Enable QCA header mode on the cpu port */
	qca8k_write(priv, QCA8K_REG_PORT_HDR_CTRL(QCA8K_CPU_PORT),
		    QCA8K_PORT_HDR_CTRL_ALL << QCA8K_PORT_HDR_CTRL_TX_S |
		    QCA8K_PORT_HDR_CTRL_ALL << QCA8K_PORT_HDR_CTRL_RX_S);

	/* Disable forwarding by default on all ports */
	for (i = 0; i < QCA8K_NUM_PORTS; i++)
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(i),
			  QCA8K_PORT_LOOKUP_MEMBER, 0);

	/* Disable MAC by default on all user ports */
	for (i = 1; i < QCA8K_NUM_PORTS; i++)
		if (ds->enabled_port_mask & BIT(i))
			qca8k_write(priv, QCA8K_REG_PORT_STATUS(i), BIT(12));

	/* Forward all unknown frames to CPU port for Linux processing */
	qca8k_write(priv, QCA8K_REG_GLOBAL_FW_CTRL1,
		    BIT(0) << QCA8K_GLOBAL_FW_CTRL1_IGMP_DP_S |
		    BIT(0) << QCA8K_GLOBAL_FW_CTRL1_BC_DP_S |
		    BIT(0) << QCA8K_GLOBAL_FW_CTRL1_MC_DP_S |
		    BIT(0) << QCA8K_GLOBAL_FW_CTRL1_UC_DP_S);

	/* Setup connection between CPU port & user ports */
	for (i = 0; i < DSA_MAX_PORTS; i++) {
		/* CPU port gets connected to all user ports of the switch */
		if (dsa_is_cpu_port(ds, i)) {
			qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(QCA8K_CPU_PORT),
				  QCA8K_PORT_LOOKUP_MEMBER,
				  ds->enabled_port_mask);
		}

		/* Invividual user ports get connected to CPU port only */
		if (ds->enabled_port_mask & BIT(i)) {
			int shift = 16 * (i % 2);

			qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(i),
				  QCA8K_PORT_LOOKUP_MEMBER,
				  BIT(QCA8K_CPU_PORT));

			/* Enable ARP Auto-learning by default */
			qca8k_reg_set(priv, QCA8K_PORT_LOOKUP_CTRL(i),
				      QCA8K_PORT_LOOKUP_LEARN);

			/* For port based vlans to work we need to set the
			 * default egress vid
			 */
			qca8k_rmw(priv, QCA8K_EGRESS_VLAN(i),
				  0xffff << shift, 1 << shift);
			qca8k_write(priv, QCA8K_REG_PORT_VLAN_CTRL0(i),
				    QCA8K_PORT_VLAN_CVID(1) |
				    QCA8K_PORT_VLAN_SVID(1));
		}
	}

	/* Flush the FDB table */
	qca8k_fdb_flush(priv);

	return 0;
}

static int
qca8k_setup(struct dsa_switch *ds)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int ret;

	/* Make sure that port 0 is the cpu port */
	if (!dsa_is_cpu_port(ds, 0)) {
		pr_err("port 0 is not the CPU port\n");
		return -EINVAL;
	}

	mutex_init(&priv->reg_mutex);

	/* Start by setting up the register mapping */
	priv->regmap = devm_regmap_init(ds->dev, NULL, priv,
					&qca8k_regmap_config);
	if (IS_ERR(priv->regmap))
		pr_warn("regmap initialization failed");

	qca8k_phy_powerdown(priv);
	qca8k_hw_soft_reset(priv);
	ret = qca8k_hw_init(ds);
	if (ret)
		return ret;
	qca8k_phy_init(priv);

	return 0;
}

static int
qca8k_qm_err_recovery(struct qca8k_priv *priv)
{
	qca8k_phy_powerdown(priv);
	qca8k_hw_soft_reset(priv);
	qca8k_hw_init(priv->ds);
	qca8k_phy_init(priv);

	return 0;
}

static int qca8k_force_mac_status(struct qca8k_priv *priv,
				  u32 port_id, bool link_en)
{
	u32 reg, value;

	if (port_id < 1 || port_id > 5)
		return -1;

	reg = QCA8K_REG_PORT_STATUS(port_id);
	value = qca8k_read(priv, reg);
	if (link_en)
		value |= QCA8K_PORT_STATUS_LINK_AUTO;
	else
		value &= (~(QCA8K_PORT_STATUS_LINK_AUTO));
	qca8k_write(priv, reg, value);

	return 0;
}

static void
qca8k_phy_manu_ctrl_en(struct qca8k_priv *priv,
		       int phy_addr, bool enable)
{
	u16 phy_val = 0;

	phy_val = qca8k_phy_dbg_read(priv->ds, phy_addr, QCA8K_PHY_DEBUG_0);
	if (enable)
		phy_val |= QCA8K_PHY_MANU_CTRL_EN;
	else
		phy_val &= (~QCA8K_PHY_MANU_CTRL_EN);
	qca8k_phy_dbg_write(priv->ds, phy_addr, QCA8K_PHY_DEBUG_0, phy_val);
}

static int
qca8k_get_qm_status(struct qca8k_priv *priv, u32 port_id, u32 *qm_buffer)
{
	u32 reg;
	u32 qm_val;

	if (port_id < 0 || port_id > 6) {
		*qm_buffer = 0;
		return -1;
	}

	if (port_id < 4) {
		reg = QCA8K_REG_QM_PORT0_3_QNUM;
		qca8k_write(priv, QCA8K_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_read(priv, QCA8K_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer = (qm_val >> (port_id * 8)) & 0xFF;
	} else {
		reg = QCA8K_REG_QM_PORT4_6_QNUM;
		qca8k_write(priv, QCA8K_REG_QM_DEBUG_ADDR, reg);
		qm_val = qca8k_read(priv, QCA8K_REG_QM_DEBUG_VALUE);
		/* every 8 bits for each port */
		*qm_buffer = (qm_val >> ((port_id - 4) * 8)) & 0xFF;
	}

	return 0;
}

static int
qca8k_force_mac_speed_duplex(struct qca8k_priv *priv,
			     u32 port_id, u32 speed, u32 duplex)
{
	u32 reg, value;

	reg = QCA8K_REG_PORT_STATUS(port_id);
	value = qca8k_read(priv, reg);
	value &= ~(QCA8K_PORT_STATUS_DUPLEX |
			QCA8K_PORT_STATUS_SPEED);
	value |= speed;
	if (duplex == QCA8K_DUPLEX_FULL)
		value |= QCA8K_PORT_STATUS_DUPLEX;
	qca8k_write(priv, reg, value);

	return 0;
}

static void
qca8k_link_sync_function(struct qca8k_priv *priv, int port,
			 struct phy_device *phy)
{
	u32 value = 0;
	u32 qm_buffer = 0;

	/* if QM error, then do SW recovery, check link the next time */
	value = qca8k_qm_error_check(priv);
	if (value) {
		qca8k_qm_err_recovery(priv);
		return;
	}

	/* Up --> Down */
	if (!phy->link) {
		/* LINK_EN disable(MAC force mode) */
		qca8k_force_mac_status(priv, port, false);

		qca8k_phy_manu_ctrl_en(priv, phy->mdio.addr, false);

		/* Check queue buffer */
		qca8k_get_qm_status(priv, port, &qm_buffer);
		if (qm_buffer == 0) {
			qca8k_force_mac_speed_duplex(priv, port,
						     QCA8K_PORT_SPEED_1000M,
						      QCA8K_DUPLEX_FULL);
		}
	} else {
		/* Down --> Up */
		qca8k_get_qm_status(priv, port, &qm_buffer);
		if (qm_buffer) {
			qca8k_qm_err_recovery(priv);
			return;
		}
		qca8k_force_mac_speed_duplex(priv, port,
					     phy->speed,
					     phy->duplex);
		usleep_range(100, 200);
		qca8k_force_mac_status(priv, port, true);

		if (phy->speed == QCA8K_PORT_SPEED_100M) {
			/* PHY is link up 100M */
			qca8k_phy_manu_ctrl_en(priv, phy->mdio.addr,
					       true);
		}
	}

}

static void
qca8k_cpuport_setup(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct qca8k_priv *priv = ds->priv;
	u32 reg;

	switch (phy->speed) {
	case 10:
		reg = QCA8K_PORT_STATUS_SPEED_10;
		break;
	case 100:
		reg = QCA8K_PORT_STATUS_SPEED_100;
		break;
	case 1000:
		reg = QCA8K_PORT_STATUS_SPEED_1000;
		break;
	default:
		dev_dbg(priv->dev, "port%d link speed %dMbps not supported.\n",
			port, phy->speed);
		return;
	}

	/* Set duplex mode */
	if (phy->duplex == DUPLEX_FULL)
		reg |= QCA8K_PORT_STATUS_DUPLEX;

	/* Force flow control */
	reg |= QCA8K_PORT_STATUS_RXFLOW | QCA8K_PORT_STATUS_TXFLOW;

	/* Force link down before changing MAC options */
	qca8k_port_set_status(priv, port, 0);
	qca8k_write(priv, QCA8K_REG_PORT_STATUS(port), reg);
	qca8k_port_set_status(priv, port, 1);
}

static void
qca8k_adjust_link(struct dsa_switch *ds, int port, struct phy_device *phy)
{
	struct qca8k_priv *priv = ds->priv;

	/* Force fixed-link setting for CPU port. */
	if (phy_is_pseudo_fixed_link(phy) && dsa_is_cpu_port(ds, port)) {
		qca8k_cpuport_setup(ds, port, phy);
		return;
	}

	mutex_lock(&priv->link_lock);
	qca8k_link_sync_function(priv, port, phy);
	mutex_unlock(&priv->link_lock);
}

static void
qca8k_get_strings(struct dsa_switch *ds, int port, uint8_t *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ar8327_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, ar8327_mib[i].name,
			ETH_GSTRING_LEN);
}

static void
qca8k_get_ethtool_stats(struct dsa_switch *ds, int port,
			uint64_t *data)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	const struct qca8k_mib_desc *mib;
	u32 reg, i;
	u64 hi;

	for (i = 0; i < ARRAY_SIZE(ar8327_mib); i++) {
		mib = &ar8327_mib[i];
		reg = QCA8K_PORT_MIB_COUNTER(port) + mib->offset;

		data[i] = qca8k_read(priv, reg);
		if (mib->size == 2) {
			hi = qca8k_read(priv, reg + 4);
			data[i] |= hi << 32;
		}
	}
}

static int
qca8k_get_sset_count(struct dsa_switch *ds)
{
	return ARRAY_SIZE(ar8327_mib);
}

static int
qca8k_set_mac_eee(struct dsa_switch *ds, int port, struct ethtool_eee *eee)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u32 lpi_en = QCA8K_REG_EEE_CTRL_LPI_EN(port);
	u32 reg;

	mutex_lock(&priv->reg_mutex);
	reg = qca8k_read(priv, QCA8K_REG_EEE_CTRL);
	if (eee->eee_enabled)
		reg |= lpi_en;
	else
		reg &= ~lpi_en;
	qca8k_write(priv, QCA8K_REG_EEE_CTRL, reg);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int
qca8k_get_mac_eee(struct dsa_switch *ds, int port, struct ethtool_eee *e)
{
	/* Nothing to do on the port's MAC */
	return 0;
}

static void
qca8k_port_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = QCA8K_PORT_LOOKUP_STATE_DISABLED;
		break;
	case BR_STATE_BLOCKING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_BLOCKING;
		break;
	case BR_STATE_LISTENING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_LISTENING;
		break;
	case BR_STATE_LEARNING:
		stp_state = QCA8K_PORT_LOOKUP_STATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = QCA8K_PORT_LOOKUP_STATE_FORWARD;
		break;
	}

	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_STATE_MASK, stp_state);
}

static int
qca8k_port_bridge_join(struct dsa_switch *ds, int port, struct net_device *br)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int port_mask = BIT(QCA8K_CPU_PORT);
	int i;

	for (i = 1; i < QCA8K_NUM_PORTS; i++) {
		if (ds->ports[i].bridge_dev != br)
			continue;
		/* Add this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		qca8k_reg_set(priv,
			      QCA8K_PORT_LOOKUP_CTRL(i),
			      BIT(port));
		if (i != port)
			port_mask |= BIT(i);
	}
	/* Add all other ports to this ports portvlan mask */
	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_MEMBER, port_mask);

	return 0;
}

static void
qca8k_port_bridge_leave(struct dsa_switch *ds, int port, struct net_device *br)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	int i;

	for (i = 1; i < QCA8K_NUM_PORTS; i++) {
		if (ds->ports[i].bridge_dev != br)
			continue;
		/* Remove this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		qca8k_reg_clear(priv,
				QCA8K_PORT_LOOKUP_CTRL(i),
				BIT(port));
	}

	/* Set the cpu port to be the only one in the portvlan mask of
	 * this port
	 */
	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		  QCA8K_PORT_LOOKUP_MEMBER, BIT(QCA8K_CPU_PORT));
}

static int
qca8k_port_enable(struct dsa_switch *ds, int port,
		  struct phy_device *phy)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	if (dsa_is_cpu_port(ds, port))
		qca8k_port_set_status(priv, port, 1);
	priv->port_sts[port].enabled = 1;

	phy->advertising |= (ADVERTISED_Pause | ADVERTISED_Asym_Pause);

	if (!dsa_is_cpu_port(ds, port))
		genphy_restart_aneg(phy);

	return 0;
}

static void
qca8k_port_disable(struct dsa_switch *ds, int port,
		   struct phy_device *phy)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;

	qca8k_port_set_status(priv, port, 0);
	priv->port_sts[port].enabled = 0;
}

static int
qca8k_port_fdb_insert(struct qca8k_priv *priv, const u8 *addr,
		      u16 port_mask, u16 vid)
{
	/* Set the vid to the port vlan id if no vid is set */
	if (!vid)
		vid = 1;

	return qca8k_fdb_add(priv, addr, port_mask, vid,
			     QCA8K_ATU_STATUS_STATIC);
}

static int
qca8k_port_fdb_add(struct dsa_switch *ds, int port,
		   const unsigned char *addr, u16 vid)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u16 port_mask = BIT(port);

	return qca8k_port_fdb_insert(priv, addr, port_mask, vid);
}

static int
qca8k_port_fdb_del(struct dsa_switch *ds, int port,
		   const unsigned char *addr, u16 vid)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	u16 port_mask = BIT(port);

	if (!vid)
		vid = 1;

	return qca8k_fdb_del(priv, addr, port_mask, vid);
}

static int
qca8k_port_fdb_dump(struct dsa_switch *ds, int port,
		    dsa_fdb_dump_cb_t *cb, void *data)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)ds->priv;
	struct qca8k_fdb _fdb = { 0 };
	int cnt = QCA8K_NUM_FDB_RECORDS;
	bool is_static;
	int ret = 0;

	mutex_lock(&priv->reg_mutex);
	while (cnt-- && !qca8k_fdb_next(priv, &_fdb, port)) {
		if (!_fdb.aging)
			break;
		is_static = (_fdb.aging == QCA8K_ATU_STATUS_STATIC);
		ret = cb(_fdb.mac, _fdb.vid, is_static, data);
		if (ret)
			break;
	}
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static enum dsa_tag_protocol
qca8k_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_QCA;
}

static const struct dsa_switch_ops qca8k_switch_ops = {
	.get_tag_protocol	= qca8k_get_tag_protocol,
	.setup			= qca8k_setup,
	.adjust_link            = qca8k_adjust_link,
	.get_strings		= qca8k_get_strings,
	.phy_read		= qca8k_phy_read,
	.phy_write		= qca8k_phy_write,
	.get_ethtool_stats	= qca8k_get_ethtool_stats,
	.get_sset_count		= qca8k_get_sset_count,
	.get_mac_eee		= qca8k_get_mac_eee,
	.set_mac_eee		= qca8k_set_mac_eee,
	.port_enable		= qca8k_port_enable,
	.port_disable		= qca8k_port_disable,
	.port_stp_state_set	= qca8k_port_stp_state_set,
	.port_bridge_join	= qca8k_port_bridge_join,
	.port_bridge_leave	= qca8k_port_bridge_leave,
	.port_fdb_add		= qca8k_port_fdb_add,
	.port_fdb_del		= qca8k_port_fdb_del,
	.port_fdb_dump		= qca8k_port_fdb_dump,
};

static ssize_t qca8k_phy_read_reg_get(struct file *fp,
				      char __user *ubuf,
				      size_t sz, loff_t *ppos)
{
	struct qca8k_priv *priv = fp->private_data;
	char lbuf[40];

	if (!priv)
		return -EFAULT;

	snprintf(lbuf, sizeof(lbuf), "reg_val: 0x%x\n",
		 priv->reg_val);

	return simple_read_from_buffer(ubuf, sz, ppos,
					lbuf, strlen(lbuf));
}

static ssize_t qca8k_phy_read_reg_set(struct file *fp,
				      const char __user *ubuf,
				      size_t sz, loff_t *ppos)
{
	struct qca8k_priv *priv = fp->private_data;
	char lbuf[32];
	size_t lbuf_size;
	char *options = lbuf;
	char *this_opt;
	u32 phy_addr, type, reg_addr;
	int val = 0;

	if (!priv)
		return -EFAULT;

	lbuf_size = min(sz, (sizeof(lbuf) - 1));
	if (copy_from_user(lbuf, ubuf, lbuf_size))
		return -EFAULT;
	lbuf[lbuf_size] = 0;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &phy_addr);
	if ((options - lbuf) >= (lbuf_size - 1))
		goto fail;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &type);
	if ((options - lbuf) >= (lbuf_size - 1))
		goto fail;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &reg_addr);

	if (phy_addr > (QCA8K_NUM_PHYS - 1))
		goto fail;

	if (type > QCA8K_PHY_MMD7)
		goto fail;

	switch (type) {
	case QCA8K_PHY_MII:
		val = qca8k_phy_read(priv->ds, phy_addr, reg_addr);
		break;
	case QCA8K_PHY_DBG:
		val = qca8k_phy_dbg_read(priv->ds, phy_addr, reg_addr);
		break;
	case QCA8K_PHY_MMD3:
		qca8k_phy_mmd_write(priv->ds, phy_addr, 3, reg_addr);
		val = qca8k_phy_mmd_read(priv->ds, phy_addr, 0x4003);
		break;
	case QCA8K_PHY_MMD7:
		qca8k_phy_mmd_write(priv->ds, phy_addr, 7, reg_addr);
		val = qca8k_phy_mmd_read(priv->ds, phy_addr, 0x4007);
		break;
	default:
		break;
	}

	priv->reg_val = (u32)val;
	return lbuf_size;

fail:
	pr_err("Format: phy_addr type reg_addr\n");
	return -EINVAL;
}

static ssize_t qca8k_phy_write_reg_set(struct file *fp,
				       const char __user *ubuf,
				       size_t sz, loff_t *ppos)
{
	struct qca8k_priv *priv = (struct qca8k_priv *)fp->private_data;
	char lbuf[32];
	size_t lbuf_size;
	char *options = lbuf;
	char *this_opt;
	u32 phy_addr, type, reg_addr, val;

	if (!priv)
		return -EFAULT;

	lbuf_size = min(sz, (sizeof(lbuf) - 1));
	if (copy_from_user(lbuf, ubuf, lbuf_size))
		return -EFAULT;
	lbuf[lbuf_size] = 0;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &phy_addr);
	if ((options - lbuf) >= (lbuf_size - 1))
		goto fail;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &type);
	if ((options - lbuf) >= (lbuf_size - 1))
		goto fail;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &reg_addr);
	if ((options - lbuf) >= (lbuf_size - 1))
		goto fail;

	this_opt = strsep(&options, " ");
	if (!this_opt)
		goto fail;

	kstrtouint(this_opt, 0, &val);

	if (phy_addr > (QCA8K_NUM_PHYS - 1))
		return -EINVAL;

	if (type > QCA8K_PHY_MMD7)
		return -EINVAL;

	switch (type) {
	case QCA8K_PHY_MII:
		qca8k_phy_write(priv->ds, phy_addr, reg_addr, val);
		break;
	case QCA8K_PHY_DBG:
		qca8k_phy_dbg_write(priv->ds, phy_addr, reg_addr, val);
		break;
	case QCA8K_PHY_MMD3:
		qca8k_phy_mmd_write(priv->ds, phy_addr, 3, reg_addr);
		qca8k_phy_mmd_write(priv->ds, phy_addr, 0x4003, val);
		break;
	case QCA8K_PHY_MMD7:
		qca8k_phy_mmd_write(priv->ds, phy_addr, 7, reg_addr);
		qca8k_phy_mmd_write(priv->ds, phy_addr, 0x4007, val);
		break;
	default:
		break;
	}

	return lbuf_size;

fail:
	pr_err("Format: phy_addr type reg_addr value\n");
	return -EINVAL;
}

static const struct file_operations qca8k_phy_write_reg_ops = {
	.open = simple_open,
	.write = qca8k_phy_write_reg_set,
	.llseek = no_llseek,
};

static const struct file_operations qca8k_phy_read_reg_ops = {
	.open = simple_open,
	.read = qca8k_phy_read_reg_get,
	.write = qca8k_phy_read_reg_set,
	.llseek = no_llseek,
};

static int
qca8k_init_debugfs_entries(struct qca8k_priv *priv)
{
	priv->top_dentry = debugfs_create_dir("qca8k", NULL);
	if (!priv->top_dentry)
		return -ENOMEM;

	priv->phy_write_dentry = debugfs_create_file("phy-write-reg", 0600,
						     priv->top_dentry,
						     priv,
						     &qca8k_phy_write_reg_ops);
	if (!priv->phy_write_dentry)
		goto fail;

	priv->phy_read_dentry = debugfs_create_file("phy-read-reg", 0600,
						    priv->top_dentry,
						    priv,
						    &qca8k_phy_read_reg_ops);
	if (!priv->phy_read_dentry)
		goto fail;

	return 0;

fail:
	debugfs_remove_recursive(priv->top_dentry);
	return -ENOMEM;
}

static int
qca8k_sw_probe(struct mdio_device *mdiodev)
{
	struct qca8k_priv *priv;
	u32 id;
	int ret;

	/* allocate the private data struct so that we can probe the switches
	 * ID register
	 */
	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->bus = mdiodev->bus;
	priv->dev = &mdiodev->dev;

	/* read the switches ID register */
	id = qca8k_read(priv, QCA8K_REG_MASK_CTRL);
	id >>= QCA8K_MASK_CTRL_ID_S;
	id &= QCA8K_MASK_CTRL_ID_M;
	if (id != QCA8K_ID_QCA8337)
		return -ENODEV;

	priv->ds = dsa_switch_alloc(&mdiodev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->priv = priv;
	priv->ds->ops = &qca8k_switch_ops;
	mutex_init(&priv->reg_mutex);
	dev_set_drvdata(&mdiodev->dev, priv);

	mutex_init(&priv->link_lock);

	ret = dsa_register_switch(priv->ds);
	if (!ret)
		qca8k_init_debugfs_entries(priv);

	return ret;
}

static void
qca8k_sw_remove(struct mdio_device *mdiodev)
{
	struct qca8k_priv *priv = dev_get_drvdata(&mdiodev->dev);
	int i;

	for (i = 0; i < QCA8K_NUM_PORTS; i++)
		qca8k_port_set_status(priv, i, 0);

	debugfs_remove_recursive(priv->top_dentry);

	dsa_unregister_switch(priv->ds);
}

#ifdef CONFIG_PM_SLEEP
static void
qca8k_set_pm(struct qca8k_priv *priv, int enable)
{
	int i;

	for (i = 0; i < QCA8K_NUM_PORTS; i++) {
		if (!priv->port_sts[i].enabled)
			continue;

		qca8k_port_set_status(priv, i, enable);
	}
}

static int qca8k_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qca8k_priv *priv = platform_get_drvdata(pdev);

	qca8k_set_pm(priv, 0);

	return dsa_switch_suspend(priv->ds);
}

static int qca8k_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qca8k_priv *priv = platform_get_drvdata(pdev);

	qca8k_set_pm(priv, 1);

	return dsa_switch_resume(priv->ds);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(qca8k_pm_ops,
			 qca8k_suspend, qca8k_resume);

static const struct of_device_id qca8k_of_match[] = {
	{ .compatible = "qca,qca8334" },
	{ .compatible = "qca,qca8337" },
	{ /* sentinel */ },
};

static struct mdio_driver qca8kmdio_driver = {
	.probe  = qca8k_sw_probe,
	.remove = qca8k_sw_remove,
	.mdiodrv.driver = {
		.name = "qca8k",
		.of_match_table = qca8k_of_match,
		.pm = &qca8k_pm_ops,
	},
};

mdio_module_driver(qca8kmdio_driver);

MODULE_AUTHOR("Mathieu Olivari, John Crispin <john@phrozen.org>");
MODULE_DESCRIPTION("Driver for QCA8K ethernet switch family");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qca8k");
