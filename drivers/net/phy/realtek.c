/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/device.h>
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/of.h>

#define RTL821x_PHYSR		0x11
#define RTL821x_PHYSR_DUPLEX	0x2000
#define RTL821x_PHYSR_SPEED	0xc000
#define RTL821x_INER		0x12
#define RTL821x_INER_INIT	0x6400
#define RTL821x_INSR		0x13

#define	RTL8211E_INER_LINK_STATUS	0x400

#define RTL8211E_EPAGSR		0x1e
#define RTL8211E_EPAGSR_EXT44	0x2c
#define RTL8211E_EPAGSR_EXT160	0xa0
#define RTL8211E_PAGSEL		0x1f
#define RTL8211E_PAGSEL_P0	0x0
#define RTL8211E_PAGSEL_P5	0x5
#define RTL8211E_PAGSEL_EXTPAGE	0x7
#define RTL8211E_LACR		0x1a /* LED Action Control Register */
#define RTL8211E_LACR_LED0_BLINKING	BIT(4)
#define RTL8211E_LACR_LED1_BLINKING	BIT(5)
#define RTL8211E_LACR_LED2_BLINKING	BIT(6)
#define RTL8211E_LCR		0x1c /* LED Control Register */
#define RTL8211E_LCR_LED0_ACTIVE_SPEED_ALL	(BIT(0) | BIT(1) | BIT(2))
#define RTL8211E_LCR_LED1_ACTIVE_SPEED_100M	BIT(5)
#define RTL8211E_LCR_LED2_ACTIVE_SPEED_1000M	BIT(10)
#define RTL8211E_RXC		0x1a
#define RTL8211E_RXC_SSC_DISABLED		BIT(2)
#define RTL8211E_P05_R05	0x5  /* EEE LED control Reg.5 in Page 5 */
#define RTL8211E_P05_R05_EEE_LED_DISABLED	0x8b82
#define RTL8211E_P05_R06	0x6  /* EEE LED control Reg.6 in Page 5 */
#define RTL8211E_P05_R06_EEE_LED_DISABLED	0x052b

MODULE_DESCRIPTION("Realtek PHY driver");
MODULE_AUTHOR("Johnson Leung");
MODULE_LICENSE("GPL");

static void rtl8211e_setup_led(struct phy_device *phydev)
{
	/* By default the EEE LED mode is enabled, that is
	 * blinking as 400ms on and 2s off. And we want to
	 * disable EEE LED mode.
	 */
	phy_write(phydev, RTL8211E_PAGSEL, RTL8211E_PAGSEL_P5);
	phy_write(phydev, RTL8211E_P05_R05, RTL8211E_P05_R05_EEE_LED_DISABLED);
	phy_write(phydev, RTL8211E_P05_R06, RTL8211E_P05_R06_EEE_LED_DISABLED);

	phy_write(phydev, RTL8211E_PAGSEL, RTL8211E_PAGSEL_EXTPAGE);
	phy_write(phydev, RTL8211E_EPAGSR, RTL8211E_EPAGSR_EXT44);
	phy_write(phydev, RTL8211E_LCR,
		  RTL8211E_LCR_LED0_ACTIVE_SPEED_ALL |
		  RTL8211E_LCR_LED1_ACTIVE_SPEED_100M |
		  RTL8211E_LCR_LED2_ACTIVE_SPEED_1000M);
	/* The LED0 blinking, the other two are in steady mode */
	phy_write(phydev, RTL8211E_LACR, RTL8211E_LACR_LED0_BLINKING);
	/* restore to default page 0 */
	phy_write(phydev, RTL8211E_PAGSEL, RTL8211E_PAGSEL_P0);
}

static void rtl8211e_enable_spread_spectrum_clock(struct phy_device *phydev)
{
	int val;

	phy_write(phydev, RTL8211E_PAGSEL, RTL8211E_PAGSEL_EXTPAGE);
	phy_write(phydev, RTL8211E_EPAGSR, RTL8211E_EPAGSR_EXT160);

	val = phy_read(phydev, RTL8211E_RXC);
	phy_write(phydev, RTL8211E_RXC, val & ~RTL8211E_RXC_SSC_DISABLED);

	phy_write(phydev, RTL8211E_PAGSEL, RTL8211E_PAGSEL_P0);

	dev_info(&phydev->dev, "Enable RTL8211E spread spectrum clock");
}

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL821x_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_init(struct phy_device *phydev)
{
	int ret;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	rtl8211e_setup_led(phydev);

	return 0;
}

static int rtl8211e_probe(struct phy_device *phydev)
{
	struct device_node *of_node = phydev->dev.of_node;

	if (of_property_read_bool(of_node, "enable-phy-ssc"))
		rtl8211e_enable_spread_spectrum_clock(phydev);

	return 0;
}

/* RTL8201CP */
static struct phy_driver rtl8201cp_driver = {
	.phy_id         = 0x00008201,
	.name           = "RTL8201CP Ethernet",
	.phy_id_mask    = 0x0000ffff,
	.features       = PHY_BASIC_FEATURES,
	.flags          = PHY_HAS_INTERRUPT,
	.config_aneg    = &genphy_config_aneg,
	.read_status    = &genphy_read_status,
	.driver         = { .owner = THIS_MODULE,},
};

/* RTL8211B */
static struct phy_driver rtl8211b_driver = {
	.phy_id		= 0x001cc912,
	.name		= "RTL8211B Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211b_config_intr,
	.driver		= { .owner = THIS_MODULE,},
};

/* RTL8211E */
static struct phy_driver rtl8211e_driver = {
	.phy_id		= 0x001cc915,
	.name		= "RTL8211E Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211e_config_intr,
	.config_init	= &rtl8211e_config_init,
	.probe		= &rtl8211e_probe,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
	.driver		= { .owner = THIS_MODULE,},
};

static int __init realtek_init(void)
{
	int ret;

	ret = phy_driver_register(&rtl8201cp_driver);
	if (ret < 0)
		return -ENODEV;
	ret = phy_driver_register(&rtl8211b_driver);
	if (ret < 0)
		return -ENODEV;
	return phy_driver_register(&rtl8211e_driver);
}

static void __exit realtek_exit(void)
{
	phy_driver_unregister(&rtl8211b_driver);
	phy_driver_unregister(&rtl8211e_driver);
}

module_init(realtek_init);
module_exit(realtek_exit);

static struct mdio_device_id __maybe_unused realtek_tbl[] = {
	{ 0x001cc912, 0x001fffff },
	{ 0x001cc915, 0x001fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, realtek_tbl);
