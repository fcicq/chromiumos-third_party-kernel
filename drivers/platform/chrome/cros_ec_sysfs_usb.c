/*
 * Copyright 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/slab.h>

#define USB_CHARGE_MODE_DISABLED 0
#define USB_CHARGE_MODE_ENABLED 4

static int cmd_usb_charge_set_mode_xfer(struct cros_ec_dev *ec, u8 port,
					u8 mode)
{
	int ret;
	struct cros_ec_command *msg;
	struct ec_params_usb_charge_set_mode *param;

	msg = kzalloc(sizeof(*msg) + sizeof(*param), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->command = EC_CMD_USB_CHARGE_SET_MODE | ec->cmd_offset;
	msg->outsize = sizeof(*param);
	param = (struct ec_params_usb_charge_set_mode *)msg->data;
	param->usb_port_id = port;
	param->mode = mode;
	ret = cros_ec_cmd_xfer_status(ec->ec_dev, msg);
	kfree(msg);
	return ret;
}

static ssize_t vbus_disable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	u8 usb_port;
	struct cros_ec_dev *ec = container_of(dev, struct cros_ec_dev,
					      class_dev);

	ret = kstrtou8(buf, 0, &usb_port);
	if (ret)
		return ret;
	ret = cmd_usb_charge_set_mode_xfer(ec, usb_port,
					   USB_CHARGE_MODE_DISABLED);
	if (ret)
		return ret;

	return count;
}

static ssize_t vbus_enable_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int ret;
	u8 usb_port;
	struct cros_ec_dev *ec = container_of(dev, struct cros_ec_dev,
					      class_dev);

	ret = kstrtou8(buf, 0, &usb_port);
	if (ret)
		return ret;
	ret = cmd_usb_charge_set_mode_xfer(ec, usb_port,
					   USB_CHARGE_MODE_ENABLED);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_WO(vbus_disable);
static DEVICE_ATTR_WO(vbus_enable);

static struct attribute *usb_attrs[] = {
	&dev_attr_vbus_disable.attr,
	&dev_attr_vbus_enable.attr,
	NULL,
};

struct attribute_group cros_ec_usb_attr_group = {
	.name = "usb",
	.attrs = usb_attrs,
};
EXPORT_SYMBOL(cros_ec_usb_attr_group);
