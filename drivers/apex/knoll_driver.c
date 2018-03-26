/* Knoll Driver.
 *
 * Copyright (C) 2017 Google, Inc.
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

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "ds_generic.h"
#include "ds_interrupt.h"
#include "ds_logging.h"
#include "ds_page_table.h"
#include "ds_sysfs.h"
#include "dw_ioctl.h"

/* Constants */
#define DW_DEVICE_NAME "Knoll"
#define DW_DRIVER_VERSION "0.1"

/* IO size on Knoll. */
#define DW_IOMEM_BYTES 0x40000

/* Total coherent memory. */
#define DW_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/* The number of user-mappable memory ranges.
 * TODO(vandwalle): Separate user accessible and kernel
 * accessible regions. */
#define NUM_USER_IOMEM_REGIONS 1

/* The number of page tables in Knoll. */
#define NUM_PAGE_TABLES 1

/* The number of mappable bars on Knoll. */
#define NUM_MAPPABLE_BARS 2

/* Bar Index. */
#define DW_BAR_INDEX 2

/* Bar Offsets. */
#define DW_BAR_OFFSET 0
#define DW_CM_OFFSET 0x1000000

/* enum sysfs_attribute_type: enumeration of the supported sysfs entries. */
enum sysfs_attribute_type {
	ATTR_KERNEL_HIB_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE,
	ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES,
};

/*
 * enum DW_bar2_regs: Register offsets into BAR2 memory.
 * Only values necessary for driver implementation are defined.
 */
enum dw_bar2_regs {
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_SIZE = 0x6000,
	DW_BAR2_REG_KERNEL_HIB_EXTENDED_TABLE = 0x6008,
	DW_BAR2_REG_KERNEL_HIB_TRANSLATION_ENABLE = 0x6010,
	DW_BAR2_REG_KERNEL_HIB_INSTR_QUEUE_INTVECCTL = 0x6018,
	DW_BAR2_REG_KERNEL_HIB_INPUT_ACTV_QUEUE_INTVECCTL = 0x6020,
	DW_BAR2_REG_KERNEL_HIB_PARAM_QUEUE_INTVECCTL = 0x6028,
	DW_BAR2_REG_KERNEL_HIB_OUTPUT_ACTV_QUEUE_INTVECCTL = 0x6030,
	DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL = 0x6038,
	DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL = 0x6040,
	DW_BAR2_REG_KERNEL_HIB_FATAL_ERR_INTVECCTL = 0x6048,
	DW_BAR2_REG_KERNEL_HIB_DMA_PAUSE = 0x6050,
	DW_BAR2_REG_KERNEL_HIB_DMA_PAUSE_MASK = 0x6058,
	DW_BAR2_REG_KERNEL_HIB_STATUS_BLOCK_DELAY = 0x6060,
	DW_BAR2_REG_KERNEL_HIB_MSIX_PENDING_BIT_ARRAY0 = 0x6068,
	DW_BAR2_REG_KERNEL_HIB_MSIX_PENDING_BIT_ARRAY1 = 0x6070,
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_INIT = 0x6078,
	DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE_INIT = 0x6080,
	DW_BAR2_REG_USER_HIB_DMA_PAUSE = 0x86d8,
	DW_BAR2_REG_USER_HIB_DMA_PAUSED = 0x86e0,
	DW_BAR2_REG_USER_HIB_WIRE_INT_PENDING_BIT_ARRAY = 0x8778,
	DW_BAR2_REG_USER_HIB_WIRE_INT_MASK_BIT_ARRAY = 0x8780,
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE = 0x10000,

	DW_BAR2_REG_AON_RESET_REG = 0x20000,
	DW_BAR2_REG_AON_CLOCK_ENABLE_REG = 0x20008,
	DW_BAR2_REG_AON_LOGIC_SHUTDOWN_REG = 0x20010,
	DW_BAR2_REG_AON_MEMORY_SHUTDOWN_REG = 0x20018,
	DW_BAR2_REG_AON_CLAMP_ENABLE_REG = 0x20020,
	DW_BAR2_REG_AON_FORCE_QUIESCE_REG = 0x20028,
	DW_BAR2_REG_AON_AXI_TIE_OFF_REG = 0x20030,
	DW_BAR2_REG_AON_IDLE_REG = 0x20038,

	// Error registers - Used mostly for debug
	DW_BAR2_REG_USER_HIB_ERROR_STATUS = 0x86f0,
	DW_BAR2_REG_SCALAR_CORE_ERROR_STATUS = 0x41a0,

};

#define USER_HIB_CLOCK_ENABLE_BIT 1
#define USER_HIB_CB_IDLE_OVERRIDE_BIT 2

#define HIB_DMA_PAUSE_TIMEOUT_NSEC 100000000LL
#define RESET_TIMEOUT_NSEC 10000000LL

// For now map the entire BAR2 into user space. (This helps debugging when
// running test vectors from user land)
// In production driver we want to exclude the kernel HIB.
static struct ds_page_table_offsets dw_page_table_offsets[] = {
	{ DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_SIZE,
		DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE,
		DW_BAR2_REG_KERNEL_HIB_EXTENDED_TABLE },
};
/* Function declarations */
static int __init dw_init(void);
static void dw_exit(void);

static int dw_add_dev_cb(struct ds_dev *ds_dev);
static int dw_remove_dev_cb(struct ds_dev *ds_dev);

static int dw_sysfs_setup_cb(struct ds_dev *ds_dev);

static int dw_device_cleanup(struct ds_dev *ds_dev);

static int dw_device_open_cb(struct ds_dev *ds_dev);

static ssize_t sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf);

static int dw_reset(struct ds_dev *ds_dev, uint type);

static int dw_get_status(struct ds_dev *ds_dev);

static uint dw_ioctl_check_permissions(struct file *file, uint cmd);

static long dw_ioctl(struct file *file, uint cmd, ulong arg);

static long dw_clock_gating(struct ds_dev *ds_dev, ulong arg);

static void dw_pause_dma(struct ds_dev *ds_dev);

/* Data definitions */

/* The data necessary to display this file's sysfs entries. */
static struct ds_sysfs_attribute dw_sysfs_attrs[] = {
	DS_SYSFS_RO(tensornode_0_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_PAGE_TABLE_SIZE),
	DS_SYSFS_RO(tensornode_0_simple_page_table_entries, sysfs_show,
		ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE),
	DS_SYSFS_RO(tensornode_0_num_mapped_pages, sysfs_show,
		ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES),
	DS_END_OF_ATTR_ARRAY
};

/* IOMEM regions that can be mapped into user space. */
static const struct ds_mappable_region
	mappable_regions[NUM_USER_IOMEM_REGIONS] = {
		{ 0x000000, DW_IOMEM_BYTES },
	};

static const struct ds_mappable_region cm_mappable_regions[1] = { { 0x0,
	DW_CH_MEM_BYTES } };

/* Interrupt descriptors for DW */
static struct ds_interrupt_desc dw_interrupts[] = {
	{ DW_INTERRUPT_WIRE_0, 0, 0 },
};

static const struct ds_wire_interrupt_offsets dw_wire_interrupt_offsets = {
	DW_BAR2_REG_USER_HIB_WIRE_INT_PENDING_BIT_ARRAY,
	DW_BAR2_REG_USER_HIB_WIRE_INT_MASK_BIT_ARRAY
};

static struct ds_driver_desc dw_desc = {
	.name = "dwn",
	.driver_version = DW_DRIVER_VERSION,
	.major = 120,
	.minor = 0,
	.module = THIS_MODULE,

	.num_page_tables = NUM_PAGE_TABLES,
	.page_table_bar_index = DW_BAR_INDEX,
	.page_table_offsets = dw_page_table_offsets,
	.page_table_extended_bit = DW_EXTENDED_SHIFT,

	.bar_descriptions =
		{
			DS_UNUSED_BAR,
			DS_UNUSED_BAR,
			{ DW_IOMEM_BYTES, (VM_WRITE | VM_READ), DW_BAR_OFFSET,
				NUM_USER_IOMEM_REGIONS, mappable_regions },
			DS_UNUSED_BAR,
			DS_UNUSED_BAR,
			{ DW_CH_MEM_BYTES, (VM_WRITE | VM_READ), DW_CM_OFFSET,
				1, cm_mappable_regions, COHERENT_MEMORY },
		},

	.interrupt_type = PLATFORM_WIRE,
	.wire_interrupt_offsets = &dw_wire_interrupt_offsets,
	.num_interrupts = 1,
	.interrupts = dw_interrupts,

	.add_dev_cb = dw_add_dev_cb,
	.remove_dev_cb = dw_remove_dev_cb,

	.enable_dev_cb = NULL,
	.disable_dev_cb = NULL,

	.sysfs_setup_cb = dw_sysfs_setup_cb,
	.sysfs_cleanup_cb = NULL,

	.device_open_cb = dw_device_open_cb,
	.device_close_cb = dw_device_cleanup,

	.ioctl_handler_cb = dw_ioctl,
	.device_status_cb = dw_get_status,
	.hardware_revision_cb = NULL,
	.device_reset_cb = dw_reset,
};

/* Module registration boilerplate */
MODULE_DESCRIPTION("Google Knoll driver");
MODULE_VERSION(DW_DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rob Springer <rspringer@google.com>");
module_init(dw_init);
module_exit(dw_exit);

static int allow_power_save = 1;
static int allow_clock_gating = 1;

module_param(allow_power_save, int, 0644);
module_param(allow_clock_gating, int, 0644);

static int __init dw_init(void)
{
	return ds_register_device(&dw_desc);
}

static void dw_exit(void)
{
	ds_unregister_device(&dw_desc);
}

static int dw_add_dev_cb(struct ds_dev *ds_dev)
{
	ulong page_table_ready, msix_table_ready;
	int retries = 0;

	while (retries < DW_RESET_RETRY) {
		page_table_ready = ds_dev_read_64(ds_dev, DW_BAR_INDEX,
			DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_INIT);
		msix_table_ready = ds_dev_read_64(ds_dev, DW_BAR_INDEX,
			DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE_INIT);
		if (page_table_ready && msix_table_ready)
			break;
		schedule_timeout(msecs_to_jiffies(DW_RESET_DELAY));
		retries++;
	}

	if (retries == DW_RESET_RETRY) {
		if (!page_table_ready)
			ds_log_error(ds_dev, "Page table init timed out.");
		if (!msix_table_ready)
			ds_log_error(ds_dev, "MSI-X table init timed out.");
		return -ETIMEDOUT;
	}

	return 0;
}

static int dw_remove_dev_cb(struct ds_dev *ds_dev)
{
	kfree(ds_dev->cb_data);
	return 0;
}

static int dw_sysfs_setup_cb(struct ds_dev *ds_dev)
{
	return ds_sysfs_create_entries(ds_dev->dev_info.device, dw_sysfs_attrs);
}

/* On device open, we want to perform a core reinit reset. */
static int dw_device_open_cb(struct ds_dev *ds_dev)
{
	ds_log_info(ds_dev, "dw_device_open_cb.");

	return ds_reset_nolock(ds_dev, DW_CHIP_REINIT_RESET);
}

/**
 * dw_get_status - Set device status.
 * @dev: Device struct.
 *
 * Description: Check the device status registers and set the driver status
 *		to ALIVE or DEAD.
 *
 *		Returns 0 if status is ALIVE, a negative error number otherwise.
 */
static int dw_get_status(struct ds_dev *ds_dev)
{

	// Return ALIVE for now
	/*
	      ulong init_val;
	      ulong error_status;

	      init_val = ds_dev_read_64(
		      ds_dev, DW_BAR_INDEX, DW_BAR2_REG_MGT_CHIP_INIT_DONE);
	      if (init_val != DW_CHIP_INIT_DONE) {
		      ds_log_error(ds_dev, "Chip init register value: %lu",
	   init_val); return DS_STATUS_DEAD;
	      }
	      error_status = ds_dev_read_64(ds_dev, DW_BAR_INDEX,
		      DW_BAR2_REG_MGT_GLOBAL_FATAL_ERROR_STATUS);
	      if (error_status) {
		      ds_log_error(
			      ds_dev, "Global error status is 0x%lx",
	   error_status); return DS_STATUS_DEAD;
	      }*/
	return DS_STATUS_ALIVE;
}

/**
 * dw_pause_dma - Pauses and waits for outstanding dma requests to complete
 * @ds_dev: DS device pointer.
 *
 * Note: there is not much we can do if DMA accesses don't complete within
 * a reasonable time frame, i.e. the SoC will most likely be stuck, hence
 * proceeds with whatever reset procedure was ongoing.
 */
static void dw_pause_dma(struct ds_dev *ds_dev)
{
	ds_dev_write_64(
		ds_dev, 1, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_DMA_PAUSE);

	if (ds_wait_sync(ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_DMA_PAUSE,
		    1, 1, HIB_DMA_PAUSE_TIMEOUT_NSEC)) {
		ds_log_error(ds_dev, "Error: DMA paused timed out.");
	}
}

/**
 * dw_toggle_reset - Toggle the chip reset so as to re-init the custom block.
 * @ds_dev: DS device pointer.
 */
static void dw_toggle_reset(struct ds_dev *ds_dev)
{
	dw_pause_dma(ds_dev);
	/* Enter reset */
	ds_dev_write_64(ds_dev, 1, DW_BAR_INDEX, DW_BAR2_REG_AON_RESET_REG);
	ds_wait_sync(ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_DMA_PAUSE, 1, 1,
		RESET_TIMEOUT_NSEC);

	/* Quit reset */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_RESET_REG);
	ds_wait_sync(ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_DMA_PAUSE, 1, 0,
		RESET_TIMEOUT_NSEC);
}

/**
 * dw_device_cleanup - Places the chip in reset.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the hardware. Called on final close via
 * device_close_cb.
 */
static int dw_device_cleanup(struct ds_dev *ds_dev)
{
	u64 scalar_error;
	u64 hib_error;
	u64 clock_enable;

	hib_error = ds_dev_read_64(
		ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_ERROR_STATUS);
	scalar_error = ds_dev_read_64(
		ds_dev, DW_BAR_INDEX, DW_BAR2_REG_SCALAR_CORE_ERROR_STATUS);

	ds_log_info(ds_dev,
		"dw_device_cleanup 0x%p hib_error 0x%llx scalar_error 0x%llx.",
		ds_dev, hib_error, scalar_error);

	if (allow_power_save) {
		/* Enters reset state */

		dw_pause_dma(ds_dev);

		/* Enters quiesce - i.e. HW will dummy out APB r/w */
		ds_dev_write_64(ds_dev, 1, DW_BAR_INDEX,
			DW_BAR2_REG_AON_FORCE_QUIESCE_REG);
		/* Enters Reset */
		ds_dev_write_64(
			ds_dev, 1, DW_BAR_INDEX, DW_BAR2_REG_AON_RESET_REG);
		ds_wait_sync(ds_dev, DW_BAR_INDEX,
			DW_BAR2_REG_USER_HIB_DMA_PAUSE, 1, 1,
			RESET_TIMEOUT_NSEC);

		/* Enters clock gating */
		clock_enable = ds_dev_read_64(
			ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_DMA_PAUSE);
		clock_enable &= ~USER_HIB_CLOCK_ENABLE_BIT;
		ds_dev_write_64(ds_dev, clock_enable, DW_BAR_INDEX,
			DW_BAR2_REG_AON_CLOCK_ENABLE_REG);
		/* Enables clamp */
		ds_dev_write_64(ds_dev, 0x1fff, DW_BAR_INDEX,
			DW_BAR2_REG_AON_CLAMP_ENABLE_REG);
		/* Shutdowns memories */
		ds_dev_write_64(ds_dev, 0x1fff, DW_BAR_INDEX,
			DW_BAR2_REG_AON_MEMORY_SHUTDOWN_REG);
		/* Shutdowns chip logic */
		ds_dev_write_64(ds_dev, 0x1fff, DW_BAR_INDEX,
			DW_BAR2_REG_AON_LOGIC_SHUTDOWN_REG);
	}
	return 0;
}

/**
 * dw_reset - Quits reset.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the hardware. Called on device open.
 */
static int dw_reset(struct ds_dev *ds_dev, uint type)
{
	u64 is_reset;

	ds_nodev_debug("dw_reset.");

	is_reset =
		ds_dev_read_64(ds_dev, DW_BAR_INDEX, DW_BAR2_REG_AON_RESET_REG);
	if (!is_reset && allow_power_save) {
		/* we are not in reset - toggle the reset bit so as to force
		 * re-init of custom block */
		dw_toggle_reset(ds_dev);
	}

	/* Disables logic shutdown */
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_LOGIC_SHUTDOWN_REG);
	/* Disables memory shutdown */
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_MEMORY_SHUTDOWN_REG);
	/* Disables clock gating */
	ds_dev_write_64(
		ds_dev, 1, DW_BAR_INDEX, DW_BAR2_REG_AON_CLOCK_ENABLE_REG);
	/* Disables clamp */
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_CLAMP_ENABLE_REG);
	/* Clears reset register */
	ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_RESET_REG);
	/* Re-enables APB access */
	ds_dev_write_64(
		ds_dev, 0, DW_BAR_INDEX, DW_BAR2_REG_AON_FORCE_QUIESCE_REG);

	return 0;
}

/*
 * dw_ioctl_check_cmd: Verify that the requested ioctl is known.
 * @cmd: The requested ioctl command.
 */
static uint dw_ioctl_check_cmd(uint cmd)
{
	switch (cmd) {
	case DW_IOCTL_GATE_CLOCK:
		return 1;
	default:
		return 0;
	}
}

/*
 * dw_ioctl_check_permissions: Check permissions for ioctls.
 * @file: File pointer from ioctl.
 * @cmd: ioctl command.
 */
static uint dw_ioctl_check_permissions(struct file *filp, uint cmd)
{
	struct ds_dev *ds_dev = filp->private_data;
	int root = capable(CAP_SYS_ADMIN);
	fmode_t write = filp->f_mode & FMODE_WRITE;
	int device_owner = (ds_dev->dev_info.ownership.is_owned &&
			    current->tgid == ds_dev->dev_info.ownership.owner);

	switch (cmd) {
	case DW_IOCTL_GATE_CLOCK:
		return root || (write && device_owner);
	}
	return 0; /* Unknown command/permissions. */
}

/*
 * dw_ioctl: Device specific ioctl handler.
 */
static long dw_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct ds_dev *dev = filp->private_data;

	if (!dw_ioctl_check_cmd(cmd))
		return -ENOTTY;

	if (!dw_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case DW_IOCTL_GATE_CLOCK:
		return dw_clock_gating(dev, arg);
		break;
	}
	return -ENOTTY; /* unknown command */
}

/*
 * dw_ioctl: Handles clock gating ioctl.
 *
 * ds_dev: The DS device.
 * arg: The ioctl arg (enable or disable clock gating)
 */
static long dw_clock_gating(struct ds_dev *ds_dev, ulong arg)
{
	int ret = 0;
	struct dw_gate_clock_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct dw_gate_clock_ioctl)))
		return -EFAULT;

	ds_nodev_debug("dw_clock_gating %d.", (int)ibuf.enable);

	if (ibuf.enable == 1) {
		/* Enters clock gated state */

		/* Pauses interrupts */
		ds_interrupt_pause(ds_dev, 1);

		/*  Quiesces : force axi interface to intercept CSR reads */
		ds_dev_write_64(ds_dev, 1, DW_BAR_INDEX,
			DW_BAR2_REG_AON_FORCE_QUIESCE_REG);

		/* Clock disable */
		ds_dev_write_64(ds_dev, USER_HIB_CB_IDLE_OVERRIDE_BIT,
			DW_BAR_INDEX, DW_BAR2_REG_AON_CLOCK_ENABLE_REG);

	} else {
		/* Leaves clock gated state */

		/* Enables clock */
		ds_dev_write_64(ds_dev,
			USER_HIB_CB_IDLE_OVERRIDE_BIT |
				USER_HIB_CLOCK_ENABLE_BIT,
			DW_BAR_INDEX, DW_BAR2_REG_AON_CLOCK_ENABLE_REG);

		/* Leaves quiesce mode : re-establish CSR access */
		ds_dev_write_64(ds_dev, 0, DW_BAR_INDEX,
			DW_BAR2_REG_AON_FORCE_QUIESCE_REG); // leave quiesce

		/* Un-pauses interrupts */
		ds_interrupt_pause(ds_dev, 0);
	}
	return ret;
}

/*
 * sysfs_show: Display driver sysfs entries.
 * @device: Kernel device structure.
 * @attr: Attribute to display.
 * @buf: Buffer to which to write output.
 *
 * Description: Looks up the driver data and file-specific attribute data (the
 * type of the attribute), then fills "buf" accordingly.
 */
static ssize_t sysfs_show(
	struct device *device, struct device_attribute *attr, char *buf)
{
	struct ds_dev *ds_dev =
		(struct ds_dev *)ds_sysfs_get_device_data(device);
	enum sysfs_attribute_type type =
		(enum sysfs_attribute_type)ds_sysfs_get_attr_data(device, attr);
	if (ds_dev == NULL) {
		ds_nodev_error("No DW device sysfs mapping found");
		return 0;
	}

	switch (type) {
	case ATTR_KERNEL_HIB_PAGE_TABLE_SIZE:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_entries(ds_dev->page_table[0]));
	case ATTR_KERNEL_HIB_SIMPLE_PAGE_TABLE_SIZE:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_entries(ds_dev->page_table[0]));
	case ATTR_KERNEL_HIB_NUM_ACTIVE_PAGES:
		return scnprintf(buf, PAGE_SIZE, "%u\n",
			ds_page_table_num_active_pages(ds_dev->page_table[0]));
	default:
		ds_log_error(ds_dev, "Unknown attribute: %s", attr->attr.name);
		return 0;
	}
}
