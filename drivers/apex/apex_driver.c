/* Driver for the Apex chip.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "dw_ioctl.h"

#include "ds_generic.h"
#include "ds_interrupt.h"
#include "ds_logging.h"
#include "ds_page_table.h"
#include "ds_sysfs.h"

/* Constants */
#define DW_DEVICE_NAME "Apex"
#define DW_DRIVER_VERSION "0.1"

// CSRs are in BAR 2.
#define DW_BAR_INDEX 2

#define DW_PCI_VENDOR_ID 0x1ac1
#define DW_PCI_DEVICE_ID 0x089a

// TODO(vandwalle): remove when older Beagle FPGA are obsoleted
// #define DW_PCI_VENDOR_ID 0x1556
// #define DW_PCI_DEVICE_ID 0x1111

/* Bar Offsets. */
#define DW_BAR_OFFSET 0
#define DW_CM_OFFSET 0x1000000

/* The sizes of each DW BAR 2. */
#define DW_BAR_BYTES 0x100000
#define DW_CH_MEM_BYTES (PAGE_SIZE * MAX_NUM_COHERENT_PAGES)

/* The number of user-mappable memory ranges in BAR2 of a DW chip. */
#define NUM_REGIONS 2

/* The number of tensor nodes in a DW Chip. */
#define NUM_TENSOR_NODES 1

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
	APEX_BAR2_REG_SCU_BASE = 0x1A300,
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_SIZE = 0x46000,
	DW_BAR2_REG_KERNEL_HIB_EXTENDED_TABLE = 0x46008,
	DW_BAR2_REG_KERNEL_HIB_TRANSLATION_ENABLE = 0x46010,
	DW_BAR2_REG_KERNEL_HIB_INSTR_QUEUE_INTVECCTL = 0x46018,
	DW_BAR2_REG_KERNEL_HIB_INPUT_ACTV_QUEUE_INTVECCTL = 0x46020,
	DW_BAR2_REG_KERNEL_HIB_PARAM_QUEUE_INTVECCTL = 0x46028,
	DW_BAR2_REG_KERNEL_HIB_OUTPUT_ACTV_QUEUE_INTVECCTL = 0x46030,
	DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL = 0x46038,
	DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL = 0x46040,
	DW_BAR2_REG_KERNEL_HIB_FATAL_ERR_INTVECCTL = 0x46048,
	DW_BAR2_REG_KERNEL_HIB_DMA_PAUSE = 0x46050,
	DW_BAR2_REG_KERNEL_HIB_DMA_PAUSE_MASK = 0x46058,
	DW_BAR2_REG_KERNEL_HIB_STATUS_BLOCK_DELAY = 0x46060,
	DW_BAR2_REG_KERNEL_HIB_MSIX_PENDING_BIT_ARRAY0 = 0x46068,
	DW_BAR2_REG_KERNEL_HIB_MSIX_PENDING_BIT_ARRAY1 = 0x46070,
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE_INIT = 0x46078,
	DW_BAR2_REG_KERNEL_HIB_MSIX_TABLE_INIT = 0x46080,
	DW_BAR2_REG_KERNEL_WIRE_INT_PENDING_BIT_ARRAY = 0x48778,
	DW_BAR2_REG_KERNEL_WIRE_INT_MASK_ARRAY = 0x48780,
	APEX_BAR2_REG_USER_HIB_DMA_PAUSE = 0x486D8,
	APEX_BAR2_REG_USER_HIB_DMA_PAUSED = 0x486E0,
	APEX_BAR2_REG_IDLEGENERATOR_IDLEGEN_IDLEREGISTER = 0x4A000,
	DW_BAR2_REG_KERNEL_HIB_PAGE_TABLE = 0x50000,

	// Error registers - Used mostly for debug
	DW_BAR2_REG_USER_HIB_ERROR_STATUS = 0x86f0,
	DW_BAR2_REG_SCALAR_CORE_ERROR_STATUS = 0x41a0,
};

/* Addresses for packed registers. */
#define APEX_BAR2_REG_AXI_QUIESCE (APEX_BAR2_REG_SCU_BASE + 0x2C)
#define APEX_BAR2_REG_GCB_CLOCK_GATE (APEX_BAR2_REG_SCU_BASE + 0x14)
#define APEX_BAR2_REG_SCU_0 (APEX_BAR2_REG_SCU_BASE + 0xc)
#define APEX_BAR2_REG_SCU_1 (APEX_BAR2_REG_SCU_BASE + 0x10)
#define APEX_BAR2_REG_SCU_2 (APEX_BAR2_REG_SCU_BASE + 0x14)
#define APEX_BAR2_REG_SCU_3 (APEX_BAR2_REG_SCU_BASE + 0x18)
#define APEX_BAR2_REG_SCU_4 (APEX_BAR2_REG_SCU_BASE + 0x1c)
#define APEX_BAR2_REG_SCU_5 (APEX_BAR2_REG_SCU_BASE + 0x20)

#define SCU3_RG_PWR_STATE_OVR_BIT_OFFSET 26
#define SCU3_RG_PWR_STATE_OVR_MASK_WIDTH 2
#define SCU3_CUR_RST_GCB_BIT_MASK 0x10
#define SCU2_RG_RST_GCB_BIT_MASK 0xc

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

static int dw_enter_reset(struct ds_dev *ds_dev, uint type);

static int dw_quit_reset(struct ds_dev *ds_dev, uint type);

static bool is_gcb_in_reset(struct ds_dev *ds_dev);

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

static const struct pci_device_id dw_pci_ids[] = {
	{ PCI_DEVICE(DW_PCI_VENDOR_ID, DW_PCI_DEVICE_ID) }, { 0 }
};

/* The regions in the BAR2 space that can be mapped into user space. */
static const struct ds_mappable_region mappable_regions[NUM_REGIONS] = {
	{ 0x1A000, 0x1000 },
	{ 0x40000, 0x40000 },
};

static const struct ds_mappable_region cm_mappable_regions[1] = { { 0x0,
	DW_CH_MEM_BYTES } };

/* Interrupt descriptors for DW */
static struct ds_interrupt_desc dw_interrupts[] = {
	{
		DW_INTERRUPT_INSTR_QUEUE,
		DW_BAR2_REG_KERNEL_HIB_INSTR_QUEUE_INTVECCTL,
		UNPACKED,
	},
	{
		DW_INTERRUPT_INPUT_ACTV_QUEUE,
		DW_BAR2_REG_KERNEL_HIB_INPUT_ACTV_QUEUE_INTVECCTL,
		UNPACKED
	},
	{
		DW_INTERRUPT_PARAM_QUEUE,
		DW_BAR2_REG_KERNEL_HIB_PARAM_QUEUE_INTVECCTL,
		UNPACKED
	},
	{
		DW_INTERRUPT_OUTPUT_ACTV_QUEUE,
		DW_BAR2_REG_KERNEL_HIB_OUTPUT_ACTV_QUEUE_INTVECCTL,
		UNPACKED
	},
	{
		DW_INTERRUPT_SC_HOST_0,
		DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL,
		PACK_0
	},
	{
		DW_INTERRUPT_SC_HOST_1,
		DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL,
		PACK_1
	},
	{
		DW_INTERRUPT_SC_HOST_2,
		DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL,
		PACK_2
	},
	{
		DW_INTERRUPT_SC_HOST_3,
		DW_BAR2_REG_KERNEL_HIB_SC_HOST_INTVECCTL,
		PACK_3
	},
	{
		DW_INTERRUPT_TOP_LEVEL_0,
		DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL,
		PACK_0
	},
	{
		DW_INTERRUPT_TOP_LEVEL_1,
		DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL,
		PACK_1
	},
	{
		DW_INTERRUPT_TOP_LEVEL_2,
		DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL,
		PACK_2
	},
	{
		DW_INTERRUPT_TOP_LEVEL_3,
		DW_BAR2_REG_KERNEL_HIB_TOP_LEVEL_INTVECCTL,
		PACK_3
	},
	{
		DW_INTERRUPT_FATAL_ERR,
		DW_BAR2_REG_KERNEL_HIB_FATAL_ERR_INTVECCTL,
		UNPACKED
	},
};

static struct ds_driver_desc dw_desc = {
	.name = "dwn",
	.driver_version = DW_DRIVER_VERSION,
	.major = 120,
	.minor = 0,
	.module = THIS_MODULE,
	.pci_id_table = dw_pci_ids,

	.num_page_tables = NUM_TENSOR_NODES,
	.page_table_bar_index = DW_BAR_INDEX,
	.page_table_offsets = dw_page_table_offsets,
	.page_table_extended_bit = DW_EXTENDED_SHIFT,

	.bar_descriptions =
		{
			DS_UNUSED_BAR,
			DS_UNUSED_BAR,
			{ DW_BAR_BYTES, (VM_WRITE | VM_READ), DW_BAR_OFFSET,
				NUM_REGIONS, mappable_regions, PCI_BAR },
			DS_UNUSED_BAR,
			DS_UNUSED_BAR,
			{ DW_CH_MEM_BYTES, (VM_WRITE | VM_READ), DW_CM_OFFSET,
				1, cm_mappable_regions, COHERENT_MEMORY },
		},

	.interrupt_type = PCI_MSIX,
	.interrupt_bar_index = DW_BAR_INDEX,
	.num_interrupts = DW_INTERRUPT_COUNT,
	.interrupts = dw_interrupts,
	.interrupt_pack_width = 7,

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
MODULE_DESCRIPTION("Google Apex driver");
MODULE_VERSION(DW_DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rob Springer <rspringer@google.com>");
MODULE_DEVICE_TABLE(pci, dw_pci_ids);
module_init(dw_init);
module_exit(dw_exit);

// Allows device to enter power save upon driver close().
static int allow_power_save = 0;

// Allows SW based clock gating.
static int allow_sw_clock_gating = 0;

// Allows HW based clock gating.
// Note: this is not mutual exclusive with SW clock gating.
static int allow_hw_clock_gating = 1;

// Act as if only GCB is instantiated.
static int bypass_top_level = 0;

module_param(allow_power_save, int, 0644);
module_param(allow_sw_clock_gating, int, 0644);
module_param(allow_hw_clock_gating, int, 0644);
module_param(bypass_top_level, int, 0644);

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

	ds_log_error(ds_dev, "dw_add_dev_cb.");

	dw_reset(ds_dev, 0);

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
	return 0;
}

static int dw_sysfs_setup_cb(struct ds_dev *ds_dev)
{
	return ds_sysfs_create_entries(ds_dev->dev_info.device, dw_sysfs_attrs);
}

/* On device open, we want to perform a core reinit reset. */
static int dw_device_open_cb(struct ds_dev *ds_dev)
{
	return ds_reset_nolock(ds_dev, DW_CHIP_REINIT_RESET);
}

/**
 * dw_get_status - Set device status.
 * @dev: Apex device struct.
 *
 * Description: Check the device status registers and set the driver status
 *		to ALIVE or DEAD.
 *
 *		Returns 0 if status is ALIVE, a negative error number otherwise.
 */
static int dw_get_status(struct ds_dev *ds_dev)
{

	// Apex, always returns ALIVE for now
	return DS_STATUS_ALIVE;
}

/**
 * dw_device_cleanup - Clean up Apex HW after close.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the Apex hardware. Called on final close via
 * device_close_cb.
 */
static int dw_device_cleanup(struct ds_dev *ds_dev)
{
	u64 scalar_error;
	u64 hib_error;
	int ret = 0;

	hib_error = ds_dev_read_64(
		ds_dev, DW_BAR_INDEX, DW_BAR2_REG_USER_HIB_ERROR_STATUS);
	scalar_error = ds_dev_read_64(
		ds_dev, DW_BAR_INDEX, DW_BAR2_REG_SCALAR_CORE_ERROR_STATUS);

	ds_log_info(ds_dev,
		"dw_device_cleanup 0x%p hib_error 0x%llx scalar_error 0x%llx.",
		ds_dev, hib_error, scalar_error);

	if (allow_power_save) {
		ret = dw_enter_reset(ds_dev, DW_CHIP_REINIT_RESET);
	}
	return ret;
}

/**
 * dw_reset - Quits reset.
 * @ds_dev: DS device pointer.
 *
 * Description: Resets the hardware, then quits reset.
 * Called on device open.
 *
 */
static int dw_reset(struct ds_dev *ds_dev, uint type)
{
	int ret;

	if (bypass_top_level) {
		return 0;
	}

	ds_log_debug(ds_dev, "dw_reset.");

	if (!is_gcb_in_reset(ds_dev)) {
		/* we are not in reset - toggle the reset bit so as to force
		 * re-init of custom block */
		ds_log_debug(ds_dev, "dw_reset: toggle reset.");

		if ((ret = dw_enter_reset(ds_dev, type))) {
			return ret;
		}
	}
	ret = dw_quit_reset(ds_dev, type);

	return ret;
}

/*
 * Enters GCB reset state.
 */
static int dw_enter_reset(struct ds_dev *ds_dev, uint type)
{
	u32 val0, val1;

	if (bypass_top_level) {
		return 0;
	}

	ds_log_debug(ds_dev, "dw_enter_reset.");

	/* Unconditional logs, temporary for SoC validation : print values of
	 * SCU2/3 before a reset
	 */
	val0 = ds_dev_read_32(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2);
	val1 = ds_dev_read_32(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
	ds_log_error(ds_dev, "Enter Reset: SCU2=0x%x SCU3=0x%x", val0, val1);

	/*
	 * Software reset:
	 * Enable sleep mode
	 *  - Software force GCB idle
	 *    - Enable GCB idle
	 */
	ds_read_modify_write_64(ds_dev, DW_BAR_INDEX,
		APEX_BAR2_REG_IDLEGENERATOR_IDLEGEN_IDLEREGISTER, 0x0, 1, 32);

	/*    - Initiate DMA pause */
	ds_dev_write_64(
		ds_dev, 1, DW_BAR_INDEX, APEX_BAR2_REG_USER_HIB_DMA_PAUSE);

	/*    - Wait for DMA pause complete. */
	if (ds_wait_async(ds_dev, DW_BAR_INDEX,
		    APEX_BAR2_REG_USER_HIB_DMA_PAUSED, 1, 1, DW_RESET_DELAY,
		    DW_RESET_RETRY)) {
		ds_log_error(ds_dev,
			"DMAs did not quiece within timeout (%d ms)",
			DW_RESET_RETRY * DW_RESET_DELAY);
		return -EINVAL;
	}

	/*  - Enable GCB reset (0x1 to rg_rst_gcb) */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2, 0x1, 2, 2);

	/*  - Enable GCB clock Gate (0x1 to rg_gated_gcb) */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2, 0x1, 2, 18);

	/*  - Enable GCB memory shut down (0x3 to rg_force_ram_sd) */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3, 0x3, 2, 14);

	/*    - Wait for RAM shutdown. */
	if (ds_wait_async(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3, 1 << 6,
		    1 << 6, DW_RESET_DELAY, DW_RESET_RETRY)) {
		ds_log_error(ds_dev,
			"RAM did not shut down within timeout (%d ms)",
			DW_RESET_RETRY * DW_RESET_DELAY);
		return -EINVAL;
	}

	return 0;
}

/*
 * Quits GCB reset state.
 */
static int dw_quit_reset(struct ds_dev *ds_dev, uint type)
{
	u32 val0, val1;

	if (bypass_top_level) {
		return 0;
	}

	ds_log_debug(ds_dev, "dw_quit_reset.");

	/*
	 * Disable sleep mode:
	 *  - Disable GCB memory shut down:
	 *    - b00: Not forced (HW controlled)
	 *    - b1x: Force disable
	 */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3, 0x0, 2, 14);

	/*
	 *  - Disable software clock gate:
	 *    - b00: Not forced (HW controlled)
	 *    - b1x: Force disable
	 */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2, 0x0, 2, 18);

	/*
	 *  - Disable GCB reset (rg_rst_gcb):
	 *    - b00: Not forced (HW controlled)
	 *    - b1x: Force disable = Force not Reset
	 */
	ds_read_modify_write_32(
		ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2, 0x2, 2, 2);

	/*    - Wait for RAM enable. */
	if (ds_wait_async(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3, 1 << 6, 0,
		    DW_RESET_DELAY, DW_RESET_RETRY)) {
		ds_log_error(ds_dev,
			"RAM did not enable within timeout (%d ms)",
			DW_RESET_RETRY * DW_RESET_DELAY);
		return -EINVAL;
	}

	/*    - Wait for Reset complete. */
	if (ds_wait_async(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3,
		    SCU3_CUR_RST_GCB_BIT_MASK, 0, DW_RESET_DELAY,
		    DW_RESET_RETRY)) {
		ds_log_error(ds_dev,
			"GCB did not leave reset within timeout (%d ms)",
			DW_RESET_RETRY * DW_RESET_DELAY);
		return -EINVAL;
	}

	if (!allow_hw_clock_gating) {
		val0 = ds_dev_read_32(
			ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
		/* Inactive and Sleep mode are disabled. */
		ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
			APEX_BAR2_REG_SCU_3, 0x3,
			SCU3_RG_PWR_STATE_OVR_MASK_WIDTH,
			SCU3_RG_PWR_STATE_OVR_BIT_OFFSET);
		val1 = ds_dev_read_32(
			ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
		ds_log_error(ds_dev, "Disallow HW clock gating 0x%x -> 0x%x",
			val0, val1);
	} else {
		val0 = ds_dev_read_32(
			ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
		/* Inactive mode enabled - Sleep mode disabled. */
		ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
			APEX_BAR2_REG_SCU_3, 2,
			SCU3_RG_PWR_STATE_OVR_MASK_WIDTH,
			SCU3_RG_PWR_STATE_OVR_BIT_OFFSET);
		val1 = ds_dev_read_32(
			ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
		ds_log_error(ds_dev, "Allow HW clock gating 0x%x -> 0x%x", val0,
			val1);
	}

	/* Temporary for SoC validation - print values of SCU2/3 after reset.
	 */
	val0 = ds_dev_read_32(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_2);
	val1 = ds_dev_read_32(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);
	ds_log_error(ds_dev, "Quit Reset: SCU2=0x%x SCU3=0x%x", val0, val1);

	return 0;
}

/*
 * Determines if GCB is in reset state.
 */
static bool is_gcb_in_reset(struct ds_dev *ds_dev)
{
	u32 val = ds_dev_read_32(ds_dev, DW_BAR_INDEX, APEX_BAR2_REG_SCU_3);

	/* Masks rg_rst_gcb bit of SCU_CTRL_2 */
	return (val & SCU3_CUR_RST_GCB_BIT_MASK);
}

/*
 * DW_ioctl_check_permissions: Check permissions for Apex ioctls.
 * @file: File pointer from ioctl.
 * @cmd: ioctl command.
 *
 * Returns 1 if the current user may execute this ioctl, and 0 otherwise.
 */
static uint dw_ioctl_check_permissions(struct file *filp, uint cmd)
{
	struct ds_dev *ds_dev = filp->private_data;
	int root = capable(CAP_SYS_ADMIN);
	int is_owner = ds_dev->dev_info.ownership.is_owned &&
		       current->tgid == ds_dev->dev_info.ownership.owner;

	if (root || is_owner)
		return 1;
	return 0;
}

/*
 * dw_ioctl: Apex-specific ioctl handler.
 */
static long dw_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct ds_dev *ds_dev = filp->private_data;

	if (!dw_ioctl_check_permissions(filp, cmd))
		return -EPERM;

	switch (cmd) {
	case DW_IOCTL_GATE_CLOCK:
		return dw_clock_gating(ds_dev, arg);
	default:
		return -ENOTTY; /* unknown command */
	}
}

/*
 * dw_clock_gating: Gates or un-gates Apex clock.
 * @ds_dev: DS device pointer.
 * @arg: User ioctl arg, in this case to a dw_gate_clock_ioctl struct.
 */
static long dw_clock_gating(struct ds_dev *ds_dev, ulong arg)
{
	struct dw_gate_clock_ioctl ibuf;

	if (bypass_top_level) {
		return 0;
	}

	if (allow_sw_clock_gating) {
		if (copy_from_user(&ibuf, (void __user *)arg, sizeof(ibuf)))
			return -EFAULT;

		ds_log_error(ds_dev, "dw_clock_gating %llu", ibuf.enable);

		if (ibuf.enable) {
			// Quiesce AXI, gate GCB clock.
			ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
				APEX_BAR2_REG_AXI_QUIESCE, 0x1, 1, 16);
			ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
				APEX_BAR2_REG_GCB_CLOCK_GATE, 0x1, 2, 18);
		} else {
			// Un-gate GCB clock, un-quiesce AXI.
			ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
				APEX_BAR2_REG_GCB_CLOCK_GATE, 0x0, 2, 18);
			ds_read_modify_write_32(ds_dev, DW_BAR_INDEX,
				APEX_BAR2_REG_AXI_QUIESCE, 0x0, 1, 16);
		}
	}
	return 0;
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
