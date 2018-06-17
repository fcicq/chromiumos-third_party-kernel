// SPDX-License-Identifier: GPL-2.0
/*
 * ISHTP client driver for ISH firmware loading
 *
 * Copyright (c) 2018, Intel Corporation.
 */

#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "ipc/hw-ish.h"
#include "ishtp/client.h"
#include "ishtp/ishtp-dev.h"

/* ISH TX/RX ring buffer pool size */
#define LOADER_CL_RX_RING_SIZE			32
#define LOADER_CL_TX_RING_SIZE			16

/* ISH Loader Host Commands */
enum ish_loader_commands {
	LOADER_SET_MODULE_PROPERTIES = 0,
	LOADER_TX_FRAGMENT,
	LOADER_START,
};

/* Command bit mask */
#define	CMD_MASK				GENMASK(6, 0)
#define	IS_RESPONSE				BIT(7)

/*
 * ISH firmware max delay for one time sending failure is 1Hz,
 * and firmware will retry 2 times, so 3Hz is used for timeout.
 */
#define ISHTP_SEND_TIMEOUT			(3 * HZ)

/* ISH Transport Loader client unique GUID */
static const uuid_le loader_ishtp_guid = UUID_LE(0xc804d06a, 0x55bd, 0x4ea7,
						0xad, 0xed, 0x1e, 0x31, 0x22,
						0x8c, 0x76, 0xdc);
/* TODO
static const guid_t loader_ishtp_guid =
	GUID_INIT(0xc804d06a, 0x55bd, 0x4ea7,
		  0xad, 0xed, 0x1e, 0x31, 0x22, 0x8c, 0x76, 0xdc);
*/
#define FILENAME_SIZE				40

/**
 * struct loader_msg_hdr - Command header for ISH Loader commands.
 * @command:		One of the LOADER_* commands. Bit 7 is the response.
 * @status:		Command response status. Non 0, is error condition.
 *
 * This structure is used as header for every command/data sent to ISH
 * Loader.
 */
struct loader_msg_hdr {
	u8 command;
	u8 reserved[2];
	u8 status;
};

struct loader_msg {
	struct loader_msg_hdr hdr;
};

struct loader_module_properties {
	struct loader_msg_hdr hdr;
	u32 total_size;
	u32 reserved;
};

struct loader_tx_frag_req {
	struct loader_msg_hdr hdr;
	u32 fragment_offset;
	u32 fragment_size;
	u8 data[] ____cacheline_aligned; /* variable len payload here */
};

struct loader_start {
	struct loader_msg_hdr hdr;
};

/* ISH version for Host FW load */
struct ish_version {
	/* SoC ISH IP version */
	char ish_ip_version[FILENAME_SIZE];
	/* Platform variant & version */
	char platform_variant[FILENAME_SIZE];
};

/**
 * struct ishtp_cl_data - Encapsulate per ISH TP Client
 * @init_done:		Init process completed successfully
 * @set_mod_prop_done	set_mod_prop loader command completed successfully
 * @tx_frag_done	tx_frag loader command completed successfully
 * @ldr_start_done	start loader command completed successfully
 * @suspended:		System is under suspend state or in progress
 * @ldr_cmd_resp_wait:	Wait queue for Host FW loading, where the
 *			client send message to ISH FW and wait for response
 * @loader_ishtp_cl	ISH transport firmware client information
 * @bad_recv_cnt:	Running count of packets received with error
 *
 * This structure is used to store completion flags per client
 */
struct ishtp_cl_data {
	struct ishtp_cl *loader_ishtp_cl;
	struct ishtp_cl_device *cl_device;

	/* completion flags */
	bool init_done;
	bool set_mod_prop_done;
	bool tx_frag_done;
	bool ldr_start_done;
	bool suspended;

	/* Wait queue for ISHFW message event */
	wait_queue_head_t ldr_cmd_resp_wait;

	struct work_struct work_ishtp_reset;

	/* Statistics */
	unsigned int bad_recv_cnt;
};

static bool get_ish_hw_version(struct ishtp_cl_data *client_data,
				       struct ish_version *v)
{
	bool flag = true;

	struct ishtp_cl_device *cl_device = client_data->cl_device;
	unsigned short pci_devid = cl_device->ishtp_dev->pdev->device;

	switch(pci_devid) {
	case SPT_Ax_DEVICE_ID:
		/* ISH 3.0: KBL */
		strcpy(v->ish_ip_version, "3.0");
		break;
	case CNL_Ax_DEVICE_ID:
		/* ISH 5.0: WHL */
		strcpy(v->ish_ip_version, "5.0");
		break;
	default:
		/* Not all SoCs support host FW load for ISH */
		dev_err(&cl_device->dev, "Unsupported SoC pci devid %04x\n",
			pci_devid);
		flag = false;
	}

	return flag;
}

static void get_baseboard_info(struct ish_version *v)
{
	const char *board_name = NULL, *board_version = NULL;

	/* Read Baseboard name & version */
	board_name = dmi_get_system_info(DMI_BOARD_NAME);
	board_version = dmi_get_system_info(DMI_BOARD_VERSION);

	if (board_name && board_version)
		snprintf(v->platform_variant, sizeof(v->platform_variant),
			"%s%s", board_name, board_version);
	else
		/* If no Baseboard info available, load "generic" FW */
		strcpy(v->platform_variant, "generic");
}

/**
 * loader_report_bad_packets() - Report bad packets
 * @loader_ishtp_cl:	Client instance to get stats
 * @recv_buf:		Raw received host interface message
 *
 * Dumps error in case bad packet is received
 */
static void loader_report_bad_packet(struct ishtp_cl *loader_ishtp_cl,
				     void *recv_buf)
{
	struct loader_msg *recv_msg = recv_buf;
	struct ishtp_cl_data *client_data = loader_ishtp_cl->client_data;

	dev_err(&client_data->cl_device->dev,
		"BAD packet: command=%02lx is_response=%u status=%02x total_bad=%u\n",
		recv_msg->hdr.command & CMD_MASK,
		recv_msg->hdr.command & IS_RESPONSE ? 1 : 0,
		recv_msg->hdr.status,
		client_data->bad_recv_cnt);
}

/**
 * ish_loader_process_recv() - Received and parse incoming packet
 * @loader_ishtp_cl:	Client instance to get stats
 * @recv_buf:		Raw received host interface message
 * @data_len:		length of the message
 *
 * Parse the incoming packet. If it is a response packet then it will update
 * per instance flags and wake up the caller waiting to for the response.
 */
static void ish_loader_process_recv(struct ishtp_cl *loader_ishtp_cl,
				    void *recv_buf, size_t data_len)
{
	struct loader_msg *recv_msg = recv_buf;
	struct ishtp_cl_data *client_data = loader_ishtp_cl->client_data;

	dev_dbg(&client_data->cl_device->dev, "command=%02lx\n",
		recv_msg->hdr.command & CMD_MASK);

	/* Sanity checks */
	if (data_len < sizeof(struct loader_msg_hdr)) {
		dev_err(&client_data->cl_device->dev,
			"data size %zu is less than header %zu\n",
			data_len,
			sizeof(struct loader_msg_hdr));
		client_data->bad_recv_cnt++;
		loader_report_bad_packet(loader_ishtp_cl, recv_msg);
		ish_hw_reset(loader_ishtp_cl->dev);
		return;
	}

	switch (recv_msg->hdr.command & CMD_MASK) {
	case LOADER_SET_MODULE_PROPERTIES:
		/* Sanity checks */
		if (!(recv_msg->hdr.command & IS_RESPONSE)) {
			dev_err(&client_data->cl_device->dev,
				"Illegal LOADER_SET_MODULE_PROPERTIES response\n");
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else if (recv_msg->hdr.status) {
			dev_err(&client_data->cl_device->dev,
				"LOADER_SET_MODULE_PROPERTIES returned status %d\n",
				recv_msg->hdr.status);
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else {
			client_data->set_mod_prop_done = true;
			wake_up_interruptible(&client_data->ldr_cmd_resp_wait);
		}

		break;

	case LOADER_TX_FRAGMENT:
		/* Sanity checks */
		if (!(recv_msg->hdr.command & IS_RESPONSE)) {
			dev_err(&client_data->cl_device->dev,
				"Illegal LOADER_TX_FRAGMENT response\n");
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else if (recv_msg->hdr.status) {
			dev_err(&client_data->cl_device->dev,
				"LOADER_TX_FRAGMENT returned status %d\n",
				recv_msg->hdr.status);
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else {
			client_data->tx_frag_done = true;
			wake_up_interruptible(&client_data->ldr_cmd_resp_wait);
		}

		break;

	case LOADER_START:
		/* Sanity checks */
		if (!(recv_msg->hdr.command & IS_RESPONSE)) {
			dev_err(&client_data->cl_device->dev,
				"Illegal LOADER_START response\n");
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else if (recv_msg->hdr.status) {
			dev_err(&client_data->cl_device->dev,
				"LOADER_START returned status %d\n",
				recv_msg->hdr.status);
			client_data->bad_recv_cnt++;
			loader_report_bad_packet(loader_ishtp_cl, recv_msg);
			ish_hw_reset(loader_ishtp_cl->dev);

		} else {
			client_data->ldr_start_done = true;
			wake_up_interruptible(&client_data->ldr_cmd_resp_wait);
		}

		break;

	default:
		dev_err(&client_data->cl_device->dev,
			"Invalid command=%02lx\n",
			recv_msg->hdr.command & CMD_MASK);
		client_data->bad_recv_cnt++;
		loader_report_bad_packet(loader_ishtp_cl, recv_msg);
		ish_hw_reset(loader_ishtp_cl->dev);
		break;
	}
}

/**
 * ish_loader_cl_event_cb() - bus driver callback for incoming message
 * @device:	Pointer to the the ishtp client device for which this
 *		message is targeted
 *
 * Remove the packet from the list and process the message by calling
 * ish_loader_process_recv
 */
static void ish_loader_cl_event_cb(struct ishtp_cl_device *cl_device)
{
	size_t r_length;
	struct ishtp_cl_rb *rb_in_proc;
	struct ishtp_cl_data *client_data;
	struct ishtp_cl	*loader_ishtp_cl = ishtp_get_drvdata(cl_device);

	client_data = loader_ishtp_cl->client_data;

	while ((rb_in_proc = ishtp_cl_rx_get_rb(loader_ishtp_cl)) != NULL) {
		if (!rb_in_proc->buffer.data)
			return;

		r_length = rb_in_proc->buf_idx;

		/* decide what to do with received data */
		ish_loader_process_recv(loader_ishtp_cl,
					rb_in_proc->buffer.data,
					r_length);

		ishtp_cl_io_rb_recycle(rb_in_proc);
	}
}

/**
 * ish_load_fw_from_host() - Loads ISH firmware from host
 * @client_data:	client data instance
 *
 * This function loads the ISH firmware from host system to ISH sram and
 * starts execution
 *
 * Return: 0 on success, non zero on error
 */
static int ish_load_fw_from_host(struct ishtp_cl_data *client_data)
{
	int rv;
	size_t fw_size;
	const struct firmware *fw = NULL;
	char filename[FILENAME_SIZE];
	u32 fragment_offset, fragment_size;
	u32 buf_payload_max_size, buf_max_size;

	struct ish_version version;
	struct loader_start ldr_start;
	struct loader_module_properties ldr_module_prop;
	struct loader_tx_frag_req *ldr_tx_frag_req = NULL;
	struct ishtp_cl *loader_ishtp_cl = client_data->loader_ishtp_cl;

	/* Sanity checks */
	if (!client_data->init_done) {
		dev_err(&client_data->cl_device->dev,
			"Flag init_done is false\n");
		rv = -EBUSY;
		goto end_error;
	}

	/*
	 * Kernel loads the ISH firmware from rootfs /lib/firmware
	 * The filename to load is determined by ISH IP version,
	 * and platform name & version
	 *	Filename : ish<ish_ip_version>-<platform-version>-fw.bin
	 *	Example  : ish3.0-intelrvp1.0-fw.bin
	 */
	rv = get_ish_hw_version(client_data, &version);
	if (!rv)
		goto end_error;

	get_baseboard_info(&version);

	snprintf(filename, sizeof(filename), "intel/ish%s-%s-fw.bin",
		version.ish_ip_version, version.platform_variant);

	/* Direct load firmware */
	rv = request_firmware(&fw, filename, &client_data->cl_device->dev);
	if (rv < 0)
		goto end_error;

	/* Step 1: Send firmware module properties loader-firmware */
	fw_size = fw->size;
	client_data->set_mod_prop_done = false;
	memset(&ldr_module_prop, 0, sizeof(struct loader_module_properties));
	ldr_module_prop.hdr.command = LOADER_SET_MODULE_PROPERTIES;
	ldr_module_prop.total_size = fw_size;
	rv = ishtp_cl_send(loader_ishtp_cl,
			   (unsigned char *)&ldr_module_prop,
			   sizeof(struct loader_module_properties));
	if (rv) {
		dev_err(&client_data->cl_device->dev,
			"ishtp_cl_send error %d\n", rv);
		goto end_err_release;
	}

	wait_event_interruptible_timeout(client_data->ldr_cmd_resp_wait,
					 client_data->set_mod_prop_done,
					 ISHTP_SEND_TIMEOUT);
	if (!client_data->set_mod_prop_done) {
		dev_err(&client_data->cl_device->dev,
			"Timed out for flag set_mod_prop_done\n");
		rv = -ETIMEDOUT;
		goto end_err_release;
	}

	/* Step 2: Send the firmware image to be loaded to ISH sram */
	buf_max_size = client_data->cl_device->fw_client->props.max_msg_length;

	/* Buffer size should be multiple of cacheline size */
	if (buf_max_size % L1_CACHE_BYTES != 0) {
		rv = -EINVAL;
		goto end_err_release;
	}

	buf_payload_max_size = buf_max_size -
				offsetof(struct loader_tx_frag_req, data);
	ldr_tx_frag_req = kzalloc(buf_max_size, GFP_KERNEL);
	if (!ldr_tx_frag_req) {
		rv = -ENOMEM;
		goto end_err_release;
	}

	fragment_offset = 0;

	while (fragment_offset < fw_size) {
		client_data->tx_frag_done = false;
		ldr_tx_frag_req->hdr.command = LOADER_TX_FRAGMENT;
		ldr_tx_frag_req->fragment_offset = fragment_offset;

		if (fw_size - fragment_offset < buf_payload_max_size)
			fragment_size = fw_size - fragment_offset;
		else
			fragment_size = buf_payload_max_size;

		ldr_tx_frag_req->fragment_size = fragment_size;
		memcpy(ldr_tx_frag_req->data,
		       &fw->data[fragment_offset], fragment_size);

		rv = ishtp_cl_send(loader_ishtp_cl,
				   (unsigned char *)ldr_tx_frag_req,
				   buf_max_size);
		if (rv) {
			dev_err(&client_data->cl_device->dev,
				"ishtp_cl_send error %d\n", rv);
			goto end_err_release;
		}

		wait_event_interruptible_timeout(client_data->ldr_cmd_resp_wait,
						 client_data->tx_frag_done,
						 ISHTP_SEND_TIMEOUT);
		if (!client_data->tx_frag_done) {
			dev_err(&client_data->cl_device->dev,
				"Timed out for flag tx_frag_done\n");
			rv = -ETIMEDOUT;
			goto end_err_release;
		}

		fragment_offset += fragment_size;
	}

	/* Set 3: Send LOADER_START */
	client_data->ldr_start_done = false;
	memset(&ldr_start, 0, sizeof(struct loader_start));
	ldr_start.hdr.command = LOADER_START;

	rv = ishtp_cl_send(loader_ishtp_cl,
			   (unsigned char *)&ldr_start,
			   sizeof(struct loader_start));
	if (rv) {
		dev_err(&client_data->cl_device->dev,
			"ishtp_cl_send error %d\n", rv);
		goto end_err_release;
	}

	wait_event_interruptible_timeout(client_data->ldr_cmd_resp_wait,
					 client_data->ldr_start_done,
					 ISHTP_SEND_TIMEOUT);
	if (!client_data->ldr_start_done) {
		dev_err(&client_data->cl_device->dev,
			"Timed out for ldr_start_done\n");
		rv = -ETIMEDOUT;
		goto end_err_release;
	}

	release_firmware(fw);
	kfree(ldr_tx_frag_req);
	dev_info(&client_data->cl_device->dev, "ISH firmware loaded\n");
	return 0;

end_err_release:
	release_firmware(fw);
end_error:
	kfree(ldr_tx_frag_req);
	dev_err(&client_data->cl_device->dev,
		"ISH host firmware load failed %d\n", rv);
	return rv;
}

/**
 * loader_ishtp_cl_init() - Init function for ISHTP client
 * @loader_ishtp_cl:	ISHTP client instance
 * @reset:		true if called for init after reset
 *
 * This function complete the initializtion of the client.
 *
 * Return: 0 on success, non zero on error
 */
static int loader_ishtp_cl_init(struct ishtp_cl *loader_ishtp_cl, int reset)
{
	struct ishtp_device *dev;
	struct ishtp_cl_data *client_data = loader_ishtp_cl->client_data;
	struct ishtp_fw_client *fw_client;
	int rv;

	dev_dbg(&client_data->cl_device->dev, "reset flag: %d\n", reset);

	rv = ishtp_cl_link(loader_ishtp_cl, ISHTP_HOST_CLIENT_ID_ANY);
	if (rv) {
		dev_err(&client_data->cl_device->dev,
			"ishtp_cl_link failed\n");
		return	-ENOMEM;
	}

	client_data->init_done = 0;

	dev = loader_ishtp_cl->dev;

	/* Connect to FW client */
	loader_ishtp_cl->rx_ring_size = LOADER_CL_RX_RING_SIZE;
	loader_ishtp_cl->tx_ring_size = LOADER_CL_TX_RING_SIZE;

	fw_client = ishtp_fw_cl_get_client(dev, &loader_ishtp_guid);
	if (!fw_client) {
		dev_err(&client_data->cl_device->dev,
			"ish client uuid not found\n");
		return -ENOENT;
	}

	loader_ishtp_cl->fw_client_id = fw_client->client_id;
	loader_ishtp_cl->state = ISHTP_CL_CONNECTING;
	rv = ishtp_cl_connect(loader_ishtp_cl);
	if (rv) {
		dev_err(&client_data->cl_device->dev,
			"client connect fail\n");
		goto err_cl_unlink;
	}

	dev_dbg(&client_data->cl_device->dev, "client connected\n");

	/* Register read callback */
	ishtp_register_event_cb(loader_ishtp_cl->device,
				ish_loader_cl_event_cb);

	client_data->init_done = 1;
	client_data->suspended = false;
	dev_dbg(&client_data->cl_device->dev, "successful init\n");

	return 0;

err_cl_unlink:
	ishtp_cl_unlink(loader_ishtp_cl);
	return rv;
}

/**
 * loader_ishtp_cl_deinit() - Deinit function for ISHTP client
 * @loader_ishtp_cl:	ISHTP client instance
 *
 * Unlink and free loader client
 */
static void loader_ishtp_cl_deinit(struct ishtp_cl *loader_ishtp_cl)
{
	ishtp_cl_unlink(loader_ishtp_cl);
	ishtp_cl_flush_queues(loader_ishtp_cl);

	/* disband and free all Tx and Rx client-level rings */
	ishtp_cl_free(loader_ishtp_cl);
}

static void loader_ishtp_cl_reset_handler(struct work_struct *work)
{
	struct ishtp_cl_data *client_data;
	struct ishtp_cl *loader_ishtp_cl;
	struct ishtp_cl_device *cl_device;
	int retry;
	int rv;

	client_data = container_of(work, struct ishtp_cl_data,
				   work_ishtp_reset);

	loader_ishtp_cl = client_data->loader_ishtp_cl;
	cl_device = client_data->cl_device;

	dev_dbg(&cl_device->dev, "%s\n", __func__);

	loader_ishtp_cl_deinit(loader_ishtp_cl);

	loader_ishtp_cl = ishtp_cl_allocate(cl_device->ishtp_dev);
	if (!loader_ishtp_cl)
		return;

	ishtp_set_drvdata(cl_device, loader_ishtp_cl);
	loader_ishtp_cl->client_data = client_data;
	client_data->loader_ishtp_cl = loader_ishtp_cl;

	retry = 3;
	do {
		rv = loader_ishtp_cl_init(loader_ishtp_cl, 1);
		if (!rv)
			break;
		dev_err(&client_data->cl_device->dev, "Retry reset init\n");
	} while (--retry);

	if (rv) {
		dev_err(&client_data->cl_device->dev, "Reset Failed\n");
		return;
	}

	/* ISH firmware loading from host */
	ish_load_fw_from_host(client_data);
}

/**
 * loader_ishtp_cl_probe() - ISHTP client driver probe
 * @cl_device:		ISHTP client device instance
 *
 * This function gets called on device create on ISHTP bus
 *
 * Return: 0 on success, non zero on error
 */
static int loader_ishtp_cl_probe(struct ishtp_cl_device *cl_device)
{
	struct ishtp_cl *loader_ishtp_cl;
	struct ishtp_cl_data *client_data;
	int rv;

	if (!cl_device)
		return	-ENODEV;

	if (uuid_le_cmp(loader_ishtp_guid,
			cl_device->fw_client->props.protocol_name) != 0)
		return	-ENODEV;

	client_data = devm_kzalloc(&cl_device->dev, sizeof(*client_data),
				   GFP_KERNEL);
	if (!client_data)
		return -ENOMEM;

	loader_ishtp_cl = ishtp_cl_allocate(cl_device->ishtp_dev);
	if (!loader_ishtp_cl)
		return -ENOMEM;

	ishtp_set_drvdata(cl_device, loader_ishtp_cl);
	loader_ishtp_cl->client_data = client_data;
	client_data->loader_ishtp_cl = loader_ishtp_cl;
	client_data->cl_device = cl_device;

	init_waitqueue_head(&client_data->ldr_cmd_resp_wait);

	INIT_WORK(&client_data->work_ishtp_reset,
		  loader_ishtp_cl_reset_handler);

	rv = loader_ishtp_cl_init(loader_ishtp_cl, 0);
	if (rv) {
		ishtp_cl_free(loader_ishtp_cl);
		return rv;
	}
	ishtp_get_device(cl_device);

	/* ISH firmware loading from host */
	ish_load_fw_from_host(client_data);

	return 0;
}

/**
 * loader_ishtp_cl_remove() - ISHTP client driver remove
 * @cl_device:		ISHTP client device instance
 *
 * This function gets called on device remove on ISHTP bus
 *
 * Return: 0
 */
static int loader_ishtp_cl_remove(struct ishtp_cl_device *cl_device)
{
	struct ishtp_cl	*loader_ishtp_cl = ishtp_get_drvdata(cl_device);

	loader_ishtp_cl->state = ISHTP_CL_DISCONNECTING;
	ishtp_cl_disconnect(loader_ishtp_cl);
	ishtp_put_device(cl_device);
	loader_ishtp_cl_deinit(loader_ishtp_cl);
	loader_ishtp_cl = NULL;

	return 0;
}

/**
 * loader_ishtp_cl_reset() - ISHTP client driver reset
 * @cl_device:		ISHTP client device instance
 *
 * This function gets called on device reset on ISHTP bus
 *
 * Return: 0
 */
static int loader_ishtp_cl_reset(struct ishtp_cl_device *cl_device)
{
	struct ishtp_cl_data *client_data;
	struct ishtp_cl	*loader_ishtp_cl = ishtp_get_drvdata(cl_device);

	client_data = loader_ishtp_cl->client_data;

	dev_dbg(&client_data->cl_device->dev, "%s\n", __func__);

	schedule_work(&client_data->work_ishtp_reset);

	return 0;
}

#define to_ishtp_cl_device(d) container_of(d, struct ishtp_cl_device, dev)

/**
 * loader_ishtp_cl_suspend() - ISHTP client driver suspend
 * @device:	device instance
 *
 * This function gets called on system suspend
 *
 * Return: 0
 */
static int __maybe_unused loader_ishtp_cl_suspend(struct device *device)
{
	struct ishtp_cl_data *client_data;
	struct ishtp_cl_device *cl_device = to_ishtp_cl_device(device);
	struct ishtp_cl	*loader_ishtp_cl = ishtp_get_drvdata(cl_device);

	client_data = loader_ishtp_cl->client_data;

	client_data->suspended = true;

	return 0;
}

/**
 * loader_ishtp_cl_resume() - ISHTP client driver resume
 * @device:	device instance
 *
 * This function gets called on system resume
 *
 * Return: 0
 */
static int __maybe_unused loader_ishtp_cl_resume(struct device *device)
{
	struct ishtp_cl_data *client_data;
	struct ishtp_cl_device *cl_device = to_ishtp_cl_device(device);
	struct ishtp_cl	*loader_ishtp_cl = ishtp_get_drvdata(cl_device);

	client_data = loader_ishtp_cl->client_data;

	client_data->suspended = false;

	return 0;
}

static SIMPLE_DEV_PM_OPS(loader_ishtp_pm_ops, loader_ishtp_cl_suspend,
			 loader_ishtp_cl_resume);

static struct ishtp_cl_driver	loader_ishtp_cl_driver = {
	.name = "ish-loader",
	.probe = loader_ishtp_cl_probe,
	.remove = loader_ishtp_cl_remove,
	.reset = loader_ishtp_cl_reset,
	.driver = {
		.pm = &loader_ishtp_pm_ops,
	},
};

static int __init ish_loader_init(void)
{
	/* Register ISHTP client device driver with ISHTP Bus */
	return ishtp_cl_driver_register(&loader_ishtp_cl_driver);
}

static void __exit ish_loader_exit(void)
{
	ishtp_cl_driver_unregister(&loader_ishtp_cl_driver);
}

late_initcall(ish_loader_init);
module_exit(ish_loader_exit);

MODULE_DESCRIPTION("ISH ISHTP Host FW Loader Client Driver");
MODULE_AUTHOR("Rushikesh S Kadam <rushikesh.s.kadam@intel.com>");

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ishtp:*");
