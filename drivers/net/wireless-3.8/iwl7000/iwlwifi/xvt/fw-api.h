/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __fw_api_h__
#define __fw_api_h__

#define IWL_XVT_DEFAULT_TX_QUEUE	1
#define IWL_XVT_CMD_QUEUE		9
#define IWL_XVT_DQA_CMD_QUEUE		0

#define IWL_XVT_DEFAULT_TX_FIFO	3
#define IWL_XVT_CMD_FIFO	7

#define IWL_XVT_TX_STA_ID_DEFAULT	0

/* command groups */
enum {
	LEGACY_GROUP = 0x0,
	LONG_GROUP = 0x1,
	PHY_OPS_GROUP = 0x4,
	DATA_PATH_GROUP = 0x5,
	CMD_GROUP_LOCATION = 0x8,
	REGULATORY_AND_NVM_GROUP = 0xc,
};

/* commands and notifications */
enum {
	XVT_ALIVE = 0x1,
	INIT_COMPLETE_NOTIF = 0x4,
	APMG_PD_SV_CMD = 0x43,

	/* ToF */
	LOCATION_GROUP_NOTIFICATION = 0x11,

	/* Tx */
	TX_CMD = 0x1C,

	/* scheduler config */
	SCD_QUEUE_CFG = 0x1d,

	/* Paging block to FW cpu2 */
	FW_PAGING_BLOCK_CMD = 0x4f,

	/* Phy */
	PHY_CONFIGURATION_CMD = 0x6a,
	CALIB_RES_NOTIF_PHY_DB = 0x6b,

	NVM_COMMIT_COMPLETE_NOTIFICATION = 0xad,

	/* NVM */
	NVM_ACCESS_CMD = 0x88,

	GET_SET_PHY_DB_CMD = 0x8f,

	/* BFE */
	REPLY_HD_PARAMS_CMD = 0xa6,

	/* Rx command */
	REPLY_RX_PHY_CMD = 0xc0,
	REPLY_RX_MPDU_CMD = 0xc1,
	REPLY_RX_DSP_EXT_INFO = 0xc4,

	DTS_MEASUREMENT_NOTIFICATION = 0xdd,

	/* Debug commands */
	REPLY_DEBUG_XVT_CMD = 0xf3,
	/* monitor data notification */
	DEBUG_LOG_MSG = 0xf7,

	REPLY_MAX = 0xff,
};

enum iwl_phy_ops_subcmd_ids {
	DTS_MEASUREMENT_NOTIF = 0XFF,
	DTS_MEASUREMENT_NOTIF_WITH_GRP = WIDE_ID(PHY_OPS_GROUP,
						 DTS_MEASUREMENT_NOTIF),
};

enum iwl_regulatory_and_nvm_subcmd_ids {
	NVM_ACCESS_COMPLETE = 0x0,
};

enum iwl_location_subcmd_ids {
	LOCATION_MCSI_NOTIFICATION = 0xFE,
	LOCATION_RANGE_RESPONSE_NOTIFICATION = 0xFF,
	LOCATION_MCSI_NOTIFICATION_WITH_GRP =
		WIDE_ID(CMD_GROUP_LOCATION, LOCATION_MCSI_NOTIFICATION),
	LOCATION_RANGE_RESPONSE_NOTIFICATION_WITH_GRP =
		WIDE_ID(CMD_GROUP_LOCATION,
			LOCATION_RANGE_RESPONSE_NOTIFICATION),
};

enum iwl_data_path_subcmd_ids {
	DQA_ENABLE_CMD = 0x0,
};

/*
 * Calibration control struct.
 * Sent as part of the phy configuration command.
 * @flow_trigger: bitmap for which calibrations to perform according to
 *		flow triggers.
 * @event_trigger: bitmap for which calibrations to perform according to
 *		event triggers.
 */
struct iwl_calib_ctrl {
	__le32 flow_trigger;
	__le32 event_trigger;
} __packed;

/*
 * Phy configuration command.
 */
struct iwl_phy_cfg_cmd {
	__le32	phy_cfg;
	struct iwl_calib_ctrl calib_control;
} __packed;

/* XVT_ALIVE 0x1 */

#define IWL_ALIVE_STATUS_ERR 0xDEAD
#define IWL_ALIVE_STATUS_OK 0xCAFE

struct xvt_alive_resp_ver2 {
	__le16 status;
	__le16 flags;
	u8 ucode_minor;
	u8 ucode_major;
	__le16 id;
	u8 api_minor;
	u8 api_major;
	u8 ver_subtype;
	u8 ver_type;
	u8 mac;
	u8 opt;
	__le16 reserved2;
	__le32 timestamp;
	__le32 error_event_table_ptr;	/* SRAM address for error log */
	__le32 log_event_table_ptr;	/* SRAM address for LMAC event log */
	__le32 cpu_register_ptr;
	__le32 dbgm_config_ptr;
	__le32 alive_counter_ptr;
	__le32 scd_base_ptr;		/* SRAM address for SCD */
	__le32 st_fwrd_addr;		/* pointer to Store and forward */
	__le32 st_fwrd_size;
	u8 umac_minor;			/* UMAC version: minor */
	u8 umac_major;			/* UMAC version: major */
	__le16 umac_id;			/* UMAC version: id */
	__le32 error_info_addr;		/* SRAM address for UMAC error log */
	__le32 dbg_print_buff_addr;
} __packed; /* ALIVE_RES_API_S_VER_2 */

struct iwl_lmac_alive {
	__le32 ucode_minor;
	__le32 ucode_major;
	u8 ver_subtype;
	u8 ver_type;
	u8 mac;
	u8 opt;
	__le32 timestamp;
	__le32 error_event_table_ptr;	/* SRAM address for error log */
	__le32 log_event_table_ptr;	/* SRAM address for LMAC event log */
	__le32 cpu_register_ptr;
	__le32 dbgm_config_ptr;
	__le32 alive_counter_ptr;
	__le32 scd_base_ptr;		/* SRAM address for SCD */
	__le32 st_fwrd_addr;		/* pointer to Store and forward */
	__le32 st_fwrd_size;
} __packed; /* UCODE_ALIVE_DATA_API_VER_3 */

struct iwl_umac_alive {
	__le32 umac_minor;		/* UMAC version: minor */
	__le32 umac_major;		/* UMAC version: major */
	__le32 error_info_addr;		/* SRAM address for UMAC error log */
	__le32 dbg_print_buff_addr;
} __packed; /* ALIVE_RES_API_S_VER_3 */

struct xvt_alive_resp_ver3 {
	__le16 status;
	__le16 flags;
	struct iwl_lmac_alive lmac_data;
	struct iwl_umac_alive umac_data;
} __packed; /* ALIVE_RES_API_S_VER_3 */

struct xvt_alive_resp_ver4 {
	__le16 status;
	__le16 flags;
	struct iwl_lmac_alive lmac_data[2];
	struct iwl_umac_alive umac_data;
} __packed; /* UCODE_ALIVE_NTFY_API_S_VER_4 */

/**
 * struct iwl_nvm_access_complete_cmd - NVM_ACCESS commands are completed
 */
struct iwl_nvm_access_complete_cmd {
	__le32 reserved;
} __packed; /* NVM_ACCESS_COMPLETE_CMD_API_S_VER_1 */

#define TX_CMD_LIFE_TIME_INFINITE	0xFFFFFFFF

#define TX_CMD_FLG_ACK_MSK cpu_to_le32(1 << 3)
#define TX_CMD_OFFLD_PAD	BIT(13) /* TX_CMD_OFLD_ASSIST_PAD */
#define TX_CMD_FLAGS_CMD_RATE	BIT(0) /* TX_CMD_FLG_RATE_FROM_CMD */

struct iwl_tx_cmd {
	__le16 len;
	__le16 next_frame_len;
	__le32 tx_flags;
	/* DRAM_SCRATCH_API_U_VER_1 */
	u8 try_cnt;
	u8 btkill_cnt;
	__le16 reserved;
	__le32 rate_n_flags;
	u8 sta_id;
	u8 sec_ctl;
	u8 initial_rate_index;
	u8 reserved2;
	u8 key[16];
	__le16 next_frame_flags;
	__le16 reserved3;
	__le32 life_time;
	__le32 dram_lsb_ptr;
	u8 dram_msb_ptr;
	u8 rts_retry_limit;
	u8 data_retry_limit;
	u8 tid_tspec;
	__le16 pm_frame_timeout;
	__le16 driver_txop;
	u8 payload[0];
	struct ieee80211_hdr hdr[0];
} __packed; /* TX_CMD_API_S_VER_3 */

struct iwl_dram_sec_info {
	__le32 pn_low;
	__le16 pn_high;
	__le16 aux_info;
} __packed; /* DRAM_SEC_INFO_API_S_VER_1 */

/**
 * struct iwl_tx_cmd_gen2 - TX command struct to FW for a000 devices
 * ( TX_CMD = 0x1c )
 * @len: in bytes of the payload, see below for details
 * @offload_assist: TX offload configuration
 * @flags: combination of &iwl_tx_cmd_flags
 * @dram_info: FW internal DRAM storage
 * @rate_n_flags: rate for *all* Tx attempts, if TX_CMD_FLG_STA_RATE_MSK is
 *	cleared. Combination of RATE_MCS_*
 * @hdr: 802.11 header
 */
struct iwl_tx_cmd_gen2 {
	__le16 len;
	__le16 offload_assist;
	__le32 flags;
	struct iwl_dram_sec_info dram_info;
	__le32 rate_n_flags;
	struct ieee80211_hdr hdr[0];
} __packed; /* TX_CMD_API_S_VER_7 */

struct agg_tx_status {
	__le16 status;
	__le16 sequence;
} __packed;

/**
 * struct iwl_xvt_tx_resp - notifies that fw is TXing a packet
 * ( REPLY_TX = 0x1c )
 * @frame_count: 1 no aggregation, >1 aggregation
 * @bt_kill_count: num of times blocked by bluetooth (unused for agg)
 * @failure_rts: num of failures due to unsuccessful RTS
 * @failure_frame: num failures due to no ACK (unused for agg)
 * @initial_rate: for non-agg: rate of the successful Tx. For agg: rate of the
 *	Tx of all the batch. RATE_MCS_*
 * @wireless_media_time: for non-agg: RTS + CTS + frame tx attempts time + ACK.
 *	for agg: RTS + CTS + aggregation tx time + block-ack time.
 *	in usec.
 * @pa_status: tx power info
 * @pa_integ_res_a: tx power info
 * @pa_integ_res_b: tx power info
 * @pa_integ_res_c: tx power info
 * @measurement_req_id: tx power info
 * @reduced_tpc: transmit power reduction used
 * @tfd_info: TFD information set by the FH
 * @seq_ctl: sequence control from the Tx cmd
 * @byte_cnt: byte count from the Tx cmd
 * @tlc_info: TLC rate info
 * @ra_tid: bits [3:0] = ra, bits [7:4] = tid
 * @frame_ctrl: frame control
 * @tx_queue: TX queue for this response
 * @status: for non-agg:  frame status TX_STATUS_*
 *	for agg: status of 1st frame, AGG_TX_STATE_*; other frame status fields
 *	follow this one, up to frame_count.
 *	For version 6 TX response isn't received for aggregation at all.
 */
struct iwl_xvt_tx_resp {
	u8 frame_count;
	u8 bt_kill_count;
	u8 failure_rts;
	u8 failure_frame;
	__le32 initial_rate;
	__le16 wireless_media_time;

	u8 pa_status;
	u8 pa_integ_res_a[3];
	u8 pa_integ_res_b[3];
	u8 pa_integ_res_c[3];
	__le16 measurement_req_id;
	u8 reduced_tpc;
	u8 reserved;

	__le32 tfd_info;
	__le16 seq_ctl;
	__le16 byte_cnt;
	u8 tlc_info;
	u8 ra_tid;
	__le16 frame_ctrl;
	union {
		struct {
			struct agg_tx_status status;
		} v3;/* TX_RSP_API_S_VER_3 */
		struct {
			__le16 tx_queue;
			__le16 reserved2;
			struct agg_tx_status status;
		} v6;
	};
} __packed; /* TX_RSP_API_S_VER_6 */

enum {
	XVT_DBG_GET_SVDROP_VER_OP = 0x01,
};

struct xvt_debug_cmd {
	__le32 opcode;
	__le32 dw_num;
}; /* DEBUG_XVT_CMD_API_S_VER_1 */

struct xvt_debug_res {
	__le32 dw_num;
	__le32 data[0];
}; /* DEBUG_XVT_RES_API_S_VER_1 */

/* Section types for NVM_ACCESS_CMD */
enum {
	NVM_SECTION_TYPE_HW = 0,
	NVM_SECTION_TYPE_SW,
	NVM_SECTION_TYPE_PAPD,
	NVM_SECTION_TYPE_BT,
	NVM_SECTION_TYPE_CALIBRATION,
	NVM_SECTION_TYPE_PRODUCTION,
	NVM_SECTION_TYPE_POST_FCS_CALIB,
	NVM_SECTION_TYPE_MAC_OVERRIDE = 11,
	NVM_NUM_OF_SECTIONS,
};

#define NUM_OF_FW_PAGING_BLOCKS	33 /* 32 for data and 1 block for CSS */

/*
 * struct iwl_fw_paging_cmd - paging layout
 *
 * (FW_PAGING_BLOCK_CMD = 0x4f)
 *
 * Send to FW the paging layout in the driver.
 *
 * @flags: various flags for the command
 * @block_size: the block size in powers of 2
 * @block_num: number of blocks specified in the command.
 * @device_phy_addr: virtual addresses from device side
*/
struct iwl_fw_paging_cmd {
	__le32 flags;
	__le32 block_size;
	__le32 block_num;
	__le32 device_phy_addr[NUM_OF_FW_PAGING_BLOCKS];
} __packed; /* FW_PAGING_BLOCK_CMD_API_S_VER_1 */

/**
 * struct iwl_nvm_access_cmd_ver2 - Request the device to send an NVM section
 * @op_code: 0 - read, 1 - write
 * @target: NVM_ACCESS_TARGET_*
 * @type: NVM_SECTION_TYPE_*
 * @offset: offset in bytes into the section
 * @length: in bytes, to read/write
 * @data: if write operation, the data to write. On read its empty
 */
struct iwl_nvm_access_cmd {
	u8 op_code;
	u8 target;
	__le16 type;
	__le16 offset;
	__le16 length;
	u8 data[];
} __packed; /* NVM_ACCESS_CMD_API_S_VER_2 */

/**
 * struct iwl_nvm_access_resp_ver2 - response to NVM_ACCESS_CMD
 * @offset: offset in bytes into the section
 * @length: in bytes, either how much was written or read
 * @type: NVM_SECTION_TYPE_*
 * @status: 0 for success, fail otherwise
 * @data: if read operation, the data returned. Empty on write.
 */
struct iwl_nvm_access_resp {
	__le16 offset;
	__le16 length;
	__le16 type;
	__le16 status;
	u8 data[];
} __packed; /* NVM_ACCESS_CMD_RESP_API_S_VER_2 */

/**
 * struct iwl_dqa_enable_cmd
 * @cmd_queue: the TXQ number of the command queue
 */
struct iwl_dqa_enable_cmd {
	__le32 cmd_queue;
} __packed; /* DQA_CONTROL_CMD_API_S_VER_1 */

/* Available options for the SCD_QUEUE_CFG HCMD */
enum iwl_scd_cfg_actions {
	SCD_CFG_DISABLE_QUEUE		= 0x0,
	SCD_CFG_ENABLE_QUEUE		= 0x1,
	SCD_CFG_UPDATE_QUEUE_TID	= 0x2,
};

/**
 * struct iwl_scd_txq_cfg_cmd - New txq hw scheduler config command
 * @token:
 * @sta_id: station id
 * @tid: traffic id
 * @scd_queue: scheduler queue to config
 * @action: 1 queue enable, 0 queue disable, 2 change txq's tid owner
 *	Value is one of %iwl_scd_cfg_actions options
 * @aggregate: 1 aggregated queue, 0 otherwise
 * @tx_fifo: &enum iwl_mvm_tx_fifo
 * @window: BA window size
 * @ssn: SSN for the BA agreement
 */
struct iwl_scd_txq_cfg_cmd {
	u8 token;
	u8 sta_id;
	u8 tid;
	u8 scd_queue;
	u8 action;
	u8 aggregate;
	u8 tx_fifo;
	u8 window;
	__le16 ssn;
	__le16 reserved;
} __packed; /* SCD_QUEUE_CFG_CMD_API_S_VER_1 */
#endif /* __fw_api_h__ */
