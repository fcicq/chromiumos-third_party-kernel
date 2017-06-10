/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2015-2017 Intel Deutschland GmbH
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
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2015-2017 Intel Deutschland GmbH
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

#ifndef __iwl_fw_api_nan_h__
#define __iwl_fw_api_nan_h__

/* TODO: read it from tlv */
#define NAN_MAX_SUPPORTED_DE_ENTRIES 10

/**
 * enum iwl_nan_subcmd_ids - Neighbor Awareness Networking (NaN) commands IDS
 */
enum iwl_nan_subcmd_ids {
	/**
	 * @NAN_CONFIG_CMD:
	 * &struct iwl_nan_cfg_cmd_v2 or &struct iwl_nan_cfg_cmd
	 */
	NAN_CONFIG_CMD = 0,

	/**
	 * @NAN_DISCOVERY_FUNC_CMD:
	 * &struct iwl_nan_add_func_cmd or &struct iwl_nan_add_func_cmd_v2
	 */
	NAN_DISCOVERY_FUNC_CMD = 0x1,

	/**
	 * @NAN_FAW_CONFIG_CMD:
	 * &struct iwl_nan_faw_config
	 */
	NAN_FAW_CONFIG_CMD = 0x2,

	/**
	 * @NAN_DISCOVERY_EVENT_NOTIF:
	 * &struct iwl_nan_disc_evt_notify
	 */
	NAN_DISCOVERY_EVENT_NOTIF = 0xFD,

	/**
	 * @NAN_DISCOVERY_TERMINATE_NOTIF:
	 * &struct iwl_nan_de_term
	 */
	NAN_DISCOVERY_TERMINATE_NOTIF = 0xFE,

	/**
	 * @NAN_FAW_START_NOTIF:
	 */
	NAN_FAW_START_NOTIF = 0xFF,
};

/**
 * struct iwl_fw_chan_avail - Defines per op. class channel availability
 *
 * @op_class: operating class
 * @chan_bitmap: channel bitmap
 */
struct iwl_fw_chan_avail {
	u8 op_class;
	__le16 chan_bitmap;
} __packed;

/**
 * struct iwl_nan_umac_cfg - NAN umac config parameters
 *
 * @action: one of the FW_CTXT_ACTION_*
 * @tsf_id: tsf id to use in beacons
 * @sta_id: STA used for NAN operations. Currently it is AUX STA.
 * @node_addr: our address
 * @reserved1: reserved
 * @master_pref: master preference value
 * @master_rand: random factor to override fw's decision (DEBUG)
 * @cluster_id: cluster id, if 0 the fw will choose one for us.
 * @dual_band: enables dual band operation.
 * @beacon_template_id: beacon template id for NAN
 */
struct iwl_nan_umac_cfg {
	__le32 action;
	__le32 tsf_id;
	__le32 sta_id;
	u8 node_addr[6];
	__le16 reserved1;
	u8 master_pref;
	u8 master_rand;
	__le16 cluster_id;
	__le32 dual_band;
	__le32 beacon_template_id;
} __packed; /* _NAN_UMAC_CONFIG_CMD_API_S_VER_1 */

/**
 * struct iwl_nan_testbed_cfg - NAN testbed specific config parameters
 *
 * @chan24: override default 2.4GHz channel (DEBUG)
 * @chan52: override default 5.2GHz channel (DEBUG)
 * @hop_count: fake hop count (DEBUG)
 * @op_bands: band bit mask (DEBUG)
 * @warmup_timer: warmup_timer in us (DEBUG)
 * @custom_tsf: fake tsf value (DEBUG)
 * @action_delay: usecs to delay SDFs (DEBUG)
 */
struct iwl_nan_testbed_cfg {
	u8 chan24;
	u8 chan52;
	u8 hop_count;
	u8 op_bands;
	__le32 warmup_timer;
	__le64 custom_tsf;
	__le32 action_delay;
} __packed; /* NAN_TEST_BED_SPECIFIC_CONFIG_S_VER_1 */

/*
 * struct iwl_nan_nan2_cfg - NAN2 specific configuration
 *
 * @cdw: committed DW interval as defined in NAN2 spec (NAN2)
 * @op_mode: operation mode as defined in NAN2 spec (NAN2)
 * @pot_avail_len: length of pot_avail array (NAN2)
 * @pot_avail: potential availability per op. class (NAN2)
 */
struct iwl_nan_nan2_cfg {
	__le16 cdw;
	u8 op_mode;
	u8 pot_avail_len;
	struct iwl_fw_chan_avail pot_avail[20];
} __packed; /* NAN_CONFIG_CMD_API_S_VER_1 */

/**
 * struct iwl_nan_cfg_cmd - NAN configuration command
 *
 * This command starts/stops/modifies NAN sync engine.
 *
 * @umac_cfg: umac specific configuration
 * @tb_cfg: testbed specific configuration
 * @nan2_cfg: nan2 specific configuration
 */
struct iwl_nan_cfg_cmd {
	struct iwl_nan_umac_cfg umac_cfg;
	struct iwl_nan_testbed_cfg tb_cfg;
	/* NAN 2 specific configuration */
	struct iwl_nan_nan2_cfg nan2_cfg;
} __packed; /* NAN_CONFIG_CMD_API_S_VER_1 */

/**
 * struct iwl_nan_cfg_cmd_v2 - NAN configuration command, version 2
 *
 * This command starts/stops/modifies NAN sync engine.
 *
 * @umac_cfg: umac specific configuration
 * @tb_cfg: testbed specific configuration
 * @unavailable_slots: Force this amount of slots to be unavailable in potential
 *	map
 * @nan2_cfg: nan2 specific configuration
 */
struct iwl_nan_cfg_cmd_v2 {
	struct iwl_nan_umac_cfg umac_cfg;
	struct iwl_nan_testbed_cfg tb_cfg;
	__le32 unavailable_slots;
	/* NAN 2 specific configuration */
	struct iwl_nan_nan2_cfg nan2_cfg;
} __packed; /* NAN_CONFIG_CMD_API_S_VER_2 */

/* NAN DE function type */
enum iwl_fw_nan_func_type {
	NAN_DE_FUNC_PUBLISH = 0,
	NAN_DE_FUNC_SUBSCRIBE = 1,
	NAN_DE_FUNC_FOLLOW_UP = 2,

	/* keep last */
	NAN_DE_FUNC_NOT_VALID,
};

/* NAN DE function flags */
enum iwl_fw_nan_func_flags {
	NAN_DE_FUNC_FLAG_UNSOLICITED_OR_ACTIVE = BIT(0),
	NAN_DE_FUNC_FLAG_SOLICITED = BIT(1),
	NAN_DE_FUNC_FLAG_UNICAST = BIT(2),
	NAN_DE_FUNC_FLAG_CLOSE_RANGE = BIT(3),
	NAN_DE_FUNC_FLAG_FAW_PRESENT = BIT(4),
	NAN_DE_FUNC_FLAG_FAW_TYPE = BIT(5),
	NAN_DE_FUNC_FLAG_FAW_NOTIFY = BIT(6),
	NAN_DE_FUNC_FLAG_RAISE_EVENTS = BIT(7),
};

/**
 * struct iwl_nan_add_func_common - NAN add/remove function, common part
 *
 * @action: one of the FW_CTXT_ACTION_ADD/REMOVE
 * @instance_id: instance id of the DE function to remove
 * @type: enum %iwl_fw_nan_func_type
 * @service_id: service id
 * @flags: a combination of &enum iwl_fw_nan_func_flags
 * @flw_up_id: follow up id
 * @flw_up_req_id: follow up requestor id
 * @flw_up_addr: follow up destination address
 * @reserved1: reserved
 * @ttl: ttl in DW's or 0 for infinite
 * @faw_ci: &struct iwl_fw_channel_info for further availability
 * @faw_attrtype: further availability bitmap
 * @serv_info_len: length of service specific information
 * @srf_len: length of the srf
 * @rx_filter_len: length of rx filter
 * @tx_filter_len: length of tx filter
 * @dw_interval: awake dw interval
 */
struct iwl_nan_add_func_common {
	__le32 action;
	u8 instance_id;
	u8 type;
	u8 service_id[6];
	__le16 flags;
	u8 flw_up_id;
	u8 flw_up_req_id;
	u8 flw_up_addr[6];
	__le16 reserved1;
	__le32 ttl;
	struct iwl_fw_channel_info faw_ci;
	u8 faw_attrtype;
	u8 serv_info_len;
	u8 srf_len;
	u8 rx_filter_len;
	u8 tx_filter_len;
	u8 dw_interval;
} __packed; /* NAN_DISCO_FUNC_FIXED_CMD_API_S_VER_1 */

/**
 * struct iwl_nan_add_func_cmd_v2 - NAN add/remove function command, version 2
 *
 * @cmn: nan add function common part
 * @cipher_capa: Cipher suite information capabilities
 * @cipher_suite_id: Bitmap of the list of cipher suite IDs
 * @security_ctx_len: length of tx security context attributes
 * @sdea_ctrl: SDEA control field
 * @data: dw aligned fields: service_info, srf, rxFilter, txFilter,
 *	security_ctx
 */
struct iwl_nan_add_func_cmd_v2 {
	struct iwl_nan_add_func_common cmn;
	u8 cipher_capa;
	u8 cipher_suite_id;
	__le16 security_ctx_len;
	__le16 sdea_ctrl;
	u8 data[0];
} __packed; /* NAN_DISCO_FUNC_FIXED_CMD_API_S_VER_2 */

/**
 * struct iwl_nan_add_func_cmd - NAN add/remove function command
 *
 * @cmn: nan add function common part
 * @reserved: reserved
 * @data: dw aligned fields -service_info, srf, rxFilter, txFilter
 */
struct iwl_nan_add_func_cmd {
	struct iwl_nan_add_func_common cmn;
	u8 reserved[2];
	u8 data[0];
} __packed; /* NAN_DISCO_FUNC_FIXED_CMD_API_S_VER_1 */

enum iwl_nan_add_func_resp_status {
	NAN_DE_FUNC_STATUS_SUCCESSFUL,
	NAN_DE_FUNC_STATUS_INSUFFICIENT_ENTRIES,
	NAN_DE_FUNC_STATUS_INSUFFICIENT_MEMORY,
	NAN_DE_FUNC_STATUS_INVALID_INSTANCE,
	NAN_DE_FUNC_STATUS_UNSPECIFIED,
};

/**
 * struct iwl_nan_add_func_res - Add NAN function response
 *
 * @instance_id: assigned instance_id (if added)
 * @status: status of the command (&enum iwl_nan_add_func_resp_status)
 * @reserved: reserved
 */
struct iwl_nan_add_func_res {
	u8 instance_id;
	u8 status;
	__le16 reserved;
} __packed; /* NAN_DISCO_FUNC_CMD_API_S_VER_1 */

/**
 * struct iwl_nan_disc_evt_notify - NaN discovery event
 *
 * @peer_mac_addr: peer address
 * @reserved1: reserved
 * @type: Type of matching function
 * @instance_id: local matching instance id
 * @peer_instance: peer matching instance id
 * @service_info_len: service specific information length
 * @attrs_len: additional peer attributes length
 * @buf: service specific information followed by attributes
 */
struct iwl_nan_disc_evt_notify {
	u8 peer_mac_addr[6];
	__le16 reserved1;
	u8 type;
	u8 instance_id;
	u8 peer_instance;
	u8 service_info_len;
	__le32 attrs_len;
	u8 buf[0];
} __packed; /* NAN_DISCO_EVENT_NTFY_API_S_VER_1 */

/* NAN function termination reasons */
enum iwl_fw_nan_de_term_reason {
	NAN_DE_TERM_FAILURE = 0,
	NAN_DE_TERM_TTL_REACHED,
	NAN_DE_TERM_USER_REQUEST,
};

/**
 * struct iwl_nan_de_term - NAN function termination event
 *
 * @type: type of terminated function (&enum iwl_fw_nan_func_type)
 * @instance_id: instance id
 * @reason: termination reason (&enum iwl_fw_nan_de_term_reason)
 * @reserved1: reserved
 */
struct iwl_nan_de_term {
	u8 type;
	u8 instance_id;
	u8 reason;
	u8 reserved1;
} __packed; /* NAN_DISCO_TERM_NTFY_API_S_VER_1 */

enum iwl_fw_post_nan_type {
	NAN_POST_NAN_ATTR_WLAN = 0,
	NAN_POST_NAN_ATTR_P2P,
	NAN_POST_NAN_ATTR_IBSS,
	NAN_POST_NAN_ATTR_MESH,
	NAN_POST_NAN_ATTR_FURTHER_NAN,
};

enum iwl_fw_config_flags {
	NAN_FAW_FLAG_NOTIFY_HOST = BIT(0),
};

/**
 * struct iwl_nan_faw_config - NAN further availability configuration command
 *
 * @id_n_color: id and color of the mac used for further availability
 * @faw_ci: channel to be available on
 * @type: type of post NAN availability (enum %iwl_fw_post_nan_type)
 * @slots: number of 16TU slots to be available on (should be < 32)
 * @flags: NAN_FAW_FLAG_*
 * @op_class: operating class which corresponds to faw_ci
 */
struct iwl_nan_faw_config {
	__le32 id_n_color;
	struct iwl_fw_channel_info faw_ci;
	u8 type;
	u8 slots;
	u8 flags;
	u8 op_class;
} __packed; /* _NAN_DISCO_FAW_CMD_API_S_VER_1 */

#endif /* __iwl_fw_api_nan_h__ */
