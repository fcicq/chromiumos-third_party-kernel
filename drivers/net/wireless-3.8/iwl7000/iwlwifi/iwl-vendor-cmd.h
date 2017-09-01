/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
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
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 - 2017 Intel Deutschland GmbH
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
#ifndef __VENDOR_CMD_H__
#define __VENDOR_CMD_H__

#define INTEL_OUI	0x001735

/**
 * enum iwl_mvm_vendor_cmd - supported vendor commands
 * @IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY: set low-latency mode for the given
 *	virtual interface
 * @IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY: query low-latency mode
 * @IWL_MVM_VENDOR_CMD_TCM_EVENT: TCM event
 * @IWL_MVM_VENDOR_CMD_LTE_STATE: inform the LTE modem state
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_CONFIG_INFO: configure LTE-Coex static
 *	parameters
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_DYNAMIC_INFO: configure LTE dynamic parameters
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_SPS_INFO: configure semi oersistent info
 * @IWL_MVM_VENDOR_CMD_LTE_COEX_WIFI_RPRTD_CHAN: Wifi reported channel as
 *	calculated by the coex-manager
 * @IWL_MVM_VENDOR_CMD_SET_COUNTRY: set a new mcc regulatory information
 * @IWL_MVM_VENDOR_CMD_PROXY_FRAME_FILTERING: filter GTK, gratuitous
 *	ARP & unsolicited NA
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_ADD: add a peer to the TDLS peer cache
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_DEL: delete a peer from the TDLS peer
 *	cache
 * @IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_QUERY: query traffic statistics for a
 *	peer in the TDLS cache
 * @IWL_MVM_VENDOR_CMD_SET_NIC_TXPOWER_LIMIT: set the NIC's (SAR) TX power limit
 * @IWL_MVM_VENDOR_CMD_OPPPS_WA: wa to pass Sigma test - applicable code is
 *	claused under CPTCFG_IWLMVM_P2P_OPPPS_TEST_WA
 * @IWL_MVM_VENDOR_CMD_GSCAN_GET_CAPABILITIES: get driver gscan capabilities as
 *	specified in %IWL_MVM_VENDOR_ATTR_GSCAN_*
 * @IWL_MVM_VENDOR_CMD_GSCAN_START: set gscan parameters and start gscan
 * @IWL_MVM_VENDOR_CMD_GSCAN_STOP: stop a previously started gscan
 * @IWL_MVM_VENDOR_CMD_GSCAN_RESULTS_EVENT: event that reports scan results
 *	from gscan. This event is sent when the scan results buffer has reached
 *	the report threshold, or when scanning a bucket with report mode
 *	%IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE was completed.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SET_BSSID_HOTLIST: set a list of AP's to track
 *	changes in their RSSI and report scan results history when RSSI goes
 *	above/below threshold. Sending this command with an empty list of AP's
 *	will cancel previous set_bssid_hotlist request.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SET_SIGNIFICANT_CHANGE: set a list of APs to track
 *	significant changes in their RSSI. Sending this command with an empty
 *	list of AP's will cancel previous set_significant_change request.
 * @IWL_MVM_VENDOR_CMD_GSCAN_HOTLIST_CHANGE_EVENT: event that indicates that an
 *	AP from the BSSID hotlist was lost or found.
 * @IWL_MVM_VENDOR_CMD_GSCAN_SIGNIFICANT_CHANGE_EVENT: event that indicates a
 *	significant change in the RSSI level of beacons received from a certain
 *	AP.
 * @IWL_MVM_VENDOR_CMD_RXFILTER: Set/clear rx filter.
 * @IWL_MVM_VENDOR_CMD_GSCAN_BEACON_EVENT: event that reports a
 *	beacon/probe response was received, and contains information from the
 *	beacon/probe response. This event is sent for buckets with report mode
 *	set to %IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_COMPLETE_RESULTS.
 * @IWL_MVM_VENDOR_CMD_DBG_COLLECT: collect debug data
 * @IWL_MVM_VENDOR_CMD_NAN_FAW_CONF: Configure post NAN further availability.
 * @IWL_MVM_VENDOR_CMD_SET_SAR_PROFILE: set the NIC's tx power limits
 *	according to the specified tx power profiles. In this command
 *	%IWL_MVM_VENDOR_ATTR_SAR_CHAIN_A_PROFILE and
 *	%IWL_MVM_VENDOR_ATTR_SAR_CHAIN_B_PROFILE must be passed.
 * @IWL_MVM_VENDOR_CMD_GET_SAR_PROFILE_INFO: get sar profile information.
 *	This command provides the user with the following information:
 *	Number of enabled SAR profiles, current used SAR profile per chain.
 * @IWL_MVM_VENDOR_CMD_NEIGHBOR_REPORT_REQUEST: Send a neighbor report request
 *	to the AP we are currently connected to. The request parameters are
 *	specified with %IWL_MVM_VENDOR_ATTR_NR_*.
 * @IWL_MVM_VENDOR_CMD_NEIGHBOR_REPORT_RESPONSE: An event that reports a list of
 *	neighbor APs received in a neighbor report response frame. The report is
 *	a nested list of &enum iwl_mvm_vendor_neighbor_report.
 * @IWL_MVM_VENDOR_CMD_GET_SAR_GEO_PROFILE: get sar geographic profile
 *	information. This command provides the user with the following
 *	information: Per band tx power offset for chain A and chain B as well as
 *	maximum allowed tx power on this band.
 */

enum iwl_mvm_vendor_cmd {
	IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_TCM_EVENT,
	IWL_MVM_VENDOR_CMD_LTE_STATE,
	IWL_MVM_VENDOR_CMD_LTE_COEX_CONFIG_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_DYNAMIC_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_SPS_INFO,
	IWL_MVM_VENDOR_CMD_LTE_COEX_WIFI_RPRTD_CHAN,
	IWL_MVM_VENDOR_CMD_SET_COUNTRY,
	IWL_MVM_VENDOR_CMD_PROXY_FRAME_FILTERING,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_ADD,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_DEL,
	IWL_MVM_VENDOR_CMD_TDLS_PEER_CACHE_QUERY,
	IWL_MVM_VENDOR_CMD_SET_NIC_TXPOWER_LIMIT,
	IWL_MVM_VENDOR_CMD_OPPPS_WA,
	IWL_MVM_VENDOR_CMD_GSCAN_GET_CAPABILITIES,
	IWL_MVM_VENDOR_CMD_GSCAN_START,
	IWL_MVM_VENDOR_CMD_GSCAN_STOP,
	IWL_MVM_VENDOR_CMD_GSCAN_RESULTS_EVENT,
	IWL_MVM_VENDOR_CMD_GSCAN_SET_BSSID_HOTLIST,
	IWL_MVM_VENDOR_CMD_GSCAN_SET_SIGNIFICANT_CHANGE,
	IWL_MVM_VENDOR_CMD_GSCAN_HOTLIST_CHANGE_EVENT,
	IWL_MVM_VENDOR_CMD_GSCAN_SIGNIFICANT_CHANGE_EVENT,
	IWL_MVM_VENDOR_CMD_RXFILTER,
	IWL_MVM_VENDOR_CMD_GSCAN_BEACON_EVENT,
	IWL_MVM_VENDOR_CMD_DBG_COLLECT,
	IWL_MVM_VENDOR_CMD_NAN_FAW_CONF,
	IWL_MVM_VENDOR_CMD_SET_SAR_PROFILE,
	IWL_MVM_VENDOR_CMD_GET_SAR_PROFILE_INFO,
	IWL_MVM_VENDOR_CMD_NEIGHBOR_REPORT_REQUEST,
	IWL_MVM_VENDOR_CMD_NEIGHBOR_REPORT_RESPONSE,
	IWL_MVM_VENDOR_CMD_GET_SAR_GEO_PROFILE,
};

/**
 * enum iwl_mvm_vendor_load - traffic load identifiers
 * @IWL_MVM_VENDOR_LOAD_LOW: low load: less than 10% airtime usage
 * @IWL_MVM_VENDOR_LOAD_MEDIUM: medium load: 10% or more, but less than 50%
 * @IWL_MVM_VENDOR_LOAD_HIGH: high load: 50% or more
 *
 * Traffic load is calculated based on the percentage of airtime used
 * (TX airtime is accounted as RTS+CTS+PPDU+ACK/BlockACK, RX airtime
 * is just the PPDU's time)
 */
enum iwl_mvm_vendor_load {
	IWL_MVM_VENDOR_LOAD_LOW,
	IWL_MVM_VENDOR_LOAD_MEDIUM,
	IWL_MVM_VENDOR_LOAD_HIGH,
};

/**
 * enum iwl_mvm_vendor_gscan_report_mode - gscan scan results report modes
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_FULL: report that scan results are
 *	available only when the scan results buffer reaches the report
 *	threshold. The report threshold is set for each bucket.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_EACH_SCAN: report that scan results are
 *	available when scanning of this bucket is complete.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_FULL_RESULTS: forward scan results
 *	(beacons/probe responses) in real time to userspace.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_HISTORY_RESERVED: reserved.
 * @IWL_MVM_VENDOR_GSCAN_REPORT_NO_BATCH: do not fill scan history buffer.
 * @NUM_IWL_MVM_VENDOR_GSCAN_REPORT: number of report mode attributes.
 *
 * Note that these must match the firmware API.
 */
enum iwl_mvm_vendor_gscan_report_mode {
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_FULL,
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_EACH_SCAN,
	IWL_MVM_VENDOR_GSCAN_REPORT_BUFFER_FULL_RESULTS,
	IWL_MVM_VENDOR_GSCAN_REPORT_HISTORY_RESERVED,
	IWL_MVM_VENDOR_GSCAN_REPORT_NO_BATCH,
	NUM_IWL_MVM_VENDOR_GSCAN_REPORT,
};

/**
 * enum iwl_mvm_vendor_gscan_channel_spec - gscan channel specification
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL: channel number
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME: u16 attribute specifying dwell
 *	time on this channel.
 * @IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE: flag attribute. If set, passive
 *	scan should be performed on this channel.
 * @NUM_IWL_MVM_VENDOR_CHANNEL_SPEC: number of channel spec attributes.
 * @MAX_IWL_MVM_VENDOR_CHANNEL_SPEC: highest channel spec attribute number.
 */
enum iwl_mvm_vendor_gscan_channel_spec {
	IWL_MVM_VENDOR_CHANNEL_SPEC_INVALID,
	IWL_MVM_VENDOR_CHANNEL_SPEC_CHANNEL,
	IWL_MVM_VENDOR_CHANNEL_SPEC_DWELL_TIME,
	IWL_MVM_VENDOR_CHANNEL_SPEC_PASSIVE,
	NUM_IWL_MVM_VENDOR_CHANNEL_SPEC,
	MAX_IWL_MVM_VENDOR_CHANNEL_SPEC =
		NUM_IWL_MVM_VENDOR_CHANNEL_SPEC - 1,
};

/**
 * enum iwl_mvm_vendor_gscan_bucket_spec - gscan bucket specification
 * @IWL_MVM_VENDOR_BUCKET_SPEC_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_BUCKET_SPEC_INDEX: bucket index
 * @IWL_MVM_VENDOR_BUCKET_SPEC_BAND: band to scan as specified in
 *	&enum iwl_gscan_band. When not set, the channel list is used.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD: interval between this bucket scans,
 *	in msecs.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE: when to report scan results.
 *	Available modes are specified in &enum iwl_mvm_vendor_report_mode.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS: array of channels to scan for this
 *	bucket. Each channel is specified with a nested attribute of
 *	%IWL_MVM_VENDOR_CHANNEL_SPEC. This channel list is used when
 *	%IWL_MVM_VENDOR_BUCKET_SPEC_BAND is set to
 *	%IWL_MVM_VENDOR_BAND_UNSPECIFIED.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_MAX_PERIOD: maximum scan interval. If it's
 *	non zero or different than period, then this bucket is an exponential
 *	back off bucket and the scan period will grow exponentially.
 * @IWL_MVM_VENDOR_BUCKET_SPEC_EXPONENT: for exponential back off bucket,
 *	scan period calculation should be done according to the following:
 *	new_period = old_period * exponent
 * @IWL_MVM_VENDOR_BUCKET_SPEC_STEP_CNT: for exponential back off bucket:
 *	number of scans to perform at a given period and until the exponent
 *	is applied.
 * @NUM_IWL_MVM_VENDOR_BUCKET_SPEC: number of bucket spec attributes.
 * @MAX_IWL_MVM_VENDOR_BUCKET_SPEC: highest bucket spec attribute number.
 */
enum iwl_mvm_vendor_gscan_bucket_spec {
	IWL_MVM_VENDOR_BUCKET_SPEC_INVALID,
	IWL_MVM_VENDOR_BUCKET_SPEC_INDEX,
	IWL_MVM_VENDOR_BUCKET_SPEC_BAND,
	IWL_MVM_VENDOR_BUCKET_SPEC_PERIOD,
	IWL_MVM_VENDOR_BUCKET_SPEC_REPORT_MODE,
	IWL_MVM_VENDOR_BUCKET_SPEC_CHANNELS,
	IWL_MVM_VENDOR_BUCKET_SPEC_MAX_PERIOD,
	IWL_MVM_VENDOR_BUCKET_SPEC_EXPONENT,
	IWL_MVM_VENDOR_BUCKET_SPEC_STEP_CNT,
	NUM_IWL_MVM_VENDOR_BUCKET_SPEC,
	MAX_IWL_MVM_VENDOR_BUCKET_SPEC =
		NUM_IWL_MVM_VENDOR_BUCKET_SPEC - 1,
};

/**
 * enum iwl_mvm_vendor_results_event_type - scan results available event type
 * @IWL_MVM_VENDOR_RESULTS_NOTIF_BUFFER_FULL: scan results available was
 *	reported because scan results buffer has reached the report threshold.
 * @IWL_MVM_VENDOR_RESULTS_NOTIF_BUCKET_END: scan results available was reported
 *	because scan of a bucket was completed.
 * @NUM_IWL_VENDOR_RESULTS_NOTIF_EVENT_TYPE: number of defined gscan results
 *	notification event types.
 *
 * Note that these must match the firmware API.
 */
enum iwl_mvm_vendor_results_event_type {
	IWL_MVM_VENDOR_RESULTS_NOTIF_BUFFER_FULL,
	IWL_MVM_VENDOR_RESULTS_NOTIF_BUCKET_END,
	NUM_IWL_VENDOR_RESULTS_NOTIF_EVENT_TYPE,
};

/**
 * enum iwl_mvm_vendor_gscan_result - gscan scan result
 * @IWL_MVM_VENDOR_GSCAN_RESULT_INVALID: attribute number 0 is reserved.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP: time since boot (in usecs) when
 *	the result was retrieved.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_SSID: SSID.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_BSSID: BSSID of the BSS (6 octets).
 * @IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL: channel frequency in MHz.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_RSSI: signal strength in dB.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_FRAME: the whole beacon/probe response
 *	frame data including the header.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_BEACON_PERIOD: period advertised in the beacon.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_CAPABILITY: capabilities advertised in the
 *	beacon / probe response.
 * @IWL_MVM_VENDOR_GSCAN_RESULT_PAD: used for padding, ignore
 * @NUM_IWL_MVM_VENDOR_GSCAN_RESULT: number of scan result attributes.
 * @MAX_IWL_MVM_VENDOR_GSCAN_RESULT: highest scan result attribute number.
 */
enum iwl_mvm_vendor_gscan_result {
	IWL_MVM_VENDOR_GSCAN_RESULT_INVALID,
	IWL_MVM_VENDOR_GSCAN_RESULT_TIMESTAMP,
	IWL_MVM_VENDOR_GSCAN_RESULT_SSID,
	IWL_MVM_VENDOR_GSCAN_RESULT_BSSID,
	IWL_MVM_VENDOR_GSCAN_RESULT_CHANNEL,
	IWL_MVM_VENDOR_GSCAN_RESULT_RSSI,
	IWL_MVM_VENDOR_GSCAN_RESULT_FRAME,
	IWL_MVM_VENDOR_GSCAN_RESULT_BEACON_PERIOD,
	IWL_MVM_VENDOR_GSCAN_RESULT_CAPABILITY,
	IWL_MVM_VENDOR_GSCAN_RESULT_PAD,
	NUM_IWL_MVM_VENDOR_GSCAN_RESULT,
	MAX_IWL_MVM_VENDOR_GSCAN_RESULT =
		NUM_IWL_MVM_VENDOR_GSCAN_RESULT - 1,
};

/**
 * enum iwl_mvm_vendor_gscan_cached_scan_res - gscan cached scan result
 * @IWL_MVM_VENDOR_GSCAN_CACHED_RES_INVALID: attribute number 0 is reserved.
 * @IWL_MVM_VENDOR_GSCAN_CACHED_RES_SCAN_ID: unique ID for this cached result.
 * @IWL_MVM_VENDOR_GSCAN_CACHED_RES_FLAGS: additional information about this
 *	scan iteration.
 * @IWL_MVM_VENDOR_GSCAN_CACHED_RES_APS: APs reported in this scan iteration.
 * @NUM_IWL_MVM_VENDOR_GSCAN_CACHED_RES: number of scan result attributes.
 * @MAX_IWL_MVM_VENDOR_GSCAN_CACHED_RES: highest scan result attribute number.
 */
enum iwl_mvm_vendor_gscan_cached_scan_res {
	IWL_MVM_VENDOR_GSCAN_CACHED_RES_INVALID,
	IWL_MVM_VENDOR_GSCAN_CACHED_RES_SCAN_ID,
	IWL_MVM_VENDOR_GSCAN_CACHED_RES_FLAGS,
	IWL_MVM_VENDOR_GSCAN_CACHED_RES_APS,
	NUM_IWL_MVM_VENDOR_GSCAN_CACHED_RES,
	MAX_IWL_MVM_VENDOR_GSCAN_CACHED_RES =
		NUM_IWL_MVM_VENDOR_GSCAN_CACHED_RES - 1,
};

/**
 * enum iwl_mvm_vendor_ap_threshold_param - parameters for tracking AP's RSSI
 * @IWL_MVM_VENDOR_AP_THRESHOLD_PARAM_INVALID: attribute number 0 is reserved.
 * @IWL_MVM_VENDOR_AP_BSSID: BSSID of the BSS (6 octets)
 * @IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD: low RSSI threshold. in dB.
 * @IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD: high RSSI threshold. in dB.
 * @NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM: number of ap threshold param
 *	attributes.
 * @MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM: highest ap threshold param
 *	attribute number.
 */
enum iwl_mvm_vendor_ap_threshold_param {
	IWL_MVM_VENDOR_AP_THRESHOLD_PARAM_INVALID,
	IWL_MVM_VENDOR_AP_BSSID,
	IWL_MVM_VENDOR_AP_LOW_RSSI_THRESHOLD,
	IWL_MVM_VENDOR_AP_HIGH_RSSI_THRESHOLD,
	NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM,
	MAX_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM =
		NUM_IWL_MVM_VENDOR_GSCAN_AP_THRESHOLD_PARAM - 1,
};

/**
 * enum iwl_mvm_vendor_hotlist_ap_status - whether an AP was found or lost
 * @IWL_MVM_VENDOR_HOTLIST_AP_FOUND: beacon from this AP was received with RSSI
 *	above the configured high threshold.
 * @IWL_MVM_VENDOR_HOTLIST_AP_LOST: beacon from this AP was received with RSSI
 *	below the configured low threshold.
 * @NUM_IWL_MVM_VENDOR_HOTLIST_AP_STATUS: number of defined AP statuses.
 *
 * Note that these must match the firmware API.
 */
enum iwl_mvm_vendor_hotlist_ap_status {
	IWL_MVM_VENDOR_HOTLIST_AP_FOUND,
	IWL_MVM_VENDOR_HOTLIST_AP_LOST,
	NUM_IWL_MVM_VENDOR_HOTLIST_AP_STATUS,
};

/**
 * enum iwl_mvm_vendor_significant_change_result - significant change result
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_CHANNEL: channel number of the reported
 *	AP.
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_BSSID: BSSID.
 * @IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RSSI_HISTORY: array of RSSI samples for
 *	the reported AP. in dB.
 * @NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT: number of significant change
 *	attriutes.
 * @MAX_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT: highest significant change
 *	result attribute number.
 */
enum iwl_mvm_vendor_significant_change_result {
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_INVALID,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_CHANNEL,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_BSSID,
	IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RSSI_HISTORY,
	NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT,
	MAX_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT =
	NUM_IWL_MVM_VENDOR_SIGNIFICANT_CHANGE_RESULT - 1,
};

/**
 * enum iwl_mvm_vendor_rxfilter_flags - the type of request rxfilter
 *
 * @IWL_MVM_VENDOR_RXFILTER_UNICAST: control unicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_BCAST: control broadcast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_MCAST4: control IPv4 multicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_MCAST6: control IPv4 multicast Rx filter
 * @IWL_MVM_VENDOR_RXFILTER_EINVAL: no Rx filter command was set
 *
 */
enum iwl_mvm_vendor_rxfilter_flags {
	IWL_MVM_VENDOR_RXFILTER_UNICAST = 1 << 0,
	IWL_MVM_VENDOR_RXFILTER_BCAST = 1 << 1,
	IWL_MVM_VENDOR_RXFILTER_MCAST4 = 1 << 2,
	IWL_MVM_VENDOR_RXFILTER_MCAST6 = 1 << 3,
	IWL_MVM_VENDOR_RXFILTER_EINVAL = 1 << 7,
};

/**
 * enum iwl_mvm_vendor_rxfilter_op - the operation associated with a filter
 *
 * @IWL_MVM_VENDOR_RXFILTER_OP_PASS: pass frames matching the filter
 * @IWL_MVM_VENDOR_RXFILTER_OP_DROP: drop frames matching the filter
 */
enum iwl_mvm_vendor_rxfilter_op {
	IWL_MVM_VENDOR_RXFILTER_OP_PASS,
	IWL_MVM_VENDOR_RXFILTER_OP_DROP,
};

/*
 * enum iwl_mvm_vendor_nr_chan_width - channel width definitions
 *
 * The values in this enum correspond to the values defined in
 * IEEE802.11-2016, table 9-153.
 */
enum iwl_mvm_vendor_nr_chan_width {
	IWL_MVM_VENDOR_CHAN_WIDTH_20,
	IWL_MVM_VENDOR_CHAN_WIDTH_40,
	IWL_MVM_VENDOR_CHAN_WIDTH_80,
	IWL_MVM_VENDOR_CHAN_WIDTH_160,
	IWL_MVM_VENDOR_CHAN_WIDTH_80P80,
};

/*
 * enum iwl_mvm_vendor_phy_type - neighbor report phy types
 *
 * The values in this enum correspond to the values defined in
 * IEEE802.11-2016, Annex C.
 */
enum iwl_mvm_vendor_phy_type {
	IWL_MVM_VENDOR_PHY_TYPE_UNSPECIFIED,
	IWL_MVM_VENDOR_PHY_TYPE_DSSS = 2,
	IWL_MVM_VENDOR_PHY_TYPE_OFDM = 4,
	IWL_MVM_VENDOR_PHY_TYPE_HRDSSS = 5,
	IWL_MVM_VENDOR_PHY_TYPE_ERP = 6,
	IWL_MVM_VENDOR_PHY_TYPE_HT = 7,
	IWL_MVM_VENDOR_PHY_TYPE_DMG = 8,
	IWL_MVM_VENDOR_PHY_TYPE_VHT = 9,
	IWL_MVM_VENDOR_PHY_TYPE_TVHT = 10,
};

/**
 * enum iwl_mvm_vendor_neighbor_report - Neighbor report for one AP
 *
 * @__IWL_MVM_VENDOR_NEIGHBOR_INVALID: attribute number 0 is reserved
 * @IWL_MVM_VENDOR_NEIGHBOR_BSSID: the BSSID of the neighbor AP.
 * @IWL_MVM_VENDOR_NEIGHBOR_BSSID_INFO: the BSSID information field as
 *	defined in IEEE802.11-2016, figure 9-296 (u32)
 * @IWL_MVM_VENDOR_NEIGHBOR_OPERATING_CLASS: the operating class of the
 *	neighbor AP (u8)
 * @IWL_MVM_VENDOR_NEIGHBOR_CHANNEL: the primary channel number of the
 *	neighbor AP (u8)
 * @IWL_MVM_VENDOR_NEIGHBOR_PHY_TYPE: the phy type of the neighbor AP
 *	as specified in &enum iwl_mvm_vendor_phy_type (u8)
 * @IWL_MVM_VENDOR_NEIGHBOR_CHANNEL_WIDTH: u32 attribute containing one of the
 *	values of &enum iwl_mvm_vendor_nr_chan_width, describing the
 *	channel width.
 * @IWL_MVM_VENDOR_NEIGHBOR_CENTER_FREQ_IDX_0: Center frequency of the first
 *	part of the channel, used for anything but 20 MHz bandwidth.
 * @IWL_MVM_VENDOR_NEIGHBOR_CENTER_FREQ_IDX_1: Center frequency of the second
 *	part of the channel, used only for 80+80 MHz bandwidth.
 * @IWL_MVM_VENDOR_NEIGHBOR_LCI: the LCI info of the neighbor AP. Optional.
 *	Binary attribute.
 * @IWL_MVM_VENDOR_NEIGHBOR_CIVIC: the CIVIC info of the neighbor AP. Optional.
 *	Binary attribute.
 * @NUM_IWL_MVM_VENDOR_NEIGHBOR_REPORT: num of neighbor report attributes
 * @MAX_IWL_MVM_VENDOR_NEIGHBOR_REPORT: highest neighbor report attribute
 *	number.

 */
enum iwl_mvm_vendor_neighbor_report {
	__IWL_MVM_VENDOR_NEIGHBOR_INVALID,
	IWL_MVM_VENDOR_NEIGHBOR_BSSID,
	IWL_MVM_VENDOR_NEIGHBOR_BSSID_INFO,
	IWL_MVM_VENDOR_NEIGHBOR_OPERATING_CLASS,
	IWL_MVM_VENDOR_NEIGHBOR_CHANNEL,
	IWL_MVM_VENDOR_NEIGHBOR_PHY_TYPE,
	IWL_MVM_VENDOR_NEIGHBOR_CHANNEL_WIDTH,
	IWL_MVM_VENDOR_NEIGHBOR_CENTER_FREQ_IDX_0,
	IWL_MVM_VENDOR_NEIGHBOR_CENTER_FREQ_IDX_1,
	IWL_MVM_VENDOR_NEIGHBOR_LCI,
	IWL_MVM_VENDOR_NEIGHBOR_CIVIC,

	NUM_IWL_MVM_VENDOR_NEIGHBOR_REPORT,
	MAX_IWL_MVM_VENDOR_NEIGHBOR_REPORT =
		NUM_IWL_MVM_VENDOR_NEIGHBOR_REPORT - 1,
};

/**
 * enum iwl_vendor_sar_per_chain_geo_table - per chain tx power table
 *
 * @IWL_VENDOR_SAR_GEO_INVALID: attribute number 0 is reserved.
 * @IWL_VENDOR_SAR_GEO_CHAIN_A_OFFSET: allowed offset for chain a (u8).
 * @IWL_VENDOR_SAR_GEO_CHAIN_B_OFFSET: allowed offset for chain b (u8).
 * @IWL_VENDOR_SAR_GEO_MAX_TXP: maximum allowed tx power (u8).
 */
enum iwl_vendor_sar_per_chain_geo_table {
	IWL_VENDOR_SAR_GEO_INVALID,
	IWL_VENDOR_SAR_GEO_CHAIN_A_OFFSET,
	IWL_VENDOR_SAR_GEO_CHAIN_B_OFFSET,
	IWL_VENDOR_SAR_GEO_MAX_TXP,
};

/**
 * enum iwl_mvm_vendor_attr - attributes used in vendor commands
 * @__IWL_MVM_VENDOR_ATTR_INVALID: attribute 0 is invalid
 * @IWL_MVM_VENDOR_ATTR_LOW_LATENCY: low-latency flag attribute
 * @IWL_MVM_VENDOR_ATTR_VIF_ADDR: interface MAC address
 * @IWL_MVM_VENDOR_ATTR_VIF_LL: vif-low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_LL: global low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_VIF_LOAD: vif traffic load (u8, see load enum)
 * @IWL_MVM_VENDOR_ATTR_LOAD: global traffic load (u8, see load enum)
 * @IWL_MVM_VENDOR_ATTR_COUNTRY: MCC to set, for regulatory information (u16)
 * IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA: filter gratuitous ARP and unsolicited
 *	Neighbor Advertisement frames
 * IWL_MVM_VENDOR_ATTR_FILTER_GTK: filter Filtering Frames Encrypted using
 *	the GTK
 * @IWL_MVM_VENDOR_ATTR_ADDR: MAC address
 * @IWL_MVM_VENDOR_ATTR_TX_BYTES: number of bytes transmitted to peer
 * @IWL_MVM_VENDOR_ATTR_RX_BYTES: number of bytes received from peer
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24: TX power limit for 2.4 GHz
 *	(s32 in units of 1/8 dBm)
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L: TX power limit for 5.2 GHz low (as 2.4)
 * @IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H: TX power limit for 5.2 GHz high (as 2.4)
 * @IWL_MVM_VENDOR_ATTR_OPPPS_WA: wa to pass Sigma test
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_CACHE_SIZE: scan cache size
 *	(in bytes)
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_BUCKETS: maximum number of channel
 *	buckets
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_CACHE_PER_SCAN: maximum number of AP's
 *	that can be stored per scan
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_RSSI_SAMPLE_SIZE: number of RSSI samples
 *	used for averaging RSSI
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_REPORTING_THRESHOLD: max possible
 *	report threshold. see %IWL_MVM_VENDOR_ATTR_GSCAN_START_REPORT_THRESHOLD
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_APS: maximum number of entries for
 *	hotlist AP's
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SIGNIFICANT_CHANGE_APS: maximum number of
 *	entries for significant change AP's
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_BSSID_HISTORY_ENTRIES: number of
 *	BSSID/RSSI entries that the device can hold
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR: mac address to be used on gscan scans
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK: mac address mask. Bits set to 0
 *	will be copied from %IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR. Bits set to 1
 *	will be randomized
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN: number of AP's to store in each
 *	scan in the BSSID/RSSI history buffer (keep the highest RSSI AP's)
 * @IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD: report that scan results
 *	are available when buffer is that much full. In percentage.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS: array of bucket specifications for
 *	this gscan start command. Each bucket spec is a nested attribute of
 *	&enum iwl_mvm_vendor_gscan_bucket_spec.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS_EVENT_TYPE: gscan results event type as
 *	specified in &enum iwl_mvm_vendor_results_event_type.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS: array of gscan results. Each result is a
 *	nested attribute of &enum iwl_mvm_vendor_gscan_result.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE: number of samples to confirm
 *	ap loss.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST: an array of nested attributes of
 *	&enum iwl_mvm_vendor_ap_threshold_param.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE: number of samples for averaging
 *	RSSI
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING: number of APs breaching threshold
 * @IWL_MVM_VENDOR_ATTR_GSCAN_HOTLIST_AP_STATUS: indicates if a reported AP was
 *	lost or found as specified in &enum iwl_mvm_vendor_hotlist_ap_status.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_SIG_CHANGE_RESULTS: array of significant
 *	change results. Each result is a nested attribute of &enum
 *	iwl_mvm_vendor_significant_change_result.
 * @IWL_MVM_VENDOR_ATTR_RXFILTER: u32 attribute.
 *      See %iwl_mvm_vendor_rxfilter_flags.
 * @IWL_MVM_VENDOR_ATTR_RXFILTER_OP: u32 attribute.
 *      See %iwl_mvm_vendor_rxfilter_op.
 * @IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER: description of collect debug data
 *	trigger.
 * @IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ: u32 attribute. Frequency (in MHz) to be
 *	used for NAN further availability.
 * @IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS: u8 attribute. Number of 16TU slots
 *	the NAN device will be available on it's FAW between DWs.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_SSIDS: maximum number of entries for
 *	hotlist SSID's
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_EPNO_NETWORKS: max number of epno entries
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_EPNO_NETWORKS_BY_SSID: max number of epno
 *	entries if ssid is specified
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_WHITE_LISTED_SSID: max number of white
 *	listed SSIDs
 * @IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_BLACK_LISTED_SSID: max number of black
 *	listed SSIDs
 *
 * @NUM_IWL_MVM_VENDOR_ATTR: number of vendor attributes
 * @MAX_IWL_MVM_VENDOR_ATTR: highest vendor attribute number
 * @IWL_MVM_VENDOR_ATTR_WIPHY_FREQ: frequency of the selected channel in MHz,
 *	defines the channel together with the attributes
 *	%IWL_MVM_VENDOR_ATTR_CHANNEL_WIDTH and if needed
 *	%IWL_MVM_VENDOR_ATTR_CENTER_FREQ1 and
 *	%IWL_MVM_VENDOR_ATTR_CENTER_FREQ2.
 * @IWL_MVM_VENDOR_ATTR_CHANNEL_WIDTH: u32 attribute containing one of the
 *	values of &enum nl80211_chan_width, describing the channel width.
 *	See the documentation of the enum for more information.
 * @IWL_MVM_VENDOR_ATTR_CENTER_FREQ1: Center frequency of the first part of the
 *	channel, used for anything but 20 MHz bandwidth.
 * @IWL_MVM_VENDOR_ATTR_CENTER_FREQ2: Center frequency of the second part of
 *	the channel, used only for 80+80 MHz bandwidth.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD_NUM: report that scan results
 *	are available when buffer is that much full. In number of scans.
 * @IWL_MVM_VENDOR_ATTR_GSCAN_CACHED_RESULTS: array of gscan cached results.
 *	Each result is a nested attribute of
 *	&enum iwl_mvm_vendor_gscan_cached_scan_res.
 * @IWL_MVM_VENDOR_ATTR_SSID: SSID (binary attribute, 0..32 octets)
 * @IWL_MVM_VENDOR_ATTR_NEIGHBOR_LCI: Flag attribute specifying that the
 *	neighbor request shall query for LCI information.
 * @IWL_MVM_VENDOR_ATTR_NEIGHBOR_CIVIC: Flag attribute specifying that the
 *	neighbor request shall query for CIVIC information.
 * @IWL_MVM_VENDOR_ATTR_NEIGHBOR_REPORT: A list of neighbor APs as received in a
 *	neighbor report frame. Each AP is a nested attribute of
 *	&enum iwl_mvm_vendor_neighbor_report.
 * @IWL_MVM_VENDOR_ATTR_LAST_MSG: Indicates that this message is the last one
 *	in the series of messages. (flag)
 * @IWL_MVM_VENDOR_ATTR_SAR_CHAIN_A_PROFILE: SAR table idx for chain A.
 *	This is a u8.
 * @IWL_MVM_VENDOR_ATTR_SAR_CHAIN_B_PROFILE: SAR table idx for chain B.
 *	This is a u8.
 * @IWL_MVM_VENDOR_ATTR_SAR_ENABLED_PROFILE_NUM: number of enabled SAR profile
 *	This is a u8.
 * @IWL_MVM_VENDOR_ATTR_SAR_GEO_PROFILE: geo profile info.
 *	see &enum iwl_vendor_sar_per_chain_geo_table.
 *
 */
enum iwl_mvm_vendor_attr {
	__IWL_MVM_VENDOR_ATTR_INVALID,
	IWL_MVM_VENDOR_ATTR_LOW_LATENCY,
	IWL_MVM_VENDOR_ATTR_VIF_ADDR,
	IWL_MVM_VENDOR_ATTR_VIF_LL,
	IWL_MVM_VENDOR_ATTR_LL,
	IWL_MVM_VENDOR_ATTR_VIF_LOAD,
	IWL_MVM_VENDOR_ATTR_LOAD,
	IWL_MVM_VENDOR_ATTR_COUNTRY,
	IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA,
	IWL_MVM_VENDOR_ATTR_FILTER_GTK,
	IWL_MVM_VENDOR_ATTR_ADDR,
	IWL_MVM_VENDOR_ATTR_TX_BYTES,
	IWL_MVM_VENDOR_ATTR_RX_BYTES,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_24,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52L,
	IWL_MVM_VENDOR_ATTR_TXP_LIMIT_52H,
	IWL_MVM_VENDOR_ATTR_OPPPS_WA,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_CACHE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_BUCKETS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_CACHE_PER_SCAN,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_RSSI_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SCAN_REPORTING_THRESHOLD,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_APS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_SIGNIFICANT_CHANGE_APS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_BSSID_HISTORY_ENTRIES,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAC_ADDR_MASK,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_AP_PER_SCAN,
	IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD,
	IWL_MVM_VENDOR_ATTR_GSCAN_BUCKET_SPECS,
	IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS_EVENT_TYPE,
	IWL_MVM_VENDOR_ATTR_GSCAN_RESULTS,
	IWL_MVM_VENDOR_ATTR_GSCAN_LOST_AP_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_AP_LIST,
	IWL_MVM_VENDOR_ATTR_GSCAN_RSSI_SAMPLE_SIZE,
	IWL_MVM_VENDOR_ATTR_GSCAN_MIN_BREACHING,
	IWL_MVM_VENDOR_ATTR_GSCAN_HOTLIST_AP_STATUS,
	IWL_MVM_VENDOR_ATTR_GSCAN_SIG_CHANGE_RESULTS,
	IWL_MVM_VENDOR_ATTR_RXFILTER,
	IWL_MVM_VENDOR_ATTR_RXFILTER_OP,
	IWL_MVM_VENDOR_ATTR_DBG_COLLECT_TRIGGER,
	IWL_MVM_VENDOR_ATTR_NAN_FAW_FREQ,
	IWL_MVM_VENDOR_ATTR_NAN_FAW_SLOTS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_HOTLIST_SSIDS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_EPNO_NETWORKS,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_EPNO_NETWORKS_BY_SSID,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_WHITE_LISTED_SSID,
	IWL_MVM_VENDOR_ATTR_GSCAN_MAX_NUM_BLACK_LISTED_SSID,
	IWL_MVM_VENDOR_ATTR_WIPHY_FREQ,
	IWL_MVM_VENDOR_ATTR_CHANNEL_WIDTH,
	IWL_MVM_VENDOR_ATTR_CENTER_FREQ1,
	IWL_MVM_VENDOR_ATTR_CENTER_FREQ2,
	IWL_MVM_VENDOR_ATTR_GSCAN_REPORT_THRESHOLD_NUM,
	IWL_MVM_VENDOR_ATTR_GSCAN_CACHED_RESULTS,
	IWL_MVM_VENDOR_ATTR_LAST_MSG,
	IWL_MVM_VENDOR_ATTR_SAR_CHAIN_A_PROFILE,
	IWL_MVM_VENDOR_ATTR_SAR_CHAIN_B_PROFILE,
	IWL_MVM_VENDOR_ATTR_SAR_ENABLED_PROFILE_NUM,
	IWL_MVM_VENDOR_ATTR_SSID,
	IWL_MVM_VENDOR_ATTR_NEIGHBOR_LCI,
	IWL_MVM_VENDOR_ATTR_NEIGHBOR_CIVIC,
	IWL_MVM_VENDOR_ATTR_NEIGHBOR_REPORT,
	IWL_MVM_VENDOR_ATTR_SAR_GEO_PROFILE,

	NUM_IWL_MVM_VENDOR_ATTR,
	MAX_IWL_MVM_VENDOR_ATTR = NUM_IWL_MVM_VENDOR_ATTR - 1,
};
#define IWL_MVM_VENDOR_FILTER_ARP_NA IWL_MVM_VENDOR_ATTR_FILTER_ARP_NA
#define IWL_MVM_VENDOR_FILTER_GTK IWL_MVM_VENDOR_ATTR_FILTER_GTK
#endif /* __VENDOR_CMD_H__ */
