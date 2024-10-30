/*
 * SPDX-FileCopyrightText: Copyright (c) 2020 Qorvo, Inc.
 * SPDX-License-Identifier: LicenseRef-QORVO-1 OR GPL-2.0
 */

#pragma once

#define UCI_FIRA_PHY_VERSION_MAJOR 1
#define UCI_FIRA_PHY_VERSION_MINOR 1
#define UCI_FIRA_TEST_VERSION_MAJOR 1
#define UCI_FIRA_TEST_VERSION_MINOR 1

/*
 * According to Table 35 - Fira Consortium UWB Command Interface Generic
 * Technical Specification 2.0.0
 */
#define UCI_FIRA_TWR_MEASUREMENT_DISTANCE_INVALID 0xffff

/**
 * enum uci_common_packet_header - Common Packet Header layout description
 * @UCI_COMMON_PACKET_HEADER_MT_OFFSET:
 *     Offset in the Common Packet Header Octet 0 of the MT field.
 * @UCI_COMMON_PACKET_HEADER_MT_MASK:
 *     Mask of the MT field once shifted down.
 * @UCI_COMMON_PACKET_HEADER_PBF_OFFSET:
 *     Offset in the Common Packet Header Octet 0 of the PBF field.
 * @UCI_COMMON_PACKET_HEADER_PBF_MASK:
 *     Mask of the PBF field once shifted down.
 * @UCI_COMMON_PACKET_HEADER_INFO_OFFSET:
 *     Offset in the Common Packet Header Octet 0 of the INFO field.
 * @UCI_COMMON_PACKET_HEADER_INFO_MASK:
 *     Mask of the INFO field once shifted down.
 */
enum uci_common_packet_header {
	UCI_COMMON_PACKET_HEADER_MT_OFFSET = 5,
	UCI_COMMON_PACKET_HEADER_MT_MASK = 0b111,
	UCI_COMMON_PACKET_HEADER_PBF_OFFSET = 4,
	UCI_COMMON_PACKET_HEADER_PBF_MASK = 1,
	UCI_COMMON_PACKET_HEADER_INFO_OFFSET = 0,
	UCI_COMMON_PACKET_HEADER_INFO_MASK = 0b1111,
};

/**
 * enum uci_message_pbf - PBF flag definition.
 *
 * @UCI_MESSAGE_PBF_COMPLETE:
 *     If the Packet contains a complete Message, the PBF SHALL be set to 0b0.
 *     If the Packet contains the last segment of a segmented Message, the PBF
 *     SHALL be set to 0b0.
 * @UCI_MESSAGE_PBF_SEGMENT:
 *     If the packet does not contain the last segment of a segmented Message,
 * the PBF SHALL be set to 0b1.
 */
enum uci_message_pbf {
	UCI_MESSAGE_PBF_COMPLETE = 0b0,
	UCI_MESSAGE_PBF_SEGMENT = 0b1,
};

/**
 * enum uci_message_type - Message type definition.
 *
 * @UCI_MESSAGE_TYPE_DATA:
 *     The message is a data packet
 * @UCI_MESSAGE_TYPE_COMMAND:
 *     The message is a command.
 * @UCI_MESSAGE_TYPE_RESPONSE:
 *     The message is a response to a command.
 * @UCI_MESSAGE_TYPE_NOTIFICATION:
 *     The message is a notification.
 * @UCI_MESSAGE_TYPE_SE_TESTING_COMMAND:
 *     The message is a test command for the Secure Element.
 * @UCI_MESSAGE_TYPE_SE_TESTING_RESPONSE:
 *     The message is a test response from the Secure Element.
 */
enum uci_message_type {
	UCI_MESSAGE_TYPE_DATA = 0b000,
	UCI_MESSAGE_TYPE_COMMAND = 0b001,
	UCI_MESSAGE_TYPE_RESPONSE = 0b010,
	UCI_MESSAGE_TYPE_NOTIFICATION = 0b011,
	UCI_MESSAGE_TYPE_SE_TESTING_COMMAND = 0b100,
	UCI_MESSAGE_TYPE_SE_TESTING_RESPONSE = 0b101,
	/* RFU 0b110 - 0b111 */
};
/**
 * enum uci_message_dpf - Message Data Packet Format (DPF) definition.
 *
 * @UCI_MESSAGE_DPF_SEND:
 *     Endpoint Format send (DATA_MESSAGE_SND)
 * @UCI_MESSAGE_DPF_RECEIVE:
 *     Endpoint Format receive (DATA_MESSAGE_RCV)
 * @UCI_MESSAGE_DPF_LL_SEND:
 *     Endpoint Format send (LL_DATA_MESSAGE_SND)
 * @UCI_MESSAGE_DPF_LL_RECEIVE:
 *     Endpoint Format receive (LL_DATA_MESSAGE_RCV)
 */
enum uci_message_dpf {
	/* UCI_MESSAGE_DPF_RFU = 0b0000,  */
	UCI_MESSAGE_DPF_SEND = 0b0001,
	UCI_MESSAGE_DPF_RECEIVE = 0b0010,
	UCI_MESSAGE_DPF_LL_SEND = 0b0011,
	UCI_MESSAGE_DPF_LL_RECEIVE = 0b0100,
	/* RFU 0b0101 - 0b1111 */
};

/**
 * enum uci_message_gid - Group identifiers definition.
 *
 * @UCI_GID_CORE:
 *     GID used for generic uci command on device.
 * @UCI_GID_SESSION_CONFIG:
 *     GID used for fira session command.
 * @UCI_GID_SESSION_CONTROL:
 *     GID used for starting stopping a session.
 * @UCI_GID_QORVO_EXT1:
 *     Secure Element group.
 * @UCI_GID_QORVO_EXT2:
 *     GID used for qorvo's additional physical tests and diagnostics.
 * @UCI_GID_ANDROID:
 *     GID used in Android HAL.
 * @UCI_GID_TEST:
 *     Test session group.
 * @UCI_GID_QORVO_MAC:
 *     GID used for uwbmac over uci feature.
 * @UCI_GID_QORVO_CALIB:
 *     Calibration reset tool.
 */
enum uci_message_gid {
	UCI_GID_CORE = 0b0000,
	UCI_GID_SESSION_CONFIG = 0b0001,
	UCI_GID_SESSION_CONTROL = 0b0010,
	/* RFU 0b0011 - 0b1000 */
	/* Vendor 0b1001 - 0b1100 */
	UCI_GID_QORVO_EXT1 = 0b1001,
	UCI_GID_QORVO_EXT2 = 0b1011,
	UCI_GID_ANDROID = 0b1100,
	/* spec PCTT */
	UCI_GID_TEST = 0b1101,
	/* Vendor 0b1110 - 0b1111 */
	UCI_GID_QORVO_MAC = 0b1110,
	/* TODO: to be removed and merged with UCI_GID_QORVO_MAC. */
	UCI_GID_QORVO_CALIB = 0b1111,
};

/**
 * enum uci_oid_core - Opcode identifiers for UCI_GID_CORE.
 *
 * @UCI_OID_CORE_DEVICE_RESET:
 *     Reset device.
 * @UCI_OID_CORE_DEVICE_STATUS:
 *     Device state notification (ready, active, error).
 * @UCI_OID_CORE_GET_DEVICE_INFO:
 *     Get UCI version information as well as device specific ones.
 * @UCI_OID_CORE_GET_CAPS_INFO:
 *     Get device capabilities.
 * @UCI_OID_CORE_SET_CONFIG:
 *     Set device configuration.
 * @UCI_OID_CORE_GET_CONFIG:
 *     Get device configuration.
 * @UCI_OID_CORE_GENERIC_ERROR:
 *     Error notification.
 * @UCI_OID_CORE_QUERY_UWBS_TIMESTAMP:
 *     Get timestamp.
 */
enum uci_oid_core {
	UCI_OID_CORE_DEVICE_RESET = 0b000000,
	UCI_OID_CORE_DEVICE_STATUS = 0b000001,
	UCI_OID_CORE_GET_DEVICE_INFO = 0b000010,
	UCI_OID_CORE_GET_CAPS_INFO = 0b000011,
	UCI_OID_CORE_SET_CONFIG = 0b000100,
	UCI_OID_CORE_GET_CONFIG = 0b000101,
	/* RFU = 0b000110 */
	UCI_OID_CORE_GENERIC_ERROR = 0b000111,
	UCI_OID_CORE_QUERY_UWBS_TIMESTAMP = 0b001000,
	/* RFU = 0b111000 - 0b111111 */
};

/**
 * enum uci_oid_session_config - Opcode identifiers for UCI_GID_SESSION_CONFIG.
 *
 * @UCI_OID_SESSION_INIT:
 *     Init session.
 * @UCI_OID_SESSION_DEINIT:
 *     Deinit session.
 * @UCI_OID_SESSION_STATUS:
 *     Session state notification (init, idle, active, deinit).
 * @UCI_OID_SESSION_SET_APP_CONFIG:
 *     Set session configuration.
 * @UCI_OID_SESSION_GET_APP_CONFIG:
 *     Get session configuration.
 * @UCI_OID_SESSION_GET_COUNT:
 *     Get the number of session in use.
 * @UCI_OID_SESSION_GET_STATE:
 *     Get the state of one session.
 * @UCI_OID_SESSION_UPDATE_CONTROLLER_MULTICAST_LIST:
 *     Update the list of controlee of a session.
 * @UCI_OID_SESSION_UPDATE_DT_ANCHOR_RANGING_ROUNDS:
 *     Configure the active ranging rounds of DT-Anchor, ranging
 *     roles for each active round and lists of Responders for each
 *     active round in which DT-Anchor acts as Initiator.
 * @UCI_OID_SESSION_UPDATE_DT_TAG_RANGING_ROUNDS:
 *     Configure the active ranging rounds of DT-Tag.
 * @UCI_OID_SESSION_QUERY_DATA_SIZE_IN_RANGING:
 *     Get maximum Application Data size in Ranging Round.
 * @UCI_OID_SESSION_SET_HUS_CONTROLLER_CONFIG:
 *     Configure phases of a HUS ranging round on a Controller device.
 * @UCI_OID_SESSION_SET_HUS_CONTROLEE_CONFIG:
 *     Configure phases of a HUS ranging round on a Controlee device.
 * @UCI_OID_SESSION_DATA_TRANSFER_PHASE_CONFIG:
 *     Configure slots allocations for data transfer.
 */
enum uci_oid_session_config {
	UCI_OID_SESSION_INIT = 0b000000,
	UCI_OID_SESSION_DEINIT = 0b000001,
	UCI_OID_SESSION_STATUS = 0b000010,
	UCI_OID_SESSION_SET_APP_CONFIG = 0b000011,
	UCI_OID_SESSION_GET_APP_CONFIG = 0b000100,
	UCI_OID_SESSION_GET_COUNT = 0b000101,
	UCI_OID_SESSION_GET_STATE = 0b000110,
	UCI_OID_SESSION_UPDATE_CONTROLLER_MULTICAST_LIST = 0b000111,
	UCI_OID_SESSION_UPDATE_DT_ANCHOR_RANGING_ROUNDS = 0b001000,
	UCI_OID_SESSION_UPDATE_DT_TAG_RANGING_ROUNDS = 0b001001,
	UCI_OID_SESSION_QUERY_DATA_SIZE_IN_RANGING = 0b001011,
	UCI_OID_SESSION_SET_HUS_CONTROLLER_CONFIG = 0b001100,
	UCI_OID_SESSION_SET_HUS_CONTROLEE_CONFIG = 0b001101,
	UCI_OID_SESSION_DATA_TRANSFER_PHASE_CONFIG = 0b001110,
	/* RFU 0b001111 - 0b111111 */
};

/**
 * enum uci_oid_session_control - Opcode identifiers for UCI_GID_SESSION_CONTROL.
 *
 * @UCI_OID_SESSION_START:
 *     Start session.
 * @UCI_OID_SESSION_INFO:
 *     Ranging session notification.
 * @UCI_OID_SESSION_STOP:
 *     Stop session.
 * @UCI_OID_SESSION_GET_RANGING_COUNT:
 *     Get the number of ranging done by an active session.
 * @UCI_OID_SESSION_DATA_CREDIT:
 *     Notification to indicate that data packet is successfully received by UWBS.
 * @UCI_OID_SESSION_DATA_TRANSFER_STATUS:
 *     Notification to indicate the status of in-band data transfer.
 * @UCI_OID_LOGICAL_LINK_CREATE:
 *     Create a logical link for data exchange.
 * @UCI_OID_LOGICAL_LINK_CLOSE:
 *     Close a logical link for data exchange.
 * @UCI_OID_LOGICAL_LINK_UWBS_CLOSE:
 *     Notification sent by UWBS to indicate release of a logical link.
 * @UCI_OID_LOGICAL_LINK_UWBS_CREATE:
 *     Notification sent by UWBS to indicate creation of a logical link.
 * @UCI_OID_LOGICAL_LINK_GET_PARAM:
 *     Get logical link layer configuration parameters.
 */
enum uci_oid_session_control {
	UCI_OID_SESSION_START = 0b000000,
	UCI_OID_SESSION_INFO = UCI_OID_SESSION_START,
	UCI_OID_SESSION_STOP = 0b000001,
	UCI_OID_SESSION_GET_RANGING_COUNT = 0b000011,
	UCI_OID_SESSION_DATA_CREDIT = 0b000100,
	UCI_OID_SESSION_DATA_TRANSFER_STATUS = 0b000101,
	UCI_OID_LOGICAL_LINK_CREATE = 0b000111,
	UCI_OID_LOGICAL_LINK_CLOSE = 0b001000,
	UCI_OID_LOGICAL_LINK_UWBS_CLOSE = 0b001001,
	UCI_OID_LOGICAL_LINK_UWBS_CREATE = 0b001010,
	UCI_OID_LOGICAL_LINK_GET_PARAM = 0b001011,
	/* RFU 0b001100 - 0b111111 */
};

/**
 * enum uci_oid_test - Opcode identifiers for UCI_GID_TEST.
 *
 * @UCI_OID_TEST_CONFIG_SET:
 *     Set configuration specific to testing.
 * @UCI_OID_TEST_CONFIG_GET:
 *     Get test configuration.
 * @UCI_OID_TEST_PERIODIC_TX:
 *     Execute periodic tx test.
 * @UCI_OID_TEST_PER_RX:
 *     Execute per rx test.
 * @UCI_OID_TEST_RX:
 *     Execute rx test.
 * @UCI_OID_TEST_LOOPBACK:
 *     Execute loopback test.
 * @UCI_OID_TEST_STOP_SESSION:
 *     Stop a test.
 * @UCI_OID_TEST_SS_TWR:
 *     Execute ss twr test.
 * @UCI_OID_TEST_SR_RX:
 *     Execute sr rx test.
 */
enum uci_oid_test {
	UCI_OID_TEST_CONFIG_SET = 0b000000,
	UCI_OID_TEST_CONFIG_GET = 0b000001,
	UCI_OID_TEST_PERIODIC_TX = 0b000010,
	UCI_OID_TEST_PER_RX = 0b000011,
	UCI_OID_TEST_RX = 0b000101,
	UCI_OID_TEST_LOOPBACK = 0b000110,
	UCI_OID_TEST_STOP_SESSION = 0b000111,
	UCI_OID_TEST_SS_TWR = 0b001000,
	UCI_OID_TEST_SR_RX = 0b001001,
	/* RFU 0b000100 */
	/* RFU 0b001010 - 0b111111 */
};

/**
 * enum uci_status_code - Status of a command.
 *
 * @UCI_STATUS_OK:
 *     Success.
 * @UCI_STATUS_REJECTED:
 *     Rejected.
 * @UCI_STATUS_FAILED:
 *     Unknown failure.
 * @UCI_STATUS_SYNTAX_ERROR:
 *     Syntax error in the UCI command.
 * @UCI_STATUS_INVALID_PARAM:
 *     A parameter was not valid.
 * @UCI_STATUS_INVALID_RANGE:
 *     A known parameter value was out of possible range.
 * @UCI_STATUS_INVALID_MESSAGE_SIZE:
 *     The size of the command message was wrong.
 * @UCI_STATUS_UNKNOWN_GID:
 *     Unknow GID.
 * @UCI_STATUS_UNKNOWN_OID:
 *     Unknow OID.
 * @UCI_STATUS_READ_ONLY:
 *     Tried to set a read only value.
 * @UCI_STATUS_UCI_MESSAGE_RETRY:
 *     UWBS request retransmission from Host.
 * @UCI_STATUS_UNKNOWN:
 *     It is not known whether the intended operation was failed or successful.
 * @UCI_STATUS_NOT_APPLICABLE:
 *     The parameter is not applicable for the requested operation.
 * @UCI_STATUS_ERROR_SESSION_NOT_EXIST:
 *     Given session does not exist.
 * @STATUS_ERROR_INVALID_PHASE_PARTICIPATION:
 *    Invalid phase participation values during primary/secondary sessions binding.
 * @UCI_STATUS_ERROR_SESSION_ACTIVE:
 *     Session is active.
 * @UCI_STATUS_ERROR_MAX_SESSIONS_EXCEEDED:
 *     Max. number of sessions already created.
 * @UCI_STATUS_ERROR_SESSION_NOT_CONFIGURED:
 *     Session is not configured with required app configurations.
 * @UCI_STATUS_ERROR_ACTIVE_SESSIONS_ONGOING:
 *     Sessions are actively running in UWBS.
 * @UCI_STATUS_ERROR_MULTICAST_LIST_FULL:
 *     Indicates when multicast list is full during one to many ranging.
 * @UCI_STATUS_ERROR_ADDRESS_NOT_FOUND:
 *     FiRa 1.3 only, RFU on newer version.
 *     Indicate one controlee of the update multicast list command was not
 *     found.
 * @UCI_STATUS_ERROR_UWB_INITIATION_TIME_TOO_OLD:
 *     The current UWBS time has gone past the configured UWB_INITIATION_TIME.
 * @UCI_STATUS_OK_NEGATIVE_DISTANCE_REPORT:
 *     Success.A negative distance was measured.
 * @UCI_STATUS_RANGING_TX_FAILED:
 *     Failed to transmit UWB packet.
 * @UCI_STATUS_RANGING_RX_TIMEOUT:
 *     No UWB packet detected by the receiver.
 * @UCI_STATUS_RANGING_RX_PHY_DEC_FAILED:
 *     UWB packet channel decoding error.
 * @UCI_STATUS_RANGING_RX_PHY_TOA_FAILED:
 *     Failed to detect time of arrival of the UWB packet from CIR samples.
 * @UCI_STATUS_RANGING_RX_PHY_STS_FAILED:
 *     UWB packet STS segment mismatch.
 * @UCI_STATUS_RANGING_RX_MAC_DEC_FAILED:
 *     MAC CRC or syntax error.
 * @UCI_STATUS_RANGING_RX_MAC_IE_DEC_FAILED:
 *     IE syntax error.
 * @UCI_STATUS_RANGING_RX_MAC_IE_MISSING:
 *     Expected IE missing in the packet.
 * @UCI_STATUS_ERROR_ROUND_INDEX_NOT_ACTIVATED:
 *     Configured DL-TDoA Ranging Round index could not be activated.
 * @UCI_STATUS_ERROR_NUMBER_OF_ACTIVE_RANGING_ROUNDS_EXCEEDED:
 *     Number of active ranging rounds exceeds the maximum number of ranging
 *     rounds supported.
 * @UCI_STATUS_ERROR_DL_TDOA_DEVICE_ADDRESS_NOT_MATCHING_IN_REPLY_TIME_LIST:
 *     Received DL-TDoA Reply Time List does not contain the Initiator Reply
 *     Time associated to the MAC address in RDM List.
 * @UCI_STATUS_ERROR_SE_BUSY:
 *     Proprietary CCC error: Secure Element is busy.
 * @UCI_STATUS_ERROR_CCC_LIFECYCLE:
 *     Proprietary CCC error: lifecycle has not been respected.
 * @UCI_STATUS_LAST:
 *     Internal use.
 */
enum uci_status_code {
	/* Generic Status Codes */
	UCI_STATUS_OK = 0x00,
	UCI_STATUS_REJECTED = 0x01,
	UCI_STATUS_FAILED = 0x02,
	UCI_STATUS_SYNTAX_ERROR = 0x03,
	UCI_STATUS_INVALID_PARAM = 0x04,
	UCI_STATUS_INVALID_RANGE = 0x05,
	UCI_STATUS_INVALID_MESSAGE_SIZE = 0x06,
	UCI_STATUS_UNKNOWN_GID = 0x07,
	UCI_STATUS_UNKNOWN_OID = 0x08,
	UCI_STATUS_READ_ONLY = 0x09,
	UCI_STATUS_UCI_MESSAGE_RETRY = 0x0a,
	UCI_STATUS_UNKNOWN = 0x0b,
	UCI_STATUS_NOT_APPLICABLE = 0x0c,
	/* RFU 0x0b - 0x0f */

	/* UWB Session Specific Status Codes */
	/* Note: 0x10 is missing from the documentation */
	UCI_STATUS_ERROR_SESSION_NOT_EXIST = 0x11,
	STATUS_ERROR_INVALID_PHASE_PARTICIPATION = 0x12,
	UCI_STATUS_ERROR_SESSION_ACTIVE = 0x13,
	UCI_STATUS_ERROR_MAX_SESSIONS_EXCEEDED = 0x14,
	UCI_STATUS_ERROR_SESSION_NOT_CONFIGURED = 0x15,
	UCI_STATUS_ERROR_ACTIVE_SESSIONS_ONGOING = 0x16,
	UCI_STATUS_ERROR_MULTICAST_LIST_FULL = 0x17,
	/* 0x18 is defined in 1.3, but RFU starting 2.0 */
	UCI_STATUS_ERROR_ADDRESS_NOT_FOUND = 0x18,
	/* RFU 0x18 - 0x19 */
	UCI_STATUS_ERROR_UWB_INITIATION_TIME_TOO_OLD = 0x1a,
	UCI_STATUS_OK_NEGATIVE_DISTANCE_REPORT = 0x1b,
	/* RFU 0x1c - 0x1f */

	/* UWB Ranging Session Specific Status Codes */
	UCI_STATUS_RANGING_TX_FAILED = 0x20,
	UCI_STATUS_RANGING_RX_TIMEOUT = 0x21,
	UCI_STATUS_RANGING_RX_PHY_DEC_FAILED = 0x22,
	UCI_STATUS_RANGING_RX_PHY_TOA_FAILED = 0x23,
	UCI_STATUS_RANGING_RX_PHY_STS_FAILED = 0x24,
	UCI_STATUS_RANGING_RX_MAC_DEC_FAILED = 0x25,
	UCI_STATUS_RANGING_RX_MAC_IE_DEC_FAILED = 0x26,
	UCI_STATUS_RANGING_RX_MAC_IE_MISSING = 0x27,
	UCI_STATUS_ERROR_ROUND_INDEX_NOT_ACTIVATED = 0x28,
	UCI_STATUS_ERROR_NUMBER_OF_ACTIVE_RANGING_ROUNDS_EXCEEDED = 0x29,
	UCI_STATUS_ERROR_DL_TDOA_DEVICE_ADDRESS_NOT_MATCHING_IN_REPLY_TIME_LIST = 0x2a,
	/* RFU 0x2c - 0x4f */

	/* Proprietary Status Codes 0x50 - 0xff */
	UCI_STATUS_ERROR_SE_BUSY = 0x50,
	UCI_STATUS_ERROR_CCC_LIFECYCLE = 0x51,
	UCI_STATUS_LAST,
};

/**
 * enum uci_data_transfer_status_code - Status Code values in the
 * SESSION_DATA_TRANSFER_STATUS_NTF
 *
 * @UCI_DATA_TRANSFER_STATUS_REPETITION_OK:
 *     When DATA_REPETITION_COUNT > 0 and
 *     SESSION_DATA_TRANSFER_STATUS_NTF_CONFIG = Enable it indicates that one
 *     data transmission is completed in a RR.
 * @UCI_DATA_TRANSFER_STATUS_OK:
 *     Complete Application Data has been fully transmitted over UWB.
 * @UCI_DATA_TRANSFER_STATUS_ERROR_DATA_TRANSFER:
 *     Application Data couldn’t be sent due to an unrecoverable error.
 * @UCI_DATA_TRANSFER_STATUS_ERROR_NO_CREDIT_AVAILABLE:
 *     DATA_MESSAGE_SND is not accepted as no credit is available.
 * @UCI_DATA_TRANSFER_STATUS_ERROR_REJECTED:
 *     DATA_MESSAGE_SND packet sent in wrong state or Application Data Size
 *     exceeds the maximum size that can be sent in one Ranging Round.
 * @UCI_DATA_TRANSFER_STATUS_SESSION_TYPE_NOT_SUPPORTED:
 *     Data transfer is not supported for given session type.
 * @UCI_DATA_TRANSFER_STATUS_ERROR_DATA_TRANSFER_IS_ONGOING:
 *     Application Data is being transmitted and the number of configured
 *     DATA_REPETITION_COUNT transmissions is not yet completed.
 * @UCI_DATA_TRANSFER_STATUS_INVALID_FORMAT:
 *     The format of the command DATA_MESSAGE_SND associated with this
 * notification is incorrect (e.g, a parameter is missing, a parameter value is
 * invalid).
 * @UCI_DATA_TRANSFER_STATUS_LAST:
 *     Internal use.
 */
enum uci_data_transfer_status_code {
	UCI_DATA_TRANSFER_STATUS_REPETITION_OK = 0x00,
	UCI_DATA_TRANSFER_STATUS_OK = 0x01,
	UCI_DATA_TRANSFER_STATUS_ERROR_DATA_TRANSFER = 0x02,
	UCI_DATA_TRANSFER_STATUS_ERROR_NO_CREDIT_AVAILABLE = 0x03,
	UCI_DATA_TRANSFER_STATUS_ERROR_REJECTED = 0x04,
	UCI_DATA_TRANSFER_STATUS_SESSION_TYPE_NOT_SUPPORTED = 0x05,
	UCI_DATA_TRANSFER_STATUS_ERROR_DATA_TRANSFER_IS_ONGOING = 0x06,
	UCI_DATA_TRANSFER_STATUS_INVALID_FORMAT = 0x07,
	/* RFU 0x08 - 0x1f */
	UCI_DATA_TRANSFER_STATUS_LAST,
};

/**
 * enum uci_device_state - Device state.
 *
 * @UCI_DEVICE_STATE_READY:
 *     Device is ready.
 * @UCI_DEVICE_STATE_ACTIVE:
 *     Device is performing a communication or ranging.
 * @UCI_DEVICE_STATE_ERROR:
 *     Device needs to be reset.
 */
enum uci_device_state {
	/* RFU 0x00 */
	UCI_DEVICE_STATE_READY = 0x01,
	UCI_DEVICE_STATE_ACTIVE = 0x02,
	/* RFU 0x03 - 0xfe */
	UCI_DEVICE_STATE_ERROR = 0xff,
};

/**
 * enum uci_device_configuration_parameters - Device configuration.
 *
 * @UCI_DEVICE_PARAMETER_DEVICE_STATE:
 *     Read only device state.
 * @UCI_DEVICE_PARAMETER_LOW_POWER_MODE:
 *     Low power mode.
 *
 * For MCPS specific parameters, see ``enum device_config`` in
 * ``uci_spec_mcps.h``.
 */
enum uci_device_configuration_parameters {
	UCI_DEVICE_PARAMETER_DEVICE_STATE = 0x00,
	UCI_DEVICE_PARAMETER_LOW_POWER_MODE = 0x01,
	/* RFU 0x02 - 0x9f */
	/* Reserved for Vendor Specific 0xa0 - 0xdf */
	/* RFU 0xe0 - 0xe2 */
	/* Vendor Specific parameters 0xe3 - 0xff */
};

/**
 * enum uci_application_configuration_parameters - Session configuration.
 *
 * @UCI_APPLICATION_PARAMETER_DEVICE_TYPE:
 *     Device type (controller or controlee).
 * @UCI_APPLICATION_PARAMETER_RANGING_ROUND_USAGE:
 *     Ranging round usage.
 * @UCI_APPLICATION_PARAMETER_STS_CONFIG:
 *     STS config.
 * @UCI_APPLICATION_PARAMETER_MULTI_NODE_MODE:
 *     Multi node mode.
 * @UCI_APPLICATION_PARAMETER_CHANNEL_NUMBER:
 *     Channel number (5 or 9).
 * @UCI_APPLICATION_PARAMETER_NUMBER_OF_CONTROLEES:
 *     Number of controlees.
 * @UCI_APPLICATION_PARAMETER_NUMBER_RESPONDERS_NODES:
 *     Number of responders nodes.
 * @UCI_APPLICATION_PARAMETER_DEVICE_MAC_ADDRESS:
 *     Device mac address.
 * @UCI_APPLICATION_PARAMETER_DST_MAC_ADDRESS:
 *     List of distant mac addresses.
 * @UCI_APPLICATION_PARAMETER_SLOT_DURATION:
 *     Slot duration.
 * @UCI_APPLICATION_PARAMETER_RANGING_INTERVAL:
 *     Ranging interval (aka RANGING_DURATION).
 * @UCI_APPLICATION_PARAMETER_STS_INDEX:
 *     STS index.
 * @UCI_APPLICATION_PARAMETER_STS_INDEX0:
 *     STS index 0.
 * @UCI_APPLICATION_PARAMETER_MAC_FCS_TYPE:
 *     MAC fcs type.
 * @UCI_APPLICATION_PARAMETER_RANGING_ROUND_CONTROL:
 *     Ranging round control.
 * @UCI_APPLICATION_PARAMETER_AOA_RESULT_REQ:
 *     AOA result requirement.
 * @UCI_APPLICATION_PARAMETER_SESSION_INFO_NTF_CONFIG:
 *     Range data ntf config.
 * @UCI_APPLICATION_PARAMETER_NEAR_PROXIMITY_CONFIG:
 *     Range data ntf proximity near.
 * @UCI_APPLICATION_PARAMETER_FAR_PROXIMITY_CONFIG:
 *     Range data ntf proximity far.
 * @UCI_APPLICATION_PARAMETER_DEVICE_ROLE:
 *     Device role (initiator or responder).
 * @UCI_APPLICATION_PARAMETER_RFRAME_CONFIG:
 *     RFrame config.
 * @UCI_APPLICATION_PARAMETER_RSSI_REPORTING:
 *     RSSI report.
 * @UCI_APPLICATION_PARAMETER_PREAMBLE_CODE_INDEX:
 *     Preamble code index.
 * @UCI_APPLICATION_PARAMETER_SYNC_CODE_INDEX:
 *     Parameter sync code index.
 * @UCI_APPLICATION_PARAMETER_SFD_ID:
 *     SDF id.
 * @UCI_APPLICATION_PARAMETER_PSDU_DATA_RATE:
 *     PSDU data rate.
 * @UCI_APPLICATION_PARAMETER_PREAMBLE_DURATION:
 *     Preamble duration.
 * @UCI_APPLICATION_PARAMETER_LINK_LAYER_MODE:
 *     Link layer mode.
 * @UCI_APPLICATION_PARAMETER_DATA_REPETITION_COUNT:
 *     Data repetition count.
 * @UCI_APPLICATION_PARAMETER_RANGING_TIME_STRUCT:
 *     Ranging time struct.
 * @UCI_APPLICATION_PARAMETER_SLOTS_PER_RR:
 *     Slots per ranging round.
 * @UCI_APPLICATION_PARAMETER_NUMBER_SLOT_PER_ROUND:
 *     Number of slot per round.
 * @UCI_APPLICATION_PARAMETER_TX_ADAPTIVE_PAYLOAD_POWER:
 *     TX adaptive payload power.
 * @UCI_APPLICATION_PARAMETER_AOA_BOUND_CONFIG:
 *     AOA_BOUND_CONFIG.
 * @UCI_APPLICATION_PARAMETER_RESPONDER_SLOT_INDEX:
 *     Responder slot index.
 * @UCI_APPLICATION_PARAMETER_PRF_MODE:
 *     PRF mode.
 * @UCI_APPLICATION_PARAMETER_CAP_SIZE_RANGE:
 *     Maximum and minimum CAP size for contention-based ranging.
 * @UCI_APPLICATION_PARAMETER_SCHEDULE_MODE:
 *     Schedule mode.
 * @UCI_APPLICATION_PARAMETER_KEY_ROTATION:
 *     Key rotation.
 * @UCI_APPLICATION_PARAMETER_KEY_ROTATION_RATE:
 *     Key rotation rate.
 * @UCI_APPLICATION_PARAMETER_SESSION_PRIORITY:
 *     Session priority.
 * @UCI_APPLICATION_PARAMETER_MAC_ADDRESS_MODE:
 *     MAC adress mode.
 * @UCI_APPLICATION_PARAMETER_VENDOR_ID:
 *     Vendor id.
 * @UCI_APPLICATION_PARAMETER_STATIC_STS_IV:
 *     Static STS IV.
 * @UCI_APPLICATION_PARAMETER_NUMBER_OF_STS_SEGMENTS:
 *     Number of STS segments.
 * @UCI_APPLICATION_PARAMETER_MAX_RR_RETRY:
 *     Max number of ranging round retry.
 * @UCI_APPLICATION_PARAMETER_UWB_INITIATION_TIME:
 *     UWB initiation time.
 * @UCI_APPLICATION_PARAMETER_HOPPING_MODE:
 *     Hopping mode.
 * @UCI_APPLICATION_PARAMETER_HOP_CONFIG_BITMASK:
 *     Hopping configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_BLOCK_STRIDE_LENGTH:
 *     Block stride length.
 * @UCI_APPLICATION_PARAMETER_RESULT_REPORT_CONFIG:
 *     Result report config.
 * @UCI_APPLICATION_PARAMETER_IN_BAND_TERMINATION_ATTEMPT_COUNT:
 *     In band termination attempt count.
 * @UCI_APPLICATION_PARAMETER_SUB_SESSION_ID:
 *     Sub session id.
 * @UCI_APPLICATION_PARAMETER_BPRF_PHR_DATA_RATE:
 *     BPRF PHR data rate.
 * @UCI_APPLICATION_PARAMETER_MAX_NUMBER_OF_MEASUREMENTS:
 *     Max number of measurements.
 * @UCI_APPLICATION_PARAMETER_UL_TDOA_TX_INTERVAL:
 *     The average transmission interval of Blink UTMs sent from UT-Tags
 *     and Synchronization UTMs sent from UT-Synchronization Anchors.
 * @UCI_APPLICATION_PARAMETER_UL_TDOA_RANDOM_WINDOW:
 *     The length of the randomization window within which Blink UTMs
 *     and Synchronization UTMs may be transmitted
 * @UCI_APPLICATION_PARAMETER_STS_LENGTH:
 *     STS length.
 * @UCI_APPLICATION_PARAMETER_UL_TDOA_NTF_REPORT_CONFIG:
 *     UT-Anchor configuration to specify the condition under which SESSION_INFO_NTF to be reported.
 * @UCI_APPLICATION_PARAMETER_UL_TDOA_DEVICE_ID:
 *     Length of the UL-TDoA device identifier (and the ID itself) sent in UTMs.
 * @UCI_APPLICATION_PARAMETER_UL_TDOA_TX_TIMESTAMP:
 *     Presence and length of TX timestamps sent in UTMs.
 * @UCI_APPLICATION_PARAMETER_MIN_FRAMES_PER_RR:
 *     Min number of frames to be transmitted in OWR AoA ranging round (block).
 * @UCI_APPLICATION_PARAMETER_MTU_SIZE:
 *     Max Transfer Unit, max size allowed to be transmitted in Data Message.
 * @UCI_APPLICATION_PARAMETER_INTER_FRAME_INTERVAL:
 *     Interval between RFRAMES transmitted in OWR for AoA
 *     (in units of 1200 RSTU)
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_RANGING_METHOD:
 *     specifies whether a DL-TDoA ranging round shall be based on a
 *     SS-TWR like message exchange or a DS-TWR message.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_TX_TIMESTAMP_CONF:
 *     configure the type (local time base vs. common time base)
 *     and length of TX timestamps included in DTM messages.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_HOP_COUNT:
 *     Presence of Hop Count field in Poll DTMs from Initiator.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_ANCHOR_CFO:
 *     Presence of DT-Anchor CFO estimates in Response.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_ANCHOR_LOCATION:
 *     Presence of DT-Anchor location in DTM messages
 *     and location configuration.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_TX_ACTIVE_RANGING_ROUNDS:
 *     Presence of active ranging round information in Poll
 *     and Response DTM messages.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_BLOCK_SKIPPING:
 *     configure the number of blocks that shall be skipped by a DT-Tag
 *     between two active ranging blocks.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_TIME_REFERENCE_ANCHOR:
 *     Set or reset a DT-Anchor to be the global time reference of
 *     a DL-TDoA network.
 * @UCI_APPLICATION_PARAMETER_SESSION_KEY:
 *     Session key for provisioned STS.
 * @UCI_APPLICATION_PARAMETER_SUB_SESSION_KEY:
 *     Sub-session key for provisioned STS.
 * @UCI_APPLICATION_PARAMETER_DATA_TRANSFER_STATUS_NTF_CONFIG:
 *     Used to configure the SESSION_DATA_TRANSFER_STATUS_NTF.
 * @UCI_APPLICATION_PARAMETER_SESSION_TIME_BASE:
 *     Used to configure the SESSION_TIME_BASE.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_RESPONDER_TOF:
 *     Configure whether a DT-Anchor with the Responder role in a
 *     given ranging round shall include the estimated Responder ToF
 *     Result in a Response DTM.
 * @UCI_APPLICATION_PARAMETER_SECURE_RANGING_NEFA_LEVEL:
 *     Normalized effective false acceptance rate (NEFA)
 * @UCI_APPLICATION_PARAMETER_SECURE_RANGING_CSW_LENGTH:
 *     PHY-layer critical search window
 * @UCI_APPLICATION_PARAMETER_APPLICATION_DATA_ENDPOINT:
 *     Local endpoint configuration for the session
 * @UCI_APPLICATION_PARAMETER_OWR_AOA_MEASUREMENT_NTF_PERIOD:
 *     Configure periodicity in which the aggregated OWR AOA measurement
 *     is reported.
 * @UCI_APPLICATION_PARAMETER_AFTER_LAST_NOT_VENDOR:
 *     Internal use.
 * @UCI_APPLICATION_PARAMETER_HOP_MODE_KEY:
 *     Vendor configuration for hopping.
 * @UCI_APPLICATION_PARAMETER_CCC_UWB_TIME0:
 *     Vendor configuration for CCC initiation time.
 * @UCI_APPLICATION_PARAMETER_SELECTED_PROTOCOL_VERSION:
 *     Vendor configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_SELECTED_UWB_CONFIG_ID:
 *     Vendor configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_SELECTED_PULSE_SHAPE_COMBO:
 *     Vendor configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_URSK_TTL:
 *     Vendor configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_CCC_STS_INDEX:
 *     Vendor configuration for CCC.
 * @UCI_APPLICATION_PARAMETER_MAC_MODE:
 *     Set the number of ranging rounds per block and the offset between rounds.
 * @UCI_APPLICATION_PARAMETER_URSK:
 *     Provision URSK through UCI.
 * @UCI_APPLICATION_PARAMETER_RADAR_TIMING_PARAMS:
 *     Octet [3:0]: BURST_PERIOD
 * 	   Duration between the start of two,
 * 	   consecutive Radar bursts in ms
 * 	   Octet [5:4]: SWEEP_PERIOD
 * 	   Duration between the start times of two
 * 	   consecutive Radar sweeps in RSTU.
 * 	   Octet [6]: SWEEPS_PER_BURST
 * 	   Number of Radar sweeps within
 *     the Radar burst
 * @UCI_APPLICATION_PARAMETER_RADAR_SAMPLES_PER_SWEEP:
 *     Number of samples (taps) per sweep. Default = 64.
 * @UCI_APPLICATION_PARAMETER_RADAR_SWEEP_OFFSET:
 *     Number of samples offset before First Path.
 * @UCI_APPLICATION_PARAMETER_RADAR_BITS_PER_SAMPLE:
 *     Number of bits per CIR sample in radar session.
 * @UCI_APPLICATION_PARAMETER_RADAR_NUMBER_OF_BURSTS:
 *     Number of bursts in radar session.
 * @UCI_APPLICATION_PARAMETER_RADAR_DATA_TYPE:
 *     Type of radar data to be reported in RADAR_DATA_MESSAGE.
 * @UCI_APPLICATION_PARAMETER_RADAR_ANTENNA_SET_ID:
 *     Antenna set id used.
 * @UCI_APPLICATION_PARAMETER_TX_PROFILE_IDX:
 *     Index of TX profile composed of three values: level of TX_power,
 *     PLL_common and TX_ctrl
 * @UCI_APPLICATION_PARAMETER_NB_OF_RANGE_MEASUREMENTS:
 *     Number of range measurements in interleaved mode.
 * @UCI_APPLICATION_PARAMETER_NB_OF_AZIMUTH_MEASUREMENTS:
 *     Number of azimuth measurements in interleaved mode.
 * @UCI_APPLICATION_PARAMETER_NB_OF_ELEVATION_MEASUREMENTS:
 *     Number of elevation measurements in interleaved mode.
 * @UCI_APPLICATION_PARAMETER_RX_ANTENNA_SELECTION:
 *     RX antenna set id.
 * @UCI_APPLICATION_PARAMETER_TX_ANTENNA_SELECTION:
 *     TX antenna set id.
 * @UCI_APPLICATION_PARAMETER_ENABLE_DIAGNOSTICS:
 *     Enable diagnostic notification in FIRA or PCTT region.
 * @UCI_APPLICATION_PARAMETER_DIAGS_FRAME_REPORTS_FIELDS:
 *     Select the fields to activate in the frame reports
 *     stored in the diags. If the ENABLE_DIAGNOSTICS is
 *     not true this parameters does not activate the diags
 *     itself.
 * @UCI_APPLICATION_PARAMETER_MAC_PAYLOAD_ENCRYPTION:
 *     Enable or disable encryption of payload data.
 * @UCI_APPLICATION_PARAMETER_ENABLE_PSDU_DUMP:
 *     Enable or disable PSDU dump.
 * @UCI_APPLICATION_PARAMETER_DL_TDOA_CROSS_CLUSTER_TIME_SYNC:
 * 	   Enable/Disable the Dl-Tdoa cross-cluster time synchronization feature; An 8 bit field,
 *     where only bit0 is used, the rest is RFU and need to be set to 0.
 * @UCI_APPLICATION_PARAMETER_PAN_ID: The identifier of the PAN which the device belongs to.
 * @UCI_APPLICATION_PARAMETER_OMLOX_RANGING_ROUND_SET: The combination of ranging round types
 *     (GTSW vs LTW) to use in the session.
 * @UCI_APPLICATION_PARAMETER_OMLOX_PRIMARY_SATELLITE: True if this device will act as the
 *     Primary Satellite, false otherwise.
 * @UCI_APPLICATION_PARAMETER_OMLOX_SATELLITE_POSITION: The position of this satellite.
 * @UCI_APPLICATION_PARAMETER_OMLOX_RCP_INIT_TX_SLOT: Slot indexes on which the satellite
 *     should transmit during the xRCP and/or INIT phases.
 * @UCI_APPLICATION_PARAMETER_OMLOX_RCP_INIT_RX_SLOTS: Slot indexes on which the satellite
 *     should receive during the xRCP and/or INIT phases.
 * @UCI_APPLICATION_PARAMETER_OMLOX_RANGING_MODE: The ranging mode (type of measurement) this device
 *     should use.
 * @UCI_APPLICATION_PARAMETER_OMLOX_RSP_PREAMBLE_CODE_INDEX: The preamble code index for this
 *     session for RSP phase, BPRF (9-24), HPRF (25-32) (default: 10)
 * @UCI_APPLICATION_PARAMETER_OMLOX_TAG_MOVING: True if the nearby OMLOX tag is currently moving,
 *     false otherwise.
 */
enum uci_application_configuration_parameters {
	UCI_APPLICATION_PARAMETER_DEVICE_TYPE = 0x00,
	UCI_APPLICATION_PARAMETER_RANGING_ROUND_USAGE = 0x01,
	UCI_APPLICATION_PARAMETER_STS_CONFIG = 0x02,
	UCI_APPLICATION_PARAMETER_MULTI_NODE_MODE = 0x03,
	UCI_APPLICATION_PARAMETER_CHANNEL_NUMBER = 0x04,
	UCI_APPLICATION_PARAMETER_NUMBER_OF_CONTROLEES = 0x05,
	UCI_APPLICATION_PARAMETER_NUMBER_RESPONDERS_NODES /* CCC */
	= UCI_APPLICATION_PARAMETER_NUMBER_OF_CONTROLEES,
	UCI_APPLICATION_PARAMETER_DEVICE_MAC_ADDRESS = 0x06,
	UCI_APPLICATION_PARAMETER_DST_MAC_ADDRESS = 0x07,
	UCI_APPLICATION_PARAMETER_SLOT_DURATION = 0x08,
	UCI_APPLICATION_PARAMETER_RANGING_INTERVAL = 0x09,
	UCI_APPLICATION_PARAMETER_STS_INDEX = 0x0a,
	UCI_APPLICATION_PARAMETER_STS_INDEX0 /* CCC */
	= UCI_APPLICATION_PARAMETER_STS_INDEX,
	UCI_APPLICATION_PARAMETER_MAC_FCS_TYPE = 0x0b,
	UCI_APPLICATION_PARAMETER_RANGING_ROUND_CONTROL = 0x0c,
	UCI_APPLICATION_PARAMETER_AOA_RESULT_REQ = 0x0d,
	UCI_APPLICATION_PARAMETER_SESSION_INFO_NTF_CONFIG = 0x0e,
	UCI_APPLICATION_PARAMETER_NEAR_PROXIMITY_CONFIG = 0x0f,
	UCI_APPLICATION_PARAMETER_FAR_PROXIMITY_CONFIG = 0x10,
	UCI_APPLICATION_PARAMETER_DEVICE_ROLE = 0x11,
	UCI_APPLICATION_PARAMETER_RFRAME_CONFIG = 0x12,
	UCI_APPLICATION_PARAMETER_RSSI_REPORTING = 0x13,
	UCI_APPLICATION_PARAMETER_PREAMBLE_CODE_INDEX = 0x14,
	UCI_APPLICATION_PARAMETER_SYNC_CODE_INDEX /* CCC */
	= UCI_APPLICATION_PARAMETER_PREAMBLE_CODE_INDEX,
	UCI_APPLICATION_PARAMETER_SFD_ID = 0x15,
	UCI_APPLICATION_PARAMETER_PSDU_DATA_RATE = 0x16,
	UCI_APPLICATION_PARAMETER_PREAMBLE_DURATION = 0x17,
	UCI_APPLICATION_PARAMETER_LINK_LAYER_MODE = 0x18,
	UCI_APPLICATION_PARAMETER_DATA_REPETITION_COUNT = 0x19,
	UCI_APPLICATION_PARAMETER_RANGING_TIME_STRUCT = 0x1a,
	UCI_APPLICATION_PARAMETER_SLOTS_PER_RR = 0x1b,
	UCI_APPLICATION_PARAMETER_NUMBER_SLOT_PER_ROUND /* CCC */
	= UCI_APPLICATION_PARAMETER_SLOTS_PER_RR,
	UCI_APPLICATION_PARAMETER_TX_ADAPTIVE_PAYLOAD_POWER = 0x1c,
	UCI_APPLICATION_PARAMETER_AOA_BOUND_CONFIG = 0x1d,
	UCI_APPLICATION_PARAMETER_RESPONDER_SLOT_INDEX = 0x1e,
	UCI_APPLICATION_PARAMETER_PRF_MODE = 0x1f,
	UCI_APPLICATION_PARAMETER_CAP_SIZE_RANGE = 0x20,
	/* UCI_APPLICATION_PARAMETER_CONTENTION_PHASE_UPDATE_LENGTH = 0x21, */
	UCI_APPLICATION_PARAMETER_SCHEDULE_MODE = 0x22,
	UCI_APPLICATION_PARAMETER_KEY_ROTATION = 0x23,
	UCI_APPLICATION_PARAMETER_KEY_ROTATION_RATE = 0x24,
	UCI_APPLICATION_PARAMETER_SESSION_PRIORITY = 0x25,
	UCI_APPLICATION_PARAMETER_MAC_ADDRESS_MODE = 0x26,
	UCI_APPLICATION_PARAMETER_VENDOR_ID = 0x27,
	UCI_APPLICATION_PARAMETER_STATIC_STS_IV = 0x28,
	UCI_APPLICATION_PARAMETER_NUMBER_OF_STS_SEGMENTS = 0x29,
	UCI_APPLICATION_PARAMETER_MAX_RR_RETRY = 0x2a,
	UCI_APPLICATION_PARAMETER_UWB_INITIATION_TIME = 0x2b,
	UCI_APPLICATION_PARAMETER_HOPPING_MODE = 0x2c,
	UCI_APPLICATION_PARAMETER_HOP_CONFIG_BITMASK /* CCC */
	= UCI_APPLICATION_PARAMETER_HOPPING_MODE,
	UCI_APPLICATION_PARAMETER_BLOCK_STRIDE_LENGTH = 0x2d,
	UCI_APPLICATION_PARAMETER_RESULT_REPORT_CONFIG = 0x2e,
	UCI_APPLICATION_PARAMETER_IN_BAND_TERMINATION_ATTEMPT_COUNT = 0x2f,
	UCI_APPLICATION_PARAMETER_SUB_SESSION_ID = 0x30,
	UCI_APPLICATION_PARAMETER_BPRF_PHR_DATA_RATE = 0x31,
	UCI_APPLICATION_PARAMETER_MAX_NUMBER_OF_MEASUREMENTS = 0x32,
	UCI_APPLICATION_PARAMETER_UL_TDOA_TX_INTERVAL = 0x33,
	UCI_APPLICATION_PARAMETER_UL_TDOA_RANDOM_WINDOW = 0x34,
	UCI_APPLICATION_PARAMETER_STS_LENGTH = 0x35,
	/* UNDEX = 0x36 */
	UCI_APPLICATION_PARAMETER_UL_TDOA_NTF_REPORT_CONFIG = 0x37,
	UCI_APPLICATION_PARAMETER_UL_TDOA_DEVICE_ID = 0x38,
	UCI_APPLICATION_PARAMETER_UL_TDOA_TX_TIMESTAMP = 0x39,
	UCI_APPLICATION_PARAMETER_MIN_FRAMES_PER_RR = 0x3a,
	UCI_APPLICATION_PARAMETER_MTU_SIZE = 0x3b,
	UCI_APPLICATION_PARAMETER_INTER_FRAME_INTERVAL = 0x3c,
	UCI_APPLICATION_PARAMETER_DL_TDOA_RANGING_METHOD = 0x3d,
	UCI_APPLICATION_PARAMETER_DL_TDOA_TX_TIMESTAMP_CONF = 0x3e,
	UCI_APPLICATION_PARAMETER_DL_TDOA_HOP_COUNT = 0x3f,
	UCI_APPLICATION_PARAMETER_DL_TDOA_ANCHOR_CFO = 0x40,
	UCI_APPLICATION_PARAMETER_DL_TDOA_ANCHOR_LOCATION = 0x41,
	UCI_APPLICATION_PARAMETER_DL_TDOA_TX_ACTIVE_RANGING_ROUNDS = 0x42,
	UCI_APPLICATION_PARAMETER_DL_TDOA_BLOCK_SKIPPING = 0x43,
	UCI_APPLICATION_PARAMETER_DL_TDOA_TIME_REFERENCE_ANCHOR = 0x44,
	UCI_APPLICATION_PARAMETER_SESSION_KEY = 0x45,
	UCI_APPLICATION_PARAMETER_SUB_SESSION_KEY = 0x46,
	UCI_APPLICATION_PARAMETER_DATA_TRANSFER_STATUS_NTF_CONFIG = 0x47,
	UCI_APPLICATION_PARAMETER_SESSION_TIME_BASE = 0x48,
	UCI_APPLICATION_PARAMETER_DL_TDOA_RESPONDER_TOF = 0x49,
	UCI_APPLICATION_PARAMETER_SECURE_RANGING_NEFA_LEVEL = 0x4a,
	UCI_APPLICATION_PARAMETER_SECURE_RANGING_CSW_LENGTH = 0x4b,
	UCI_APPLICATION_PARAMETER_APPLICATION_DATA_ENDPOINT = 0x4c,
	UCI_APPLICATION_PARAMETER_OWR_AOA_MEASUREMENT_NTF_PERIOD = 0x4d,
	UCI_APPLICATION_PARAMETER_AFTER_LAST_NOT_VENDOR,
	/* RFU 0x4e - 0x9f */
	/* CCC 0xa0 - 0xdf */
	/* RFU 0xe0 - 0xe2 */
	/* CCC specific */
	UCI_APPLICATION_PARAMETER_HOP_MODE_KEY = 0xa0,
	UCI_APPLICATION_PARAMETER_CCC_UWB_TIME0 = 0xa1,
	UCI_APPLICATION_PARAMETER_SELECTED_PROTOCOL_VERSION = 0xa3,
	UCI_APPLICATION_PARAMETER_SELECTED_UWB_CONFIG_ID = 0xa4,
	UCI_APPLICATION_PARAMETER_SELECTED_PULSE_SHAPE_COMBO = 0xa5,
	UCI_APPLICATION_PARAMETER_URSK_TTL = 0xa6,
	UCI_APPLICATION_PARAMETER_CCC_STS_INDEX = 0xa8,
	/* ACWG specific */
	UCI_APPLICATION_PARAMETER_MAC_MODE = 0xa9,
	UCI_APPLICATION_PARAMETER_URSK = 0xaa,
	/* Radar specific */
	UCI_APPLICATION_PARAMETER_RADAR_TIMING_PARAMS = 0xb0,
	UCI_APPLICATION_PARAMETER_RADAR_SAMPLES_PER_SWEEP = 0xb1,
	UCI_APPLICATION_PARAMETER_RADAR_SWEEP_OFFSET = 0xb2,
	UCI_APPLICATION_PARAMETER_RADAR_BITS_PER_SAMPLE = 0xb3,
	UCI_APPLICATION_PARAMETER_RADAR_NUMBER_OF_BURSTS = 0xb4,
	UCI_APPLICATION_PARAMETER_RADAR_DATA_TYPE = 0xb5,
	UCI_APPLICATION_PARAMETER_RADAR_ANTENNA_SET_ID = 0xb6,
	UCI_APPLICATION_PARAMETER_TX_PROFILE_IDX = 0xb7,
	/* Proprietary 0xe3 - 0xff */
	UCI_APPLICATION_PARAMETER_NB_OF_RANGE_MEASUREMENTS = 0xe3,
	UCI_APPLICATION_PARAMETER_NB_OF_AZIMUTH_MEASUREMENTS = 0xe4,
	UCI_APPLICATION_PARAMETER_NB_OF_ELEVATION_MEASUREMENTS = 0xe5,
	/* Antenna set selection for physical testing */
	UCI_APPLICATION_PARAMETER_RX_ANTENNA_SELECTION = 0xe6,
	UCI_APPLICATION_PARAMETER_TX_ANTENNA_SELECTION = 0xe7,
	UCI_APPLICATION_PARAMETER_ENABLE_DIAGNOSTICS = 0xe8,
	UCI_APPLICATION_PARAMETER_DIAGS_FRAME_REPORTS_FIELDS = 0xe9,
	UCI_APPLICATION_PARAMETER_MAC_PAYLOAD_ENCRYPTION = 0xea,
	/* PSDU dump */
	UCI_APPLICATION_PARAMETER_ENABLE_PSDU_DUMP = 0xeb,
	/* DL-TDOA specific*/
	UCI_APPLICATION_PARAMETER_DL_TDOA_CROSS_CLUSTER_TIME_SYNC = 0xec,
	/* OMLOX specific*/
	UCI_APPLICATION_PARAMETER_PAN_ID = 0xed,
	UCI_APPLICATION_PARAMETER_OMLOX_RANGING_ROUND_SET = 0xef,
	UCI_APPLICATION_PARAMETER_OMLOX_PRIMARY_SATELLITE = 0xf0,
	UCI_APPLICATION_PARAMETER_OMLOX_SATELLITE_POSITION = 0xf1,
	UCI_APPLICATION_PARAMETER_OMLOX_RCP_INIT_TX_SLOT = 0xf2,
	UCI_APPLICATION_PARAMETER_OMLOX_RCP_INIT_RX_SLOTS = 0xf3,
	UCI_APPLICATION_PARAMETER_OMLOX_RANGING_MODE = 0xf4,
	UCI_APPLICATION_PARAMETER_OMLOX_RSP_PREAMBLE_CODE_INDEX = 0xf5,
	UCI_APPLICATION_PARAMETER_OMLOX_TAG_MOVING = 0xf6,
};

/* FIXME => To remove later */
#define UCI_APPLICATION_PARAMETER_CCC_NUM UCI_APPLICATION_PARAMETER_HOP_CONFIG_BITMASK + 1

#define UCI_APPLICATION_PARAMETER_VENDOR1_CCC_BEGIN UCI_APPLICATION_PARAMETER_HOP_MODE_KEY
#define UCI_APPLICATION_PARAMETER_VENDOR1_CCC_END UCI_APPLICATION_PARAMETER_URSK
#define UCI_APPLICATION_PARAMETER_VENDOR1_CCC_NUM                                                  \
	(UCI_APPLICATION_PARAMETER_VENDOR1_CCC_END - UCI_APPLICATION_PARAMETER_VENDOR1_CCC_BEGIN + \
	 1)

#define UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(name)                         \
	(UCI_APPLICATION_PARAMETER_CCC_NUM + UCI_APPLICATION_PARAMETER_##name - \
	 UCI_APPLICATION_PARAMETER_VENDOR1_CCC_BEGIN)

#define UCI_APPLICATION_PARAMETER_VENDOR2_CCC_BEGIN UCI_APPLICATION_PARAMETER_RX_ANTENNA_SELECTION
#define UCI_APPLICATION_PARAMETER_VENDOR2_CCC_END UCI_APPLICATION_PARAMETER_ENABLE_PSDU_DUMP
#define UCI_APPLICATION_PARAMETER_VENDOR2_CCC_NUM                                                  \
	(UCI_APPLICATION_PARAMETER_VENDOR2_CCC_END - UCI_APPLICATION_PARAMETER_VENDOR2_CCC_BEGIN + \
	 1)

#define UCI_APPLICATION_PARAMETER_VENDOR2_CCC_TMP(name)                                  \
	(UCI_APPLICATION_PARAMETER_CCC_NUM + UCI_APPLICATION_PARAMETER_VENDOR1_CCC_NUM + \
	 UCI_APPLICATION_PARAMETER_##name - UCI_APPLICATION_PARAMETER_VENDOR2_CCC_BEGIN)

#define UCI_APPLICATION_PARAMETER_HOP_MODE_KEY_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(HOP_MODE_KEY)
#define UCI_APPLICATION_PARAMETER_CCC_UWB_TIME0_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(CCC_UWB_TIME0)
#define UCI_APPLICATION_PARAMETER_SELECTED_PROTOCOL_VERSION_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(SELECTED_PROTOCOL_VERSION)
#define UCI_APPLICATION_PARAMETER_SELECTED_UWB_CONFIG_ID_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(SELECTED_UWB_CONFIG_ID)
#define UCI_APPLICATION_PARAMETER_SELECTED_PULSE_SHAPE_COMBO_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(SELECTED_PULSE_SHAPE_COMBO)
#define UCI_APPLICATION_PARAMETER_URSK_TTL_TMP UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(URSK_TTL)
#define UCI_APPLICATION_PARAMETER_CCC_STS_INDEX_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(CCC_STS_INDEX)
#define UCI_APPLICATION_PARAMETER_MAC_MODE_TMP UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(MAC_MODE)
#define UCI_APPLICATION_PARAMETER_URSK_TMP UCI_APPLICATION_PARAMETER_VENDOR1_CCC_TMP(URSK)

#define UCI_APPLICATION_PARAMETER_RX_ANTENNA_SELECTION_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR2_CCC_TMP(RX_ANTENNA_SELECTION)
#define UCI_APPLICATION_PARAMETER_TX_ANTENNA_SELECTION_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR2_CCC_TMP(TX_ANTENNA_SELECTION)
#define UCI_APPLICATION_PARAMETER_MAC_PAYLOAD_ENCRYPTION_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR2_CCC_TMP(MAC_PAYLOAD_ENCRYPTION)
#define UCI_APPLICATION_PARAMETER_REPORT_PSDUS_TMP \
	UCI_APPLICATION_PARAMETER_VENDOR2_CCC_TMP(ENABLE_PSDU_DUMP)

/**
 * enum uci_device_type - Device type.
 *
 * @UCI_DEVICE_TYPE_CONTROLEE:
 *     Controlee.
 * @UCI_DEVICE_TYPE_CONTROLLER:
 *     Controller.
 */
enum uci_device_type {
	UCI_DEVICE_TYPE_CONTROLEE = 0x00,
	UCI_DEVICE_TYPE_CONTROLLER = 0x01,
	/* RFU 0x02 - 0xff */
};

/**
 * enum uci_ranging_round_usage - Ranging Round usage
 *
 * @UCI_OWR_UL_TDOA:
 *     One Way Ranging UL-TDoA.
 * @UCI_SSTWR_DEFERRED:
 *     SS-TWR with Deferred Mode.
 * @UCI_DSTWR_DEFERRED:
 *     DS-TWR with Deferred Mode.
 * @UCI_SSTWR_NON_DEFERRED:
 *     SS-TWR with Non-Deferred mode.
 * @UCI_DSTWR_NON_DEFERRED:
 *     DS-TWR with Non-Deferred mode.
 * @UCI_OWR_DL_TDOA:
 *     One Way Ranging UL-TDoA.
 * @UCI_OWR_AOA:
 *     OWR for AoA measurement.
 * @UCI_ESS_TWR_NON_DEFERRED_CONTENTION_BASED:
 *     eSS-TWR with Non-deferred Mode for Contention-based ranging.
 * @UCI_ADS_TWR_CONTENTION_BASED:
 *     aDS-TWR for Contention-based ranging.
 */
enum uci_ranging_round_usage {
	UCI_OWR_UL_TDOA = 0x00,
	UCI_SSTWR_DEFERRED = 0x01,
	UCI_DSTWR_DEFERRED = 0x02,
	UCI_SSTWR_NON_DEFERRED = 0x03,
	UCI_DSTWR_NON_DEFERRED = 0x04,
	UCI_OWR_DL_TDOA = 0x05,
	UCI_OWR_AOA = 0x06,
	UCI_ESS_TWR_NON_DEFERRED_CONTENTION_BASED = 0x07,
	UCI_ADS_TWR_CONTENTION_BASED = 0x08,
	/* RFU 0x09 - 0xff */
};

/**
 * enum uci_ranging_round_control - Ranging Round Control
 *	b0: Send Ranging Result Report Message (RRRM).
 *	b1: Send Control Message in-band (currently always set).
 *	b2: Skip Ranging Control Phase (RCP) in non-deferred mode.
 *
 * @UCI_RRRM_DISABLED_RCP_ENABLED:
 *	Ranging Result Report Message disabled,
 *	Ranging Control Phase included in non-deferred mode.
 * @UCI_RRRM_ENABLED_RCP_ENABLED:
 *	Ranging Result Report Message enabled,
 *	Ranging Control Phase included in non-deferred mode.
 * @UCI_RRRM_DISABLED_RCP_DISABLED:
 *	Ranging Result Report Message disabled,
 *	Ranging Control Phase skipped in non-deferred mode.
 * @UCI_RRRM_ENABLED_RCP_DISABLED:
 *	Ranging Result Report Message enabled,
 *	Ranging Control Phase skipped in non-deferred mode.
 */
enum uci_ranging_round_control {
	UCI_RRRM_DISABLED_RCP_ENABLED = 0x02,
	UCI_RRRM_ENABLED_RCP_ENABLED = 0x03,
	UCI_RRRM_DISABLED_RCP_DISABLED = 0x06,
	UCI_RRRM_ENABLED_RCP_DISABLED = 0x07,
};

/**
 * enum uci_device_role - Device role.
 *
 * @UCI_DEVICE_ROLE_RESPONDER:
 *     Responder.
 * @UCI_DEVICE_ROLE_INITIATOR:
 *     Initiator.
 * @UCI_DEVICE_ROLE_ADVERTISER:
 *     Advertiser.
 * @UCI_DEVICE_ROLE_OBSERVER:
 *     Observer.
 */
enum uci_device_role {
	UCI_DEVICE_ROLE_RESPONDER = 0x00,
	UCI_DEVICE_ROLE_INITIATOR = 0x01,
	/* RFU 0x02 - 0x04 */
	UCI_DEVICE_ROLE_ADVERTISER = 0x05,
	UCI_DEVICE_ROLE_OBSERVER = 0x06,
	/* RFU 0x07 - 0xff */
};

/**
 * enum uci_test_configuration_parameters - Test configuration.
 *
 * @UCI_TEST_PARAMETER_NUM_PACKETS:
 *     Number of packets.
 * @UCI_TEST_PARAMETER_T_GAP:
 *     T gap.
 * @UCI_TEST_PARAMETER_T_START:
 *     T start.
 * @UCI_TEST_PARAMETER_T_WIN:
 *     T win.
 * @UCI_TEST_PARAMETER_RANDOMIZE_PSDU:
 *     Randomize psdu.
 * @UCI_TEST_PARAMETER_PHR_RANGING_BIT:
 *     PHR ranging bit.
 * @UCI_TEST_PARAMETER_RMARKER_TX_START:
 *     R marker tx start.
 * @UCI_TEST_PARAMETER_RMARKER_RX_START:
 *     R marker rx start.
 * @UCI_TEST_PARAMETER_STS_INDEX_AUTO_INCR:
 *     STS index auto incr.
 * @UCI_TEST_PARAMETER_STS_DETECT_BITMAP_EN:
 *     STS detection bitmap enable.
 * @UCI_TEST_PARAMETER_RSSI_OUTLIERS:
 *     Number of outliers to remove.
 */
enum uci_test_configuration_parameters {
	UCI_TEST_PARAMETER_NUM_PACKETS = 0x00,
	UCI_TEST_PARAMETER_T_GAP = 0x01,
	UCI_TEST_PARAMETER_T_START = 0x02,
	UCI_TEST_PARAMETER_T_WIN = 0x03,
	UCI_TEST_PARAMETER_RANDOMIZE_PSDU = 0x04,
	UCI_TEST_PARAMETER_PHR_RANGING_BIT = 0x05,
	UCI_TEST_PARAMETER_RMARKER_TX_START = 0x06,
	UCI_TEST_PARAMETER_RMARKER_RX_START = 0x07,
	UCI_TEST_PARAMETER_STS_INDEX_AUTO_INCR = 0x08,
	UCI_TEST_PARAMETER_STS_DETECT_BITMAP_EN = 0x09,
	/* RFU 0x09 - 0xff */
	/* Reserved for extension of ID 0xe0 - 0xe4 */
	/* Proprietary 0xe5 - 0xff */
	UCI_TEST_PARAMETER_RSSI_OUTLIERS = 0xeb
};

/**
 * enum uci_controller_action - Controller multicast action.
 *
 * @UCI_CONTROLLER_ACTION_ADD:
 *     Add controlee.
 * @UCI_CONTROLLER_ACTION_DELETE:
 *     Delete controlee.
 * @UCI_CONTROLLER_ACTION_ADD_WITH_SUBKEY16:
 *     Add controlee with 16 bytes subkey.
 * @UCI_CONTROLLER_ACTION_ADD_WITH_SUBKEY32:
 *     Add controlee with 32 bytes subkey.
 */
enum uci_controller_action {
	UCI_CONTROLLER_ACTION_ADD = 0x00,
	UCI_CONTROLLER_ACTION_DELETE = 0x01,
	UCI_CONTROLLER_ACTION_ADD_WITH_SUBKEY16 = 0x02,
	UCI_CONTROLLER_ACTION_ADD_WITH_SUBKEY32 = 0x03,
};

enum uci_device_capability_parameters {
	UCI_CAP_MAX_DATA_MESSAGE_SIZE = 0x00,
	UCI_CAP_MAX_DATA_PACKET_PAYLOAD_SIZE = 0x01,
	UCI_CAP_FIRA_PHY_VERSION_RANGE = 0x02,
	UCI_CAP_FIRA_MAC_VERSION_RANGE = 0x03,
	UCI_CAP_DEVICE_TYPES = 0x04,
	UCI_CAP_DEVICE_ROLES = 0x05,
	UCI_CAP_RANGING_METHOD = 0x06,
	UCI_CAP_STS_CONFIG = 0x07,
	UCI_CAP_MULTI_NODE_MODE = 0x08,
	UCI_CAP_RANGING_TIME_STRUCT = 0x09,
	UCI_CAP_SCHEDULE_MODE = 0x0a,
	UCI_CAP_HOPPING_MODE = 0x0b,
	UCI_CAP_BLOCK_STRIDING = 0x0c,
	UCI_CAP_UWB_INITIATION_TIME = 0x0d,
	UCI_CAP_CHANNELS = 0x0e,
	UCI_CAP_RFRAME_CONFIG = 0x0f,
	UCI_CAP_CC_CONSTRAINT_LENGTH = 0x10,
	UCI_CAP_BPRF_PARAMETER_SETS = 0x11,
	UCI_CAP_HPRF_PARAMETER_SETS = 0x12,
	UCI_CAP_AOA_SUPPORT = 0x13,
	UCI_CAP_EXTENDED_MAC_ADDRESS = 0x14,
	UCI_CAP_SUSPEND_RANGING_SUPPORT = 0x15,
	UCI_CAP_SESSION_KEY_LENGTHS = 0x16,
	UCI_CAP_DT_ANCHOR_MAX_ACTIVE_RR = 0x17,
	UCI_CAP_DT_TAG_MAX_ACTIVE_RR = 0x18,
	UCI_CAP_DT_TAG_BLOCK_SKIPPING = 0x19,
	UCI_CAP_PSDU_LENGTH_SUPPORT = 0x1a,
	UCI_CAP_LL_CAPABILITY_PARAM = 0x1b,
	UCI_CAP_BYPASS_MODE_SUPPORT = 0x1c,
	/* RFU 0x1d - 0x9f */
	/* VENDOR SPECIFIC 0xa0 - 0xdf */
	/* RESERVED FOR EXTENSION OF IDS 0xe0 - 0xe2 */
	/* VENDOR SPECIFIC APP CONFIG 0xe3 - 0xff */
};

/**
 * enum uci_device_capability_aosp - Vendor device capabilities for Android.
 *
 * Up-to-date documentation and definition is available in
 * hardware/uwb/fira_android/UwbVendorCapabilityTlvTypes.aidl.
 *
 * All values are prefixed with AOSP_CAPS compared to their original definition.
 */
enum uci_device_capability_aosp {
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_CHAPS_PER_SLOT: 1 byte bitmask with a list
	 *of supported chaps per slot
	 *
	 * Bitmap of supported values of Slot durations as a multiple of TChap,
	 * NChap_per_Slot as defined in CCC Specification.
	 * Each “1” in this bit map corresponds to a specific
	 * value of NChap_per_Slot where:
	 * 0x01 = “3”,
	 * 0x02 = “4”,
	 * 0x04= “6”,
	 * 0x08 =“8”,
	 * 0x10 =“9”,
	 * 0x20 = “12”,
	 * 0x40 = “24”,
	 * 0x80 is reserved.
	 */
	AOSP_CAPS_CCC_SUPPORTED_CHAPS_PER_SLOT = 0xa0,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_SYNC_CODES: 4 byte bitmask with a list of
	 *		supported sync codes.
	 *
	 * Bitmap of SYNC code indices that can be used.
	 * The position of each “1” in this bit pattern
	 * corresponds to the index of a SYNC code that
	 * can be used, where:
	 * 0x00000001 = “1”,
	 * 0x00000002 = “2”,
	 * 0x00000004 = “3”,
	 * 0x00000008 = “4”,
	 * ….
	 * 0x40000000 = “31”,
	 * 0x80000000 = “32”
	 * Refer to IEEE 802.15.4-2015 and CCC
	 * Specification for SYNC code index definition
	 */
	AOSP_CAPS_CCC_SUPPORTED_SYNC_CODES = 0xa1,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_HOPPING_CONFIG_MODES_AND_SEQUENCES: 1 byte
	 * bitmask with a list of supported hopping config modes
	 * and sequences.
	 *
	 * [b7 b6 b5] : bitmask of hopping modes the
	 * device offers to use in the ranging session
	 * 100 - No Hopping
	 * 010 - Continuous Hopping
	 * 001 - Adaptive Hopping
	 * [b4 b3 b2 b1 b0] : bit mask of hopping
	 * sequences the device offers to use in the
	 * ranging session
	 * b4=1 is always set because of the default
	 * hopping sequence. Support for it is mandatory.
	 * b3=1 is set when the optional AES based
	 * hopping sequence is supported.
	 */
	AOSP_CAPS_CCC_SUPPORTED_HOPPING_CONFIG_MODES_AND_SEQUENCES = 0xa2,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_CHANNELS: 1 byte bitmask with list of
	 *supported channels.
	 *
	 * Bitmap of supported UWB channels. Each “1” in
	 * this bit map corresponds to a specific value of
	 * UWB channel where:
	 * 0x01 = "Channel 5"
	 * 0x02 = "Channel 9"
	 */
	AOSP_CAPS_CCC_SUPPORTED_CHANNELS = 0xa3,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_VERSIONS: Supported CCC version.
	 *
	 * 2 byte tuple {major_version (1 byte), minor_version (1 byte)} array
	 * with list of supported CCC versions
	 */
	AOSP_CAPS_CCC_SUPPORTED_VERSIONS = 0xa4,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_UWB_CONFIGS: byte array with a list of
	 *supported UWB configs.
	 *
	 * UWB configurations are define in chapter
	 * "21.4 UWB Frame Elements" of the CCC
	 * specification. Configuration 0x0000 is
	 * mandatory for device and vehicle, configuration
	 * 0x0001 is mandatory for the device, optional for
	 * the vehicle.
	 */
	AOSP_CAPS_CCC_SUPPORTED_UWB_CONFIGS = 0xa5,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_PULSE_SHAPE_COMBOS: supported CCC pulse
	 *shape combo.
	 *
	 * 1 byte tuple {initiator_tx (4 left-most bits), responder_tx (4 right-most bits)} array
	 *with list of supported pulse shape combos
	 *
	 * Values:
	 *  PULSE_SHAPE_SYMMETRICAL_ROOT_RAISED_COSINE = 0
	 *  PULSE_SHAPE_PRECURSOR_FREE = 1
	 *  PULSE_SHAPE_PRECURSOR_FREE_SPECIAL = 2
	 */
	AOSP_CAPS_CCC_SUPPORTED_PULSE_SHAPE_COMBOS = 0xa6,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_RAN_MULTIPLIER: Int value for indicating
	 *		supported ran multiplier.
	 */
	AOSP_CAPS_CCC_SUPPORTED_RAN_MULTIPLIER = 0xa7,
	/**
	 * @AOSP_CAPS_CCC_SUPPORTED_MAX_RANGING_SESSION_NUMBER: 4 byte value to
	 * 		indicate the maximum number of CCC ranging sessions
	 * 		supported.
	 */
	AOSP_CAPS_CCC_SUPPORTED_MAX_RANGING_SESSION_NUMBER = 0xa8,
	/**
	 * @AOSP_CAPS_SUPPORTED_POWER_STATS_QUERY: byte value for indicating
	 *		supported (1 == supported)
	 */
	AOSP_CAPS_SUPPORTED_POWER_STATS_QUERY = 0xc0,
	/**
	 * @AOSP_CAPS_SUPPORTED_AOA_RESULT_REQ_ANTENNA_INTERLEAVING: 1 byte
	 *value to indicate support for antenna interleaving feature.
	 *
	 * Values:
	 *  1 - Feature supported.
	 *  0 - Feature not supported.
	 */
	AOSP_CAPS_SUPPORTED_AOA_RESULT_REQ_ANTENNA_INTERLEAVING = 0xe3,
	/**
	 * @AOSP_CAPS_SUPPORTED_SESSION_INFO_NTF_CONFIG: 4 bytes bitmask to
	 *		indicate the supported SESSION_INFO_NTF_CONFIG values.
	 */
	AOSP_CAPS_SUPPORTED_SESSION_INFO_NTF_CONFIG = 0xe5,
	/**
	 * @AOSP_CAPS_SUPPORTED_MIN_RANGING_INTERVAL_MS: 4 byte value to
	 * 		indicate the min ranging interval supported in ms .
	 */
	AOSP_CAPS_SUPPORTED_MIN_RANGING_INTERVAL_MS = 0xe4,
	/**
	 * @AOSP_CAPS_SUPPORTED_RSSI_REPORTING: 1 byte bitmask to
	 *		indicate the supported RSSI_REPORTING values.
	 *
	 * Values:
	 *  1 - Feature supported.
	 *  0 - Feature not supported.
	 */
	AOSP_CAPS_SUPPORTED_RSSI_REPORTING = 0xe6,
	/**
	 * @AOSP_CAPS_SUPPORTED_DIAGNOSTICS: 1 byte value to indicate support
	 *		for diagnostics feature.
	 * Values:
	 *  1 - Feature supported.
	 *  0 - Feature not supported.
	 */
	AOSP_CAPS_SUPPORTED_DIAGNOSTICS = 0xe7,
	/**
	 * @AOSP_CAPS_SUPPORTED_MIN_SLOT_DURATION_RSTU: 4 byte value to indicate
	 * 		the min slot duration supported in RSTU.
	 */
	AOSP_CAPS_SUPPORTED_MIN_SLOT_DURATION_RSTU = 0xe8,
	/**
	 * @AOSP_CAPS_SUPPORTED_MAX_RANGING_SESSION_NUMBER: 4 byte value to
	 * 		indicate the maximum number of FiRa ranging sessions
	 * 		supported.
	 */
	AOSP_CAPS_SUPPORTED_MAX_RANGING_SESSION_NUMBER = 0xe9,
};

/**
 * enum uci_diagnostic_notif_frame_report_field - Report fields included
 * in vendor specific Range Diagnostics Notification.
 *
 * @UCI_DIAG_REPORT_AOAS: AoAs report field.
 * @UCI_DIAG_REPORT_EXTRA_STATUS: Extra Status report field.
 * @UCI_DIAG_REPORT_CFO_Q26: CFO report field.
 * @UCI_DIAG_REPORT_EMITTER_SHORT_ADDR: Emitter short address report field.
 * @UCI_DIAG_REPORT_SEGMENT_METRICS: Segment metrics report field.
 * @UCI_DIAG_REPORT_CIRS: CIRs report field.
 *
 */
enum uci_diagnostic_notif_frame_report_field {
	/* Flag 0x0 is deprecated. */
	UCI_DIAG_REPORT_AOAS = 0x01,
	/* Flag 0x2 is deprecated. */
	UCI_DIAG_REPORT_EXTRA_STATUS = 0x03,
	UCI_DIAG_REPORT_CFO_Q26 = 0x04,
	UCI_DIAG_REPORT_EMITTER_SHORT_ADDR = 0x05,
	UCI_DIAG_REPORT_SEGMENT_METRICS = 0x06,
	UCI_DIAG_REPORT_CIRS = 0x07,
};
