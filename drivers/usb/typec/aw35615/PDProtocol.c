// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author	: awinic
 * Date		  : 2021-09-10
 * Description   : .C file function description
 * Version	   : 1.0
 * Function List :
 ******************************************************************************/
#include "platform_helpers.h"
#include "aw35615_driver.h"
#include "TypeC.h"
#include "PDPolicy.h"
#include "PDProtocol.h"
#include "PD_Types.h"

#ifdef AW_HAVE_VDM
#include "vdm/vdm_callbacks.h"
#include "vdm/vdm_callbacks_defs.h"
#include "vdm/vdm.h"
#include "vdm/vdm_types.h"
#include "vdm/bitfield_translators.h"
#endif /* AW_HAVE_VDM */

/* USB PD Protocol Layer Routines */
void USBPDProtocol(Port_t *port)
{
	if (port->Registers.Status.I_HARDRST || port->Registers.Status.I_HARDSENT) {

		ResetProtocolLayer(port, AW_TRUE);
		if (port->PolicyIsSource) {
			TimerStart(&port->PolicyStateTimer, tPSHardReset);
			SetPEState(port, peSourceTransitionDefault);
		} else {
			if (port->get_sink_cap_flag) {
				port->Registers.Switches.AUTO_CRC = 0;
				DeviceWrite(port, regSwitches1, 1,
						&port->Registers.Switches.byte[1]);
			}
			SetPEState(port, peSinkTransitionDefault);
		}
	} else {
		/* AW_LOG("ProtocolState %d\n", port->ProtocolState); */
		switch (port->ProtocolState) {
		case PRLReset:
			/* Sending a hard reset. */
			ProtocolSendHardReset(port);
			port->PDTxStatus = txWait;
			port->ProtocolState = PRLResetWait;
			break;
		case PRLResetWait:
			/* Wait on hard reset signaling */
			ProtocolResetWait(port);
			break;
		case PRLIdle:
			/* Wait on Tx/Rx */
			ProtocolIdle(port);
			break;
		case PRLTxSendingMessage:
			/* Wait on Tx to finish */
			ProtocolSendingMessage(port);
			break;
		case PRLTxVerifyGoodCRC:
			/* Verify returned GoodCRC */
			ProtocolVerifyGoodCRC(port);
			break;
		case PRLDisabled:
			break;
		default:
			break;
		}
	}
}

void ProtocolSendCableReset(Port_t *port)
{
	port->ProtocolTxBytes = 0;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = RESET1;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = RESET1;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC3_TOKEN;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = TXOFF;
	DeviceWrite(port, regFIFO, port->ProtocolTxBytes, &port->ProtocolTxBuffer[0]);

	port->Registers.Control.TX_START = 1;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	port->Registers.Control.TX_START = 0;
	port->MessageIDCounter[port->ProtocolMsgTxSop] = 0;
	port->MessageID[port->ProtocolMsgTxSop] = 0xFF;
}

void ProtocolIdle(Port_t *port)
{
	if (port->PDTxStatus == txReset) {
		port->ProtocolState = PRLReset;/* Need to send a reset? */
	} else if (port->ProtocolMsgRxPending || port->Registers.Status.I_GCRCSENT) {
	/* else if (port->Registers.Status.I_CRC_CHK || port->ProtocolMsgRxPending || */
    /* port->Registers.Status.I_GCRCSENT) */
		/* Using the CRC CHK interrupt as part of the GSCE workaround. */
		if (port->DoTxFlush) {
			ProtocolFlushTxFIFO(port);
			port->DoTxFlush = AW_FALSE;
		}

		if (!port->ProtocolMsgRx) {
			/* Only read in a new msg if there isn't already one waiting */
			/* to be processed by the PE. */
			ProtocolGetRxPacket(port, AW_FALSE);
		} else {
			/* Otherwise make a note to handle it later. */
			port->ProtocolMsgRxPending = AW_TRUE;
		}

		//port->Registers.Status.I_CRC_CHK = 0;
		port->Registers.Status.I_GCRCSENT = 0;
	} else if (port->PDTxStatus == txSend) {
		/* Have a message to send? */
		if (port->ProtocolMsgRx || port->ProtocolMsgRxPending)
			port->PDTxStatus = txAbort;
		else
			ProtocolTransmitMessage(port);
	}
#ifdef AW_HAVE_EXT_MSG
	else if (port->ExtTxOrRx != NoXfer && port->ExtWaitTxRx == AW_FALSE)
		port->PDTxStatus = txSend;
#endif /* AW_HAVE_EXT_MSG */
}

void ProtocolResetWait(Port_t *port)
{
	if (port->Registers.Status.I_HARDSENT) {
		port->ProtocolState = PRLIdle;
		port->PDTxStatus = txSuccess;
	}
}

void ProtocolGetRxPacket(Port_t *port, AW_BOOL HeaderReceived)
{
	/* AW_LOG("enter\n"); */
	AW_U32 i = 0, j = 0;
	AW_U8 data[4];

	port->ProtocolMsgRxPending = AW_FALSE;

	TimerDisable(&port->ProtocolTimer);

	/* Update to make sure GetRxPacket can see a valid RxEmpty value */
	DeviceRead(port, regStatus1, 1, &port->Registers.Status.byte[5]);

	/* AW_LOG("regStatus1 = 0x%x\n", port->Registers.Status.byte[5]); */
	if (port->Registers.Status.RX_EMPTY == 1)
		return;/* Nothing to see here... */

	if (HeaderReceived == AW_FALSE) {
		/* Read the Rx token and two header bytes */
		DeviceRead(port, regFIFO, 3, &data[0]);
		port->PolicyRxHeader.byte[0] = data[1];
		port->PolicyRxHeader.byte[1] = data[2];

		/* Figure out what SOP* the data came in on */
		port->ProtocolMsgRxSop = TokenToSopType(data[0]);
	} else {
		/* PolicyRxHeader, ProtocolMsgRxSop already set */
	}

#ifdef AW_HAVE_MANUAL_CRC
	/* This is a manual reply to the CRC message */
	/* Pre-load manual GoodCRC */

	port->PolicyTxHeader.word = 0;
	port->PolicyTxHeader.MessageType = CMTGoodCRC;
	port->PolicyTxHeader.PortDataRole = port->PolicyIsDFP;
	port->PolicyTxHeader.PortPowerRole = port->PolicyIsSource;
	if (port->ProtocolMsgRxSop == SOP_TYPE_SOP1 || port->ProtocolMsgRxSop == SOP_TYPE_SOP2)
		port->PolicyTxHeader.PortPowerRole = 0;
	port->PolicyTxHeader.SpecRevision = port->PolicyRxHeader.SpecRevision;
	port->PolicyTxHeader.MessageID = port->PolicyRxHeader.MessageID;
	ProtocolSendGoodCRC(port, port->ProtocolMsgRxSop);

	/* Transmit pre-loaded GoodCRC msg */
	port->Registers.Control.TX_START = 1;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	port->Registers.Control.TX_START = 0;

	/* Brief delay to allow the GoodCRC to transmit */
	/* platform_delay_10us(10); */

	/* Flush any retries that may have made it into the buffer */
	/* ProtocolFlushRxFIFO(port); */
#endif /* AW_HAVE_MANUAL_CRC */

	if (port->ProtocolMsgRxSop == SOP_TYPE_ERROR) {
		/* SOP result can be corrupted in rare cases, or possibly if */
		/* the FIFO reads get out of sync. */
		/* TODO - Flush? */
		return;
	}

	if ((port->PolicyRxHeader.NumDataObjects == 0) &&
		(port->PolicyRxHeader.MessageType == CMTSoftReset)) {
		/* For a soft reset, reset ID's, etc. */
		port->MessageIDCounter[port->ProtocolMsgRxSop] = 0;
		port->MessageID[port->ProtocolMsgRxSop] = 0xFF;
		port->ProtocolMsgRx = AW_TRUE;
	} else if (port->PolicyRxHeader.MessageID != port->MessageID[port->ProtocolMsgRxSop]) {
		port->MessageID[port->ProtocolMsgRxSop] = port->PolicyRxHeader.MessageID;
		port->ProtocolMsgRx = AW_TRUE;
	}

	/* AW_LOG("num = %d Type =%d\n", port->PolicyRxHeader.NumDataObjects, */
	/* port->PolicyRxHeader.MessageType); */
	if ((port->PolicyRxHeader.NumDataObjects == 0) &&
		(port->PolicyRxHeader.MessageType == CMTGoodCRC)) {
		/* Rare cases may result in the next GoodCRC being processed before */
		/* the expected current message.  Handle and continue on to next msg. */

		/* Read out the 4 CRC bytes to move the address to the next packet */
		DeviceRead(port, regFIFO, 4, data);

		port->ProtocolState = PRLIdle;
		port->PDTxStatus = txSuccess;
		port->ProtocolMsgRx = AW_FALSE;

		ProtocolGetRxPacket(port, AW_FALSE);

		return;
	} else if (port->PolicyRxHeader.NumDataObjects > 0) {
		/* Data message - Grab the data from the FIFO, including 4 byte CRC */
		DeviceRead(port, regFIFO, ((port->PolicyRxHeader.NumDataObjects << 2) + 4),
				&port->ProtocolRxBuffer[0]);

#ifdef AW_HAVE_EXT_MSG
		if (port->PolicyRxHeader.Extended == 1) {
			/* Copy ext header first */
			port->ExtRxHeader.byte[0] = port->ProtocolRxBuffer[0];
			port->ExtRxHeader.byte[1] = port->ProtocolRxBuffer[1];

			if (port->ExtRxHeader.ReqChunk == 1) {
				/* Received request for another chunk. Continue sending....*/
				port->ExtWaitTxRx = AW_FALSE;
				if (port->ExtRxHeader.ChunkNum < port->ExtChunkNum) {
					/* Resend the previous chunk */
					port->ExtChunkNum = port->ExtRxHeader.ChunkNum;
					port->ExtChunkOffset =
						port->ExtChunkNum * EXT_MAX_MSG_LEGACY_LEN;
				}
			} else {
				if (port->ExtRxHeader.DataSize > EXT_MAX_MSG_LEGACY_LEN) {
					if (port->ExtRxHeader.DataSize > 260)
						port->ExtRxHeader.DataSize = 260;
					if (port->ExtRxHeader.ChunkNum == 0) {
						port->ExtChunkOffset = 0;   /* First message */
						port->ExtChunkNum = 1;	  /* Next chunk number */
					}
				}
			}
		}
#endif /* AW_HAVE_EXT_MSG */
		for (i = 0; i < port->PolicyRxHeader.NumDataObjects; i++) {
			for (j = 0; j < 4; j++) {
#ifdef AW_HAVE_EXT_MSG
				if (port->PolicyRxHeader.Extended == 1) {
					/* Skip ext header */
					if (i == 0 && (j == 0 || j == 1))
						continue;

					if (port->ExtRxHeader.ReqChunk == 0)
						port->ExtMsgBuffer[port->ExtChunkOffset++] =
							port->ProtocolRxBuffer[j + (i << 2)];
				} else
#endif /* AW_HAVE_EXT_MSG */
				{
					port->PolicyRxDataObj[i].byte[j] =
							port->ProtocolRxBuffer[j + (i << 2)];
					//AW_LOG("data = 0x%x\n", port->PolicyRxDataObj[i].byte[j]);
				}
			}
	}

#ifdef AW_HAVE_EXT_MSG
		if (port->PolicyRxHeader.Extended == 1) {
			if (port->ExtRxHeader.ReqChunk == 0) {
				if (port->ExtChunkOffset < port->ExtRxHeader.DataSize) {
					/* more message left. continue receiving */
					port->ExtTxOrRx = RXing;
					port->ProtocolMsgRx = AW_FALSE;
					port->ExtWaitTxRx = AW_FALSE;
				} else {
					port->ExtTxOrRx = NoXfer;
					port->ProtocolMsgRx = AW_TRUE;
				}
			} else if (port->ExtRxHeader.ReqChunk == 1 &&
					port->ExtChunkOffset < port->ExtTxHeader.DataSize) {
				port->ExtWaitTxRx = AW_FALSE;
			} else {
				/* Last message received */
				port->ExtTxOrRx = NoXfer;
			}
		}
#endif /* AW_HAVE_EXT_MSG */
	} else {
		/* Command message */
		/* Read out the 4 CRC bytes to move the address to the next packet */
		DeviceRead(port, regFIFO, 4, data);
	}
	/* Special VDM use case where a second message appears too quickly */
	if ((port->PolicyRxHeader.NumDataObjects != 0) &&
		(port->PolicyRxHeader.MessageType == DMTVenderDefined) &&
		(port->PolicyRxDataObj[0].SVDM.CommandType == 0)) /* Initiator */ {
		/* Delay and check if a new mesage has been received */
		/* Note: May need to increase this delay (2-3ms) or find alternate
		 * method for some slow systems - e.g. Android.
		 */
		platform_delay_10us(100); /* 1ms */

		DeviceRead(port, regInterruptb, 3,
				   &port->Registers.Status.byte[3]);

		if (port->Registers.Status.I_GCRCSENT && !port->Registers.Status.RX_EMPTY) {
			/* Get the next message - overwriting the current message */
			ProtocolGetRxPacket(port, AW_FALSE);
		}
	} else {
		/* A quickly sent second message can be received
		 * into the buffer without triggering an (additional) interrupt.
		 */
		DeviceRead(port, regStatus0, 2, &port->Registers.Status.byte[4]);

		if (!port->Registers.Status.ACTIVITY && !port->Registers.Status.RX_EMPTY) {
			platform_delay_10us(50);
			DeviceRead(port, regStatus0, 2, &port->Registers.Status.byte[4]);
			if (!port->Registers.Status.ACTIVITY && !port->Registers.Status.RX_EMPTY)
				port->ProtocolMsgRxPending = AW_TRUE;
		}
	}

	/* If a message has been received during an attempt to transmit,
	 * abort and handle the received message before trying again.
	 */
	if (port->ProtocolMsgRx && (port->PDTxStatus == txSend))
		port->PDTxStatus = txAbort;
}

void ProtocolTransmitMessage(Port_t *port)
{
	AW_U32 i, j;
	sopMainHeader_t temp_PolicyTxHeader = { 0 };

	port->DoTxFlush = AW_FALSE;

	/* AW_LOG("enter\n"); */

	/* Note: Power needs to be set a bit before we write TX_START to update */
	ProtocolLoadSOP(port, port->ProtocolMsgTxSop);

#ifdef AW_HAVE_EXT_MSG
	if (port->ExtTxOrRx == RXing) {
		/* Set up chunk request */
		temp_PolicyTxHeader.word = port->PolicyRxHeader.word;
		temp_PolicyTxHeader.PortPowerRole = port->PolicyIsSource;
		temp_PolicyTxHeader.PortDataRole = port->PolicyIsDFP;
	} else
#endif /* AW_HAVE_EXT_MSG */
		temp_PolicyTxHeader.word = port->PolicyTxHeader.word;

	if ((temp_PolicyTxHeader.NumDataObjects == 0) &&
		(temp_PolicyTxHeader.MessageType == CMTSoftReset)) {
		port->MessageIDCounter[port->ProtocolMsgTxSop] = 0;
		port->MessageID[port->ProtocolMsgTxSop] = 0xFF;
	}

#ifdef AW_HAVE_EXT_MSG
	if (temp_PolicyTxHeader.Extended == 1) {
		if (port->ExtTxOrRx == TXing) {
			/* Remaining bytes */
			i = port->ExtTxHeader.DataSize - port->ExtChunkOffset;

			if (i > EXT_MAX_MSG_LEGACY_LEN) {
				temp_PolicyTxHeader.NumDataObjects = 7;
			} else {
				/* Round up to 4 byte boundary.
				 * Two extra byte is for the extended header.
				 */
				temp_PolicyTxHeader.NumDataObjects = (i + 4) / 4;
			}
			port->PolicyTxHeader.NumDataObjects = temp_PolicyTxHeader.NumDataObjects;
			port->ExtTxHeader.ChunkNum = port->ExtChunkNum;
		} else if (port->ExtTxOrRx == RXing) {
			temp_PolicyTxHeader.NumDataObjects = 1;
		}
		port->ExtWaitTxRx = AW_TRUE;
	}
#endif /* AW_HAVE_EXT_MSG */

	temp_PolicyTxHeader.MessageID = port->MessageIDCounter[port->ProtocolMsgTxSop];
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
			PACKSYM | (2 + (temp_PolicyTxHeader.NumDataObjects << 2));
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
			temp_PolicyTxHeader.byte[0];
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
			temp_PolicyTxHeader.byte[1];

	/* If this is a data object... */
	if (temp_PolicyTxHeader.NumDataObjects > 0) {
#ifdef AW_HAVE_EXT_MSG
		if (temp_PolicyTxHeader.Extended == 1) {
			if (port->ExtTxOrRx == RXing) {
				port->ExtTxHeader.ChunkNum = port->ExtChunkNum;
				port->ExtTxHeader.DataSize = 0;
				port->ExtTxHeader.Chunked = 1;
				port->ExtTxHeader.ReqChunk = 1;
			} else if (port->ExtTxOrRx == TXing) {
				port->ExtTxHeader.ChunkNum = port->ExtChunkNum;
			}

			/* Copy the two byte extended header. */
			port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
							port->ExtTxHeader.byte[0];
			port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
							port->ExtTxHeader.byte[1];
		}
#endif /* AW_HAVE_EXT_MSG */
		for (i = 0; i < temp_PolicyTxHeader.NumDataObjects; i++) {
			for (j = 0; j < 4; j++) {
#ifdef AW_HAVE_EXT_MSG
				if (temp_PolicyTxHeader.Extended == 1) {
					/* Skip extended header */
					if (i == 0 && (j == 0 || j == 1))
						continue;
					if (port->ExtChunkOffset < port->ExtTxHeader.DataSize)
						port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
							port->ExtMsgBuffer[port->ExtChunkOffset++];
					else
						port->ProtocolTxBuffer[port->ProtocolTxBytes++] = 0;
				} else
#endif /* AW_HAVE_EXT_MSG */
				{
					/* Load the actual bytes */
					port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
							port->PolicyTxDataObj[i].byte[j];
				}
			}
		}
	}

	/* Load the CRC, EOP and stop sequence */
	ProtocolLoadEOP(port);

	/* Commit the FIFO to the device */
	if (DeviceWrite(port, regFIFO, port->ProtocolTxBytes,
					&port->ProtocolTxBuffer[0]) == AW_FALSE) {
		/* If a FIFO write happens while a GoodCRC is being transmitted, */
		/* the transaction will NAK and will need to be discarded. */
		port->DoTxFlush = AW_TRUE;
		port->PDTxStatus = txAbort;
		return;
	}

	port->Registers.Control.N_RETRIES = DPM_Retries(port, port->ProtocolMsgTxSop);
	port->Registers.Control.AUTO_RETRY = 1;

	DeviceWrite(port, regControl3, 1, &port->Registers.Control.byte[3]);
	port->Registers.Control.TX_START = 1;
	DeviceWrite(port, regControl0, 1, &port->Registers.Control.byte[0]);
	port->Registers.Control.TX_START = 0;

	/* Set the transmitter status to busy */
	port->PDTxStatus = txBusy;
	port->ProtocolState = PRLTxSendingMessage;

	/* Timeout specifically for chunked messages, but used with each transmit */
	/* to prevent a theoretical protocol hang. */

	TimerStart(&port->ProtocolTimer, tChunkSenderRequest);
}

void ProtocolSendingMessage(Port_t *port)
{

	/* Waiting on result/status of transmission */
	if (port->Registers.Status.I_TXSENT || port->Registers.Status.I_CRC_CHK) {
		port->Registers.Status.I_TXSENT = 0;
		port->Registers.Status.I_CRC_CHK = 0;
		ProtocolVerifyGoodCRC(port);
	} else if (port->Registers.Status.I_COLLISION) {
		port->Registers.Status.I_COLLISION = 0;
		port->PDTxStatus = txCollision;
		port->ProtocolState = PRLIdle;
	} else if (port->Registers.Status.I_RETRYFAIL) {
		port->Registers.Status.I_RETRYFAIL = 0;
		port->PDTxStatus = txError;
		port->ProtocolState = PRLIdle;
	} else if (port->Registers.Status.I_GCRCSENT) {
		/* Interruption */
		port->PDTxStatus = txError;
		port->ProtocolState = PRLIdle;
		port->ProtocolMsgRxPending = AW_TRUE;
		port->Registers.Status.I_GCRCSENT = 0;
	}

	/* Make an additional check for missed/pending message data */
	if (port->ProtocolState == PRLIdle)
		ProtocolIdle(port);
}

void ProtocolVerifyGoodCRC(Port_t *port)
{
	/* AW_LOG("enter\n"); */
	AW_U8 data[4];
	sopMainHeader_t header;
	SopType sop;

	/* Read the Rx token and two header bytes */
	DeviceRead(port, regFIFO, 3, &data[0]);
	header.byte[0] = data[1];
	header.byte[1] = data[2];

	/* Figure out what SOP* the data came in on */
	sop = TokenToSopType(data[0]);

	if ((header.NumDataObjects == 0) && (header.MessageType == CMTGoodCRC)) {
		AW_U8 MIDcompare;

		if (sop == SOP_TYPE_ERROR)
			MIDcompare = 0xFF;
		else
			MIDcompare = port->MessageIDCounter[sop];

		if (header.MessageID != MIDcompare) {
			/* Read out the 4 CRC bytes to move the addr to the next packet */
			DeviceRead(port, regFIFO, 4, data);
			port->PDTxStatus = txError;
			port->ProtocolState = PRLIdle;
		} else {
			if (sop != SOP_TYPE_ERROR) {
				/* Increment and roll over */
				port->MessageIDCounter[sop]++;
				port->MessageIDCounter[sop] &= 0x07;
#ifdef AW_HAVE_EXT_MSG
				if (port->ExtTxOrRx != NoXfer) {
					if (port->ExtChunkOffset >= port->ExtTxHeader.DataSize) {
						/* All data has been sent */
						port->ExtTxOrRx = NoXfer;
					}
					port->ExtChunkNum++;
				}
#endif /* AW_HAVE_EXT_MSG */
			}
			port->ProtocolState = PRLIdle;
			port->PDTxStatus = txSuccess;

			/* Read out the 4 CRC bytes to move the addr to the next packet */
			DeviceRead(port, regFIFO, 4, data);
		}
	} else {
		port->ProtocolState = PRLIdle;
		port->PDTxStatus = txError;

		/* Pass header and SOP* on to GetRxPacket */
		port->PolicyRxHeader.word = header.word;
		port->ProtocolMsgRxSop = sop;

		/* Rare case, next received message preempts GoodCRC */
		ProtocolGetRxPacket(port, AW_TRUE);
	}
}

static AW_U8 g_trim_osc = 1;
static AW_U8 temp_reg_e9;
static AW_BOOL flag_is_need_trim_osc = AW_FALSE;

void ProtocolTrimOscFun(Port_t *port)
{
	AW_U8 buf, regE9_buf, regEA_buf;
	AW_BOOL reg_E9_BIT_7;
	AW_BOOL reg_EA_BIT_0;

	if (g_trim_osc) {
		DeviceRead(port, 0xE9, 1, &regE9_buf);
		reg_E9_BIT_7 = (regE9_buf & 0x80 ? AW_TRUE : AW_FALSE);

		DeviceRead(port, 0xEA, 1, &regEA_buf);
		reg_EA_BIT_0 = (regEA_buf & 0x01 ? AW_TRUE : AW_FALSE);

		DeviceRead(port, 0x01, 1, &buf);
		if ((buf == 0x91) && (reg_E9_BIT_7 == AW_FALSE) && (reg_EA_BIT_0 == AW_FALSE)) {
			flag_is_need_trim_osc = AW_TRUE;
			DeviceRead(port, 0xe9, 1, &buf);
			temp_reg_e9 = (buf & 0xF0) | ((buf + 2) & 0x0F);
		}
		g_trim_osc = 0;
	}

	if (flag_is_need_trim_osc) {
		buf = 0xc2;
		DeviceWrite(port, 0xf9, 1, &buf);

		buf = 0x0F;
		DeviceWrite(port, 0x0B, 1, &buf);

		buf = 0x01;
		DeviceWrite(port, 0xe2, 1, &buf);

		buf = temp_reg_e9;
		DeviceWrite(port, 0xF0, 1, &buf);

		buf = 0x07;
		DeviceWrite(port, 0xE8, 1, &buf);

		buf = 0x00;
		DeviceWrite(port, 0xE2, 1, &buf);

		buf = 0x00;
		DeviceWrite(port, 0xF9, 1, &buf);

		buf = 0x01;
		DeviceWrite(port, 0x0B, 1, &buf);
	}
}

SopType DecodeSopFromPdMsg(AW_U8 msg0, AW_U8 msg1)
{
	/* this SOP* decoding is based on AW35615 GUI: */
	/* SOP   => 0b 0xx1 xxxx xxx0 xxxx */
	/* SOP'  => 0b 1xx1 xxxx xxx0 xxxx */
	/* SOP'' => 0b 1xx1 xxxx xxx1 xxxx */
	if (((msg1 & 0x90) == 0x10) && ((msg0 & 0x10) == 0x00))
		return SOP_TYPE_SOP;
	else if (((msg1 & 0x90) == 0x90) && ((msg0 & 0x10) == 0x00))
		return SOP_TYPE_SOP1;
	else if (((msg1 & 0x90) == 0x90) && ((msg0 & 0x10) == 0x10))
		return SOP_TYPE_SOP2;
	else
		return SOP_TYPE_SOP;
}

void ProtocolSendGoodCRC(Port_t *port, SopType sop)
{
	ProtocolLoadSOP(port, sop);

	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = PACKSYM | 0x02;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
					port->PolicyTxHeader.byte[0];
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] =
					port->PolicyTxHeader.byte[1];
	ProtocolLoadEOP(port);
	DeviceWrite(port, regFIFO, port->ProtocolTxBytes,
					&port->ProtocolTxBuffer[0]);
}

void ProtocolLoadSOP(Port_t *port, SopType sop)
{
	/* Clear the Tx byte counter */
	port->ProtocolTxBytes = 0;

	switch (sop) {
	case SOP_TYPE_SOP1:
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC3_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC3_TOKEN;
		break;
	case SOP_TYPE_SOP2:
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC3_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC3_TOKEN;
		break;
	case SOP_TYPE_SOP:
	default:
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC1_TOKEN;
		port->ProtocolTxBuffer[port->ProtocolTxBytes++] = SYNC2_TOKEN;
		break;
	}
}

void ProtocolLoadEOP(Port_t *port)
{
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = JAM_CRC;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = EOP;
	port->ProtocolTxBuffer[port->ProtocolTxBytes++] = TXOFF;
}

void ProtocolSendHardReset(Port_t *port)
{
	AW_U8 data = port->Registers.Control.byte[3] | 0x40;  /* Hard Reset bit */

	/* If the shortcut flag is set, we've already sent the HR command */
	if (port->WaitingOnHR)
		port->WaitingOnHR = AW_FALSE;
	else
		DeviceWrite(port, regControl3, 1, &data);
}

void ProtocolFlushRxFIFO(Port_t *port)
{
	AW_U8 data = port->Registers.Control.byte[1] | 0x04;  /* RX_FLUSH bit */

	DeviceWrite(port, regControl1, 1, &data);
}

void ProtocolFlushTxFIFO(Port_t *port)
{
	AW_U8 data = port->Registers.Control.byte[0] | 0x40;  /* TX_FLUSH bit */

	DeviceWrite(port, regControl0, 1, &data);
}

void ResetProtocolLayer(Port_t *port, AW_BOOL ResetPDLogic)
{
	AW_U32 i;
	AW_U8 data = 0x02; /* PD_RESET bit */

	if (ResetPDLogic)
		DeviceWrite(port, regReset, 1, &data);

	port->ProtocolState = PRLIdle;
	port->PDTxStatus = txIdle;

	port->WaitingOnHR = AW_FALSE;

#ifdef AW_HAVE_VDM
	TimerDisable(&port->VdmTimer);
	port->VdmTimerStarted = AW_FALSE;
#endif /* AW_HAVE_VDM */

	port->ProtocolTxBytes = 0;

	if (!port->no_clear_message) {
		for (i = 0; i < SOP_TYPE_NUM; i++) {
			port->MessageIDCounter[i] = 0;
			port->MessageID[i] = 0xFF;
		}
	} else {
		port->no_clear_message = AW_FALSE;
	}

	port->ProtocolMsgRx = AW_FALSE;
	port->ProtocolMsgRxSop = SOP_TYPE_SOP;
	port->ProtocolMsgRxPending = AW_FALSE;
	port->USBPDTxFlag = AW_FALSE;
	port->PolicyHasContract = AW_FALSE;
	port->USBPDContract.object = 0;

	port->SrcCapsHeaderReceived.word = 0;
	port->SnkCapsHeaderReceived.word = 0;
	for (i = 0; i < 7; i++) {
		port->SrcCapsReceived[i].object = 0;
		port->SnkCapsReceived[i].object = 0;
	}

#ifdef AW_HAVE_EXT_MSG
	port->ExtWaitTxRx = AW_FALSE;
	port->ExtChunkNum = 0;
	port->ExtTxOrRx = NoXfer;
	port->ExtChunkOffset = 0;
#endif /* AW_HAVE_EXT_MSG */
}
