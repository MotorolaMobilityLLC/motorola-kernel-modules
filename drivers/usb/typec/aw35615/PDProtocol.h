/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef _PDPROTOCOL_H_
#define _PDPROTOCOL_H_
#include "Port.h"

/* USB PD Protocol Layer Routines */
void USBPDProtocol(Port_t *port);
void ProtocolSendCableReset(Port_t *port);
void ProtocolIdle(Port_t *port);
void ProtocolResetWait(Port_t *port);
void ProtocolRxWait(void);
void ProtocolGetRxPacket(Port_t *port, AW_BOOL HeaderReceived);
void ProtocolTransmitMessage(Port_t *port);
void ProtocolSendingMessage(Port_t *port);
void ProtocolWaitForPHYResponse(void);
void ProtocolVerifyGoodCRC(Port_t *port);
void ProtocolSendGoodCRC(Port_t *port, SopType sop);
void ProtocolTrimOscFun(Port_t *port);
void ProtocolLoadSOP(Port_t *port, SopType sop);
void ProtocolLoadEOP(Port_t *port);
void ProtocolSendHardReset(Port_t *port);
void ProtocolFlushRxFIFO(Port_t *port);
void ProtocolFlushTxFIFO(Port_t *port);
void ResetProtocolLayer(Port_t *port, AW_BOOL ResetPDLogic);
#endif	/* _PDPROTOCOL_H_ */

