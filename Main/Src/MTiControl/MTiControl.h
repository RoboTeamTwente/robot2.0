/*
 * MTiControl.h
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */

#ifndef MTICONTROL_H_
#define MTICONTROL_H_

#include <stdlib.h>
#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "xbusmessage.h"
#include "xbusparser.h"
#include "xbusutility.h"
#include "xsdeviceid.h"

#define MAX_RAW_MESSAGE_SIZE 	2055

extern uint8_t cplt_mess_stored_flag;
extern struct XbusMessage* ReceivedMessageStorage;

void XBP_handleMessage(struct XbusMessage const* message);
void* XBP_allocateBuffer(size_t bufSize);
void XBP_deallocateBuffer(void const* buffer);
void CancelMtiOperation();
void MtiReset();
void SendWakeUpAck();
int WaitForAck(enum XsMessageId XMID);
HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size);
void MTi_Init();
void SendXbusMessage(struct XbusMessage XbusMessage);
void ReadNewMessage(uint8_t cancel_previous);
void StoreReceivedBytes();
void CheckWhatNeedsToBeDone();
void DeallocateMem();
void MT_HAL_UART_TxCpltCallback();
void MT_HAL_UART_RxCpltCallback();

#endif /* MTICONTROL_H_ */
