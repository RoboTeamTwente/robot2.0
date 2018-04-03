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
#include "xbus/xbusparser.h"
#include "xbus/xbusutility.h"
#include "xbus/xsdeviceid.h"

#define MAX_RAW_MESSAGE_SIZE 	2055
#define huartMT huart2

/*extern */uint8_t MT_cplt_mess_stored_flag;
/*extern */struct XbusMessage* MT_ReceivedMessageStorage;

void MT_Init();
void MT_Update();
void Mt_CancelOperation();
void MT_SendWakeUpAck();
int  MT_WaitForAck(enum XsMessageId XMID);
void MT_SendXbusMessage(struct XbusMessage XbusMessage);
void MT_ReadNewMessage(uint8_t cancel_previous);
void MT_HandleMessage(struct XbusMessage* RX_message);
void MT_ReadContinuously(bool par);
// Callback is called when the HAL_Uart application is finished transmitting its bytes
void MT_UART_TxCpltCallback();
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_UART_RxCpltCallback();

#endif /* MTICONTROL_H_ */
