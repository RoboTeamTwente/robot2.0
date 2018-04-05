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

typedef enum {
	MT_succes,
	MT_failed
}MT_StatusTypeDef;

void MT_Init();
MT_StatusTypeDef MT_Update();
MT_StatusTypeDef MT_StartOperation(bool to_config);
void MT_CancelOperation();
MT_StatusTypeDef MT_WaitForAck(enum XsMessageId XMID);
void MT_SendXbusMessage(struct XbusMessage XbusMessage);
void MT_ReadNewMessage(uint8_t cancel_previous);
void MT_HandleMessage(struct XbusMessage* RX_message);
void MT_ReadContinuously(bool yes);
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_UART_RxCpltCallback();

#endif /* MTICONTROL_H_ */
