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

#define MAX_RAW_MESSAGE_SIZE 	2055
#define huartMT huart2

typedef enum {
	MT_succes,
	MT_failed
}MT_StatusTypeDef;

MT_StatusTypeDef MT_Init();
MT_StatusTypeDef MT_Update();
MT_StatusTypeDef MT_StartOperation(bool to_config);
MT_StatusTypeDef MT_CancelOperation();
MT_StatusTypeDef MT_GoToConfig();
MT_StatusTypeDef MT_GoToMeasure();
MT_StatusTypeDef MT_BuildConfig(enum XsDataIdentifier XDI, uint16_t frequency, bool complete);
uint* MT_GetSuccErr();
float* MT_GetAcceleration();
float* MT_GetAngles();
void MT_FactoryReset();
void MT_RequestConfig();
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_UART_RxCpltCallback();

#endif /* MTICONTROL_H_ */
