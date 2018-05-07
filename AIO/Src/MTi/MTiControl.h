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

#define MAX_RAW_MESSAGE_SIZE 	2055		// maximum extended message length
#define huartMT huart2						// which huart interface to use

typedef enum {
	MT_succes,
	MT_failed
}MT_StatusTypeDef;// Defines the status that most functions will return
enum{
	Xsens_Reset,
	Xsens_Config,
	Xsens_Measure,
	Xsens_Unknown
}Xsens_state = Xsens_Unknown;

MT_StatusTypeDef MT_Init();
/*	Call at a High > 200 HZ to ensure all messages will be handled
 * 	Return MT_failed if HAL_UART gives an error
 */
MT_StatusTypeDef MT_Update();
/* 	Start the Xsens and leave it in configuration mode
 * 	retval: MT_succes if startup message is received;
 */
MT_StatusTypeDef MT_StartOperation(bool to_config);
/*	Put the xsens in Reset mode
 * 	retval: HAL_failed if unable to abort UART operation
 */
MT_StatusTypeDef MT_DeInit();
/*	Put Xsens into Configuration state
 * 	retval: MT_succes if GoToConfigAck is received MT_failed otherwise
 */
MT_StatusTypeDef MT_GoToConfig();
/*	set the option flags
 *
 */
MT_StatusTypeDef MT_SetOptions();
/*	request the option flags
 *
 */
MT_StatusTypeDef MT_ReqOptions();
/*	Put Xsens into Measurement state
 * 	retval: MT_succes if GoToMeasutementAck is received MT_failed otherwise
 */
MT_StatusTypeDef MT_GoToMeasure();
/*	Used to build a config for the Xsens output
 * 	param:
 * 		XDI: Datatype choose from enum XsDataIdentifier
 * 		frequency: maximum of 100 divided by a multiple of 2
 * 		complete: if true complete config is sent to Xsens
 * 	retval: status
 */
MT_StatusTypeDef MT_BuildConfig(enum XsDataIdentifier XDI, uint16_t frequency, bool complete);
/*	Get the succes rate by number of successful data receptions and number of error messages
 * 	retval: pointer to struct containing two uints {success, error}
 */
uint* MT_GetSuccErr();
/*	Returns the last received Acceleration values
 * 	retval: pointer to array of 3 floats { x, y, z}
 */
float* MT_GetAcceleration();
/*	Returns the last received Euler angles
 * 	retval: pointer to array of 3 floats { x, y, z}
 */
float* MT_GetAngles();
/*	Resets the Xsens to factory settings
 *
 */
void MT_FactoryReset();
/*	request the current outputconfig
 *
 */
void MT_RequestConfig();
/*	Callback is called when the HAL_Uart received its wanted amount of bytes
 *
 */
void MT_UART_RxCpltCallback();

#endif /* MTICONTROL_H_ */
