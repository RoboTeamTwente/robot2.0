/*
 * MTiControl.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */
#include "MTiControl.h"
#include <string.h>
#include "../PuttyInterface/PuttyInterface.h"

uint8_t aRxBuffer[2055];
struct XbusParser * XBParser;
uint8_t RxCpltCallback_flag = 0;
uint8_t HAL_UART_ErrorCallback_flag = 0;
uint8_t bytes_received[MAX_RAW_MESSAGE_SIZE];
// When XBP_Handlemessage is called, this value is set true
uint8_t cplt_mess_stored_flag;
struct XbusMessage* ReceivedMessageStorage;

enum reception_states{
	receive_5,
	receive_rest
}reception_state = receive_5;
/*
 * static function prototypes
 */
static void XBP_handleMessage(struct XbusMessage const* message);
static void* XBP_allocateBuffer(size_t bufSize);
static void XBP_deallocateBuffer(void const* buffer);
static void SendWakeUpAck();

/*
 * -----------------------------public functions
 */
// Initialize controlling the MTi device
void MT_Init(){
	ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);// Reserve memory to store the longest possible message

	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);// set the MTi in reset state
	struct XbusParserCallback XBP_callback = {};// Create a structure to contain the callback functions
	XBP_callback.handleMessage = XBP_handleMessage;
	XBP_callback.allocateBuffer = XBP_allocateBuffer;
	XBP_callback.deallocateBuffer = XBP_deallocateBuffer;
	XBParser = XbusParser_create(&XBP_callback);// Create an XBus parser
}

MT_StatusTypeDef MT_Update(){
	if(RxCpltCallback_flag != 0){
		RxCpltCallback_flag = 0;
		if(cplt_mess_stored_flag){
			cplt_mess_stored_flag = 0;
			MT_HandleMessage(ReceivedMessageStorage);
			TheAlligator(XBParser);
			MT_ReadNewMessage(0);
		}else{
			switch(reception_state){
			case receive_5:
				if(XBParser->currentMessage.length){
					reception_state = receive_rest;
					HAL_UART_Receive_IT(&huartMT, (uint8_t *)aRxBuffer, XBParser->currentMessage.length);
				}
				break;
			case receive_rest:

				break;
			}
		}
	}

	if(HAL_UART_ErrorCallback_flag != 0){
		HAL_UART_ErrorCallback_flag = 0;
		return MT_failed;
	}

	return MT_succes;
}


MT_StatusTypeDef MT_StartOperation(bool to_config){
	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 1);
	if(MT_WaitForAck(XMID_WakeUp) == MT_succes){
		if(to_config) SendWakeUpAck();
		return MT_succes;
	}else{
		return MT_failed;
	}
}

void MT_CancelOperation(){
	HAL_UART_Abort(&huartMT);
	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);
}
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_UART_RxCpltCallback(){
	switch(reception_state){
	case receive_5:
		XbusParser_parseBuffer(XBParser, aRxBuffer, 5);
		break;
	case receive_rest:
		XbusParser_parseBuffer(XBParser, aRxBuffer, XBParser->currentMessage.length);
		reception_state = receive_5;
		break;
	}
	RxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart application returns an error
void MT_UART_ErrorCallback(){
	HAL_UART_ErrorCallback_flag = 1;
}
// An xbusmessage is formatted and sent over usart
void MT_SendXbusMessage(struct XbusMessage XbusMessage){
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	HAL_UART_Transmit_IT(&huartMT, raw, XbusMes_size);
	for(uint i = 0; i < XbusMes_size; i++){
		uprintf("[%02x]", raw[i]);
	}
	uprintf("\n\r");
}

// Wait till a certain message type is received from MTi over usart
MT_StatusTypeDef MT_WaitForAck(enum XsMessageId XMID){
	uint timeout = 0;
	bool timedout = false;
	MT_ReadNewMessage(0);
	timeout = HAL_GetTick();
	while(ReceivedMessageStorage->mid != XMID && (timedout = ((HAL_GetTick() - timeout) < 500U))){
		MT_Update();
	}
	if(timedout){
		return MT_succes;
	}else{
		return MT_failed;
	}
}

// Start reading a new message
// if cancel_previous, the current running receive operation is cancelled
void MT_ReadNewMessage(uint8_t cancel_previous){
	if(cancel_previous){
		HAL_UART_AbortReceive(&huartMT);
	}
	HAL_UART_Receive_IT(&huartMT, (uint8_t *)aRxBuffer, 5);
}

__weak void MT_HandleMessage(struct XbusMessage* RX_message){
	UNUSED(RX_message);// prevents warning, because it is not used
	//rewrite implementation in main.c
}

/*
 * ----------------------------Static function--------------------
 */

// XBP_handleMessage is called when the xbusparser is done parsing one message
static void XBP_handleMessage(struct XbusMessage const* message){
	cplt_mess_stored_flag = 1;
	memcpy(ReceivedMessageStorage, message, MAX_RAW_MESSAGE_SIZE);
}
static void* XBP_allocateBuffer(size_t bufSize){
	return bufSize < MAX_RAW_MESSAGE_SIZE? malloc(bufSize) : NULL;
}
static void XBP_deallocateBuffer(void const* buffer){
	free((uint8_t(*)[MAX_RAW_MESSAGE_SIZE])buffer);
}
//This function sends the wakeUpAck message.
//param  None
static void SendWakeUpAck(){
	struct XbusMessage XbusMes = { XMID_WakeUpAck};
	MT_SendXbusMessage(XbusMes);
}
