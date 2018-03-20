/*
 * MTiControl.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */
#include "MTiControl.h"
#include <string.h>

#define MThuart huart2

uint8_t aRxBuffer[2];
struct XbusParser * XBParser;
uint8_t HAL_UART_TxCpltCallback_flag = 0;
uint8_t HAL_UART_RxCpltCallback_flag = 0;
uint8_t HAL_UART_ErrorCallback_flag = 0;
uint8_t bytes_received[MAX_RAW_MESSAGE_SIZE];
uint16_t bytes_received_cnt = 0;

//also extern
uint8_t cplt_mess_stored_flag = 0;
struct XbusMessage* ReceivedMessageStorage;



void CheckWhatNeedsToBeDone(){
	if(HAL_UART_TxCpltCallback_flag){
		HAL_UART_TxCpltCallback_flag = 0;
	}
	if(HAL_UART_RxCpltCallback_flag != 0){
		HAL_UART_RxCpltCallback_flag = 0;
		StoreReceivedBytes();
	}
	if(HAL_UART_ErrorCallback_flag != 0){
		HAL_UART_ErrorCallback_flag = 0;
	}
}
// Everytime a byte is received over usart this function will be called and store/parse it in the xbusparser.
void StoreReceivedBytes(){
	static uint16_t bytes_parsed = 0;
	//TextOut("StoreReceivedBytes\n\r");
	while(bytes_parsed < bytes_received_cnt){
		XbusParser_parseByte(XBParser, bytes_received[bytes_parsed++]);
	}
	if(!cplt_mess_stored_flag){
		Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
	}else{
		bytes_parsed = 0;
		bytes_received_cnt = 0;
	}
}
HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size){
	HAL_StatusTypeDef return_value;
	return_value = HAL_UART_Receive_IT(&MThuart, message, size);

	return return_value;
}
// Callback is called when the HAL_Uart application is finished transmitting its bytes
void MT_HAL_UART_TxCpltCallback(){

	HAL_UART_TxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_HAL_UART_RxCpltCallback(){
	bytes_received[bytes_received_cnt++] = aRxBuffer[0];
	HAL_UART_RxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart application returns an error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	// Prevent unused argument(s) compilation warning
	UNUSED(huart);
	HAL_UART_ErrorCallback_flag = 1;
}
// An xbusmessage is formatted and sent over usart
void SendXbusMessage(struct XbusMessage XbusMessage){
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	HAL_UART_Transmit_IT(&MThuart, raw, XbusMes_size);
}
//This function sends the wakeUpAck message.
//param  None
//retval 0 if sent -1 is error, 1 is timout, busy or smth else
void SendWakeUpAck(){
	struct XbusMessage XbusMes = {XMID_WakeUpAck};
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMes, XLLF_Uart);
	HAL_UART_Transmit_IT(&MThuart, raw, XbusMes_size);
}
// Wait till a certain message type is received from MTi over usart
int WaitForAck(enum XsMessageId XMID){
	uint8_t buf[5];
	HAL_UART_Receive(&MThuart, buf, 5, 1000);
	XbusParser_parseBuffer(XBParser, buf, 5);
	if(cplt_mess_stored_flag){
		if(ReceivedMessageStorage->mid == XMID){
			return 1;
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}
// Initialize controlling the MTi device
void MTi_Init(){
	ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);// Reserve memory to store the longest possible message

	HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 0);// set the MTi in reset state
	struct XbusParserCallback XBP_callback = {};// Create a struct to contain the cllback functions
	XBP_callback.handleMessage = XBP_handleMessage;
	XBP_callback.allocateBuffer = XBP_allocateBuffer;
	XBP_callback.deallocateBuffer = XBP_deallocateBuffer;
	XBParser = XbusParser_create(&XBP_callback);// Create an XBus parser
}

// Start reading a new message
// if cancel_previous, the current running receive operation is cancelled
void ReadNewMessage(uint8_t cancel_previous){
	if(cancel_previous){
		HAL_UART_AbortReceive(&MThuart);
	}
	Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
}
// When a complete message is received from the device this function will process it

void MtiReset(){
	HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 0);
}
void CancelMtiOperation(){
	HAL_UART_Abort(&MThuart);
	MtiReset();
}
// XBP_handleMessage is called when the xbusparser is done parsing one message
void XBP_handleMessage(struct XbusMessage const* message){
	cplt_mess_stored_flag = 1;
	memcpy(ReceivedMessageStorage, message, MAX_RAW_MESSAGE_SIZE);
}
void* XBP_allocateBuffer(size_t bufSize){
	return bufSize < MAX_RAW_MESSAGE_SIZE? malloc(bufSize) : NULL;
}
void XBP_deallocateBuffer(void const* buffer){
	free((uint8_t(*)[MAX_RAW_MESSAGE_SIZE])buffer);
}
void DeallocateMem(){
	TheAlligator(XBParser);
}
