/*
 * MTiControl.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */
#include "MTiControl.h"
#include <string.h>

uint8_t aRxBuffer[2];
struct XbusParser * XBParser;
uint8_t HAL_UART_TxCpltCallback_flag = 0;
uint8_t HAL_UART_RxCpltCallback_flag = 0;
uint8_t HAL_UART_ErrorCallback_flag = 0;
uint8_t bytes_received[MAX_RAW_MESSAGE_SIZE];
uint16_t bytes_received_cnt = 0;
uint8_t message_handled_flag = 0;
uint8_t stop_after_message_complete = 1;

//also extern
//uint8_t MT_cplt_mess_stored_flag = 0;
//struct XbusMessage* MT_ReceivedMessageStorage;

void XBP_handleMessage(struct XbusMessage const* message);
void* XBP_allocateBuffer(size_t bufSize);
void XBP_deallocateBuffer(void const* buffer);
HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size);
void StoreReceivedBytes();
void HandleMessage();
void DeallocateMem();
void Reset();

// Initialize controlling the MTi device
void MT_Init(){
	MT_ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);// Reserve memory to store the longest possible message

	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);// set the MTi in reset state
	struct XbusParserCallback XBP_callback = {};// Create a structure to contain the callback functions
	XBP_callback.handleMessage = XBP_handleMessage;
	XBP_callback.allocateBuffer = XBP_allocateBuffer;
	XBP_callback.deallocateBuffer = XBP_deallocateBuffer;
	XBParser = XbusParser_create(&XBP_callback);// Create an XBus parser
}

void MT_Update(){
	if(MT_cplt_mess_stored_flag){
		MT_cplt_mess_stored_flag = 0;
		HandleMessage();
		if(!stop_after_message_complete){
			message_handled_flag = 0;
			MT_ReadNewMessage(0);
		}
	}
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
void Reset(){
	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);
}
void MT_CancelOperation(){
	HAL_UART_Abort(&huartMT);
	Reset();
}
// Callback is called when the HAL_Uart application is finished transmitting its bytes
void MT_UART_TxCpltCallback(){
	HAL_UART_TxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MT_UART_RxCpltCallback(){
	bytes_received[bytes_received_cnt++] = aRxBuffer[0];
	HAL_UART_RxCpltCallback_flag = 1;
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
}
//This function sends the wakeUpAck message.
//param  None
//retval 0 if sent -1 is error, 1 is timout, busy or smth else
void MT_SendWakeUpAck(){
	struct XbusMessage XbusMes = {XMID_WakeUpAck};
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMes, XLLF_Uart);
	HAL_UART_Transmit_IT(&huartMT, raw, XbusMes_size);
}
// Wait till a certain message type is received from MTi over usart
int MT_WaitForAck(enum XsMessageId XMID){
	uint8_t buf[5];
	HAL_UART_Receive(&huartMT, buf, 5, 500);
	XbusParser_parseBuffer(XBParser, buf, 5);
	if(MT_cplt_mess_stored_flag){
		if(MT_ReceivedMessageStorage->mid == XMID){
			return 1;
		}else{
			return 0;
		}
	}else{
		return 0;
	}
}

// Start reading a new message
// if cancel_previous, the current running receive operation is cancelled
void MT_ReadNewMessage(uint8_t cancel_previous){
	if(cancel_previous){
		HAL_UART_AbortReceive(&huartMT);
	}
	Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
}

__weak void MT_HandleMessage(struct XbusMessage* RX_message){
	UNUSED(RX_message);// prevents warning, because it is not used
	//rewrite implementation in main.c
}
void MT_ReadContinuously(bool par){
	stop_after_message_complete = par;
}
// When a complete message is received from the device this function will process it

void HandleMessage(){
	MT_HandleMessage(MT_ReceivedMessageStorage);
	message_handled_flag = 1;
	DeallocateMem();
}
// XBP_handleMessage is called when the xbusparser is done parsing one message
void XBP_handleMessage(struct XbusMessage const* message){
	MT_cplt_mess_stored_flag = 1;
	memcpy(MT_ReceivedMessageStorage, message, MAX_RAW_MESSAGE_SIZE);
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
// Everytime a byte is received over usart this function will be called and store/parse it in the xbusparser.
void StoreReceivedBytes(){
	static uint16_t bytes_parsed = 0;
	//TextOut("StoreReceivedBytes\n\r");
	while(bytes_parsed < bytes_received_cnt){
		XbusParser_parseByte(XBParser, bytes_received[bytes_parsed++]);
	}
	if(!MT_cplt_mess_stored_flag){
		Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
	}else{
		bytes_parsed = 0;
		bytes_received_cnt = 0;
	}
}
HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size){
	HAL_StatusTypeDef return_value;
	return_value = HAL_UART_Receive_IT(&huartMT, message, size);

	return return_value;
}
