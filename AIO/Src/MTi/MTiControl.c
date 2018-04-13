/*
 * MTiControl.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */
#include "MTiControl.h"
#include <string.h>
#include "xbus/xbusutility.h"
#include "xbus/xsdeviceid.h"
#include "../PuttyInterface/PuttyInterface.h"

#define MAX_XDI_CONFIGS 64
#define MT_DEBUG 0

uint8_t aRxBuffer[2055];
struct XbusParser * XBParser;
uint8_t RxCpltCallback_flag = 0;
uint8_t HAL_UART_ErrorCallback_flag = 0;
uint8_t bytes_received[MAX_RAW_MESSAGE_SIZE];
// When XBP_Handlemessage is called, this value is set true
uint8_t cplt_mess_stored_flag;
struct XbusMessage* ReceivedMessageStorage;
uint MT_Data_succerr[2] = {0};
float angles[3];
float acc[3];

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
static void PrintOutputConfig(struct XbusMessage const* message);
static inline void ErrorHandler(struct XbusMessage const* message);
static void PrintMessageData(struct XbusMessage const* message);
static void SendXbusMessage(struct XbusMessage XbusMessage);
static inline void ReadNewMessage(uint8_t cancel_previous);
static inline MT_StatusTypeDef WaitForAck(enum XsMessageId XMID);

/*
 * -----------------------------public functions-------------------------------------
 */
// Initialize controlling the MTi device
MT_StatusTypeDef MT_Init(){
	ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);				// Reserve memory to store the longest possible message

	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);			// set the MTi in reset state
	struct XbusParserCallback XBP_callback = {};						// Create a structure to contain the callback functions
	XBP_callback.handleMessage = XBP_handleMessage;
	XBP_callback.allocateBuffer = XBP_allocateBuffer;
	XBP_callback.deallocateBuffer = XBP_deallocateBuffer;
	XBParser = XbusParser_create(&XBP_callback);						// Create an XBus parser
	if(XBParser == NULL){
		return MT_failed;
	}
	return MT_succes;
}

MT_StatusTypeDef MT_Update(){
	if(RxCpltCallback_flag != 0){
		RxCpltCallback_flag = 0;
		if(cplt_mess_stored_flag){
			cplt_mess_stored_flag = 0;
			switch(ReceivedMessageStorage->mid){
			case XMID_Error:
				ErrorHandler(ReceivedMessageStorage);
				MT_Data_succerr[1]++;
				break;
			case XMID_MTData2:
				MT_Data_succerr[0]++;
				PrintMessageData(ReceivedMessageStorage);
				break;
			case XMID_ReqOutputConfigurationAck:
				PrintOutputConfig(ReceivedMessageStorage);
				break;
			default:
				break;
			}
			TheAlligator(XBParser);
			ReadNewMessage(0);
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
	if(WaitForAck(XMID_WakeUp) == MT_succes){
		if(to_config) SendWakeUpAck();
		return MT_succes;
	}else{
		return MT_failed;
	}
}

MT_StatusTypeDef MT_CancelOperation(){
	HAL_GPIO_WritePin(XSENS_nRST_GPIO_Port, XSENS_nRST_Pin, 0);
	if(HAL_OK != HAL_UART_Abort(&huartMT)){
		return MT_failed;
	}
	return MT_succes;
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
void SendXbusMessage(struct XbusMessage XbusMessage){
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	HAL_UART_Transmit_IT(&huartMT, raw, XbusMes_size);
	HAL_Delay(1);
//	for(uint i = 0; i < XbusMes_size; i++){
//		uprintf("[%02x]", raw[i]);
//	}
//	uprintf("\n\r");
}

MT_StatusTypeDef MT_GoToConfig(){
	struct XbusMessage mess = {XMID_GoToConfig};
	uint8_t cnt = 0;
	do{
		SendXbusMessage(mess);
		cnt++;
	}while(WaitForAck(XMID_GoToConfigAck) != MT_succes && cnt < 20 );
	uprintf("cnt = %d\n\r",  cnt);
	if(cnt < 20){
		TextOut("In config state.\n\r");
		return MT_succes;
	}else{
		TextOut("No GoToConfigAck received.\n\r");
		return MT_failed;
	}
}

MT_StatusTypeDef MT_GoToMeasure(){
	struct XbusMessage mess = {XMID_GoToMeasurement};
	uint8_t cnt = 0;
	do {
		SendXbusMessage(mess);
		cnt++;
	}while(WaitForAck(XMID_GoToMeasurementAck) != MT_succes && cnt < 20 );
	uprintf("cnt = %d\n\r",  cnt);
	if(cnt < 20){
		TextOut("In measurement state.\n\r");
		return MT_succes;
	}else{
		TextOut("No GoToMeasurementAck received.\n\r");
		return MT_failed;
	}
}

MT_StatusTypeDef MT_BuildConfig(enum XsDataIdentifier XDI, uint16_t frequency, bool complete){
	static uint8_t n_configs = 0;
	static struct OutputConfiguration config[MAX_XDI_CONFIGS];
	config[n_configs].dtype = XDI;
	config[n_configs++].freq =  frequency;
	if(complete){
		struct XbusMessage mess;
		mess.mid = XMID_SetOutputConfiguration;
		mess.length = n_configs;
		mess.data = &config;
//		uint16_t* mdptr = mess.data;
//		uprintf( "[%x %x] [%x %x] [%x %x]\n\r", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);
		SendXbusMessage(mess);
		n_configs = 0;
	}
	return MT_succes;
}

uint* MT_GetSuccErr(){
	return MT_Data_succerr;
}

float* MT_GetAcceleration(){
	return acc;
}
float* MT_GetAngles(){
	return angles;
}
void MT_FactoryReset(){
	struct XbusMessage mess = { .mid = XMID_RestoreFactoryDef,
										.length = 0,
										.data = NULL};
	SendXbusMessage(mess);
}

void MT_RequestConfig(){
	struct XbusMessage mess = { .mid = XMID_ReqOutputConfiguration,
								.data = NULL,
								.length = 0};
	SendXbusMessage(mess);
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
	SendXbusMessage(XbusMes);
}
static void PrintOutputConfig(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("MTiPrintOutputConfig:\n\r");

	uint8_t* rawptr = message->data;
	uint16_t fptr[message->length];
	uprintf( "len = [%u]", message->length);

	for(uint16_t * i = fptr; i < fptr + (message->length)/2; i++){
		rawptr = XbusUtility_readU16(i, rawptr);
		uprintf( "[%04x]", *i);

	}
	uprintf( "\n\r");


	uint16_t freq;
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Temperature, message))){
		uprintf("XDI_Temperature:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_UtcTime, message))){
		uprintf("XDI_UtcTime:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_PacketCounter, message))){
		uprintf("XDI_PacketCounter:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeFine, message))){
		uprintf("XDI_SampleTimeFine:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeCoarse, message))){
		uprintf("XDI_SampleTimeCoarse:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Quaternion, message))){
		uprintf("XDI_Quaternion:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RotationMatrix, message))){
		uprintf("XDI_RotationMatrix:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_EulerAngles, message))){
		uprintf("XDI_EulerAngles:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaV, message))){
		uprintf("XDI_DeltaV:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Acceleration, message))){
		uprintf("XDI_Acceleration:%x\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_FreeAcceleration, message))){
			uprintf("XDI_FreeAcceleration:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_AccelerationHR, message))){
		uprintf("XDI_AccelerationHR:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurn, message))){
		uprintf("XDI_RateOfTurn:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaQ, message))){
		uprintf("XDI_DeltaQ:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurnHR, message))){
		uprintf("XDI_RateOfTurnHR:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_MagneticField, message))){
		uprintf("XDI_MagneticField:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusByte, message))){
		uprintf("XDI_StatusByte:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusWord, message))){
		uprintf("XDI_StatusWord:%u\n\r", freq);

	}
}
static inline void ErrorHandler(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("ERROR: %02x\n\r", *(uint8_t *)(message->data));

}

static void PrintMessageData(struct XbusMessage const* message){
	int bytes = message->length;
	if (!message)
		return;
	if(MT_DEBUG) uprintf("MT:");

	uint16_t counter;
	if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message)){
		if(MT_DEBUG) uprintf( " Packet cnt: %5d", counter);
		bytes -= 1 + 2 + 2;
	}
	uint32_t SampleTimeFine;
	if (XbusMessage_getDataItem(&SampleTimeFine, XDI_SampleTimeFine, message)){
		if(MT_DEBUG) uprintf( " SampleTimeFine: %lu", SampleTimeFine);
		bytes -= 1 + 2 + 4;
	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message)){
		if(MT_DEBUG) uprintf( " Orientation: (%.3f, %.3f, %.3f, %.3f)", ori[0], ori[1],
				ori[2], ori[3]);
		bytes -= 1 + 4 * 4 + 2;

	}
	if (XbusMessage_getDataItem(angles, XDI_EulerAngles, message)){
		if(MT_DEBUG) uprintf( " EulerAngles: (%.3f, %.3f, %.3f)", angles[0], angles[1], angles[2]);
		bytes -= 1 + 3 * 4 + 2;

	}
	float delta_v[3];
	if (XbusMessage_getDataItem(delta_v, XDI_DeltaV, message)){
		if(MT_DEBUG) uprintf( " deltaV: (%.3f, %.3f, %.3f)", delta_v[0], delta_v[1], delta_v[2]);
		bytes -= 1 + 3 * 4 + 2;
	}
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message)){
		if(MT_DEBUG) uprintf( " Acceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);
		bytes -= 1 + 2 * 4 + 2;
	}
	if (XbusMessage_getDataItem(acc, XDI_FreeAcceleration, message)){
		if(MT_DEBUG) uprintf( " FreeAcceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);
		bytes -= 1 + 3 * 4 + 2;
	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message)){
		if(MT_DEBUG) uprintf( " Rate Of Turn: (%.3f, %.3f, %.3f)", gyr[0], gyr[1], gyr[2]);
		bytes -= 1 + 3 * 4 + 2;
	}
	float delta_q[4];
	if (XbusMessage_getDataItem(delta_q, XDI_Quaternion, message)){
		if(MT_DEBUG) uprintf( " deltaQ: (%.3f, %.3f, %.3f, %.3f)", delta_q[0], delta_q[1],
				delta_q[2], delta_q[3]);
		bytes -= 1 + 4 * 4 + 2;
	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message)){
		if(MT_DEBUG) uprintf( " Magnetic Field: (%.3f, %.3f, %.3f)", mag[0], mag[1], mag[2]);
		bytes -= 1 + 3 * 4 + 2;
	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message)){
		if(MT_DEBUG) uprintf( " Status:%lX", status);
		bytes -= 1 + 4 + 2;
	}
	if(MT_DEBUG) uprintf(" [%i] bytes unread\n\r", bytes);
}
// Start reading a new message
// if cancel_previous, the current running receive operation is cancelled
static inline void ReadNewMessage(uint8_t cancel_previous){
	if(cancel_previous){
		HAL_UART_AbortReceive(&huartMT);
	}
	HAL_UART_Receive_IT(&huartMT, (uint8_t *)aRxBuffer, 5);
}
// Wait till a certain message type is received from MTi over usart
static inline MT_StatusTypeDef WaitForAck(enum XsMessageId XMID){
	uint timeout = 0;
	bool timedout = false;
	ReadNewMessage(0);
	timeout = HAL_GetTick();
	while(ReceivedMessageStorage->mid != XMID && (timedout = ((HAL_GetTick() - timeout) < 1000U))){
		MT_Update();
	}
	if(timedout){
		return MT_succes;
	}else{
		return MT_failed;
	}
}