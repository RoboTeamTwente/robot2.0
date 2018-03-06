/*
 * motorscomm.c
 *
 *  Created on: Nov 30, 2017
 *      Author: Leon
 */
#include "motorscomm.h"

#include <stdlib.h>
#include "../PuttyInterface/PuttyInterface.h"

// used to convert the abstract message into a byte array
typedef union{
	float vel;
	uint32_t id;
}velocity_id_union;

///* Pulls slave line low to start communication
// *
// */
//static void motorscomm_SlaveSelect(motorscomm_HandleTypeDef* mc, uint32_t slave){
//	switch(slave){
//	case 0:
//		HAL_GPIO_WritePin((mc->slaves+slave)->slave_GPIO_Port,(mc->slaves+slave)->slave_Pin, 0);
//		break;
//	}
//	//HAL_Delay(1);
//}
//
///*
// * pulls slave line high to stop communication
// */
//static void motorscomm_SlaveDeselect(motorscomm_HandleTypeDef* mc, uint32_t slave){
//	switch(slave){
//	case 0:
//		HAL_GPIO_WritePin((mc->slaves+slave)->slave_GPIO_Port,(mc->slaves+slave)->slave_Pin, 1);
//		break;
//	}
//}

/*
 * initialization function
 */
void motorscomm_Init(motorscomm_HandleTypeDef* mc){
	HAL_UART_Receive_DMA(mc->huart, mc->UART2RX_buf, 16);
}

/*
 * update function which should be called regularly to keep communication going
 */
uint8_t motorscomm_Update(motorscomm_HandleTypeDef* mc){
	HAL_UART_Receive_DMA(mc->huart,mc->UART2RX_buf,16);
	motorscomm_DecodeBuf(mc);
	return 1;
}

/*
 * Function to convert the received bytearray into an abstract message
 */
void motorscomm_DecodeBuf(motorscomm_HandleTypeDef* mc){
	uint32_t* ptr = (uint32_t*)mc->UART2RX_buf;
	velocity_id_union uni;
	for(uint32_t i = 0; i < 4; i++){
		uni.id = *ptr++;
		mc->RX_message.wheel_speed[i] = uni.vel;
		mc->RX_message.id[i] = uni.id & 0x03;
	}
}

/*
 * Function to transform an abstract message into a byte array
 */
void motorscomm_EncodeBuf(motorscomm_HandleTypeDef* mc){
	velocity_id_union bytes;
	uint32_t* ptr = (uint32_t*)mc->UART2TX_buf;
	for(uint32_t i = 0; i < 4; i++){
		bytes.vel = mc->TX_message.wheel_speed[i];
		bytes.id &= 0xfffffffc;
		bytes.id |= mc->TX_message.id[i] & 0x3;
		*ptr++ = bytes.id;
	}
}

/*
 * Start transmit receive operation, if this is the slave, it will wait otherwise it will select the slave and start the communication
 */
HAL_StatusTypeDef motorscomm_UART_StartTransmit(motorscomm_HandleTypeDef* mc, uint32_t slave){
	motorscomm_EncodeBuf(mc);
	return HAL_UART_Transmit_DMA(mc->huart,mc->UART2TX_buf, 16);
}

/*
 * function to be called when a UART Callback is received
 */
void motorscomm_UART_Callback(motorscomm_HandleTypeDef* mc, UART_HandleTypeDef *huart)
{
}
