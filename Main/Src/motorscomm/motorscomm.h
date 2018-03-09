/*
 * motorscomm.h
 *
 *  Created on: Nov 30, 2017
 *      Author: Leon
 */

#ifndef MOTORSCOMM_H_
#define MOTORSCOMM_H_

#include "gpio.h"
#include "usart.h"

#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

/*
 * holds a slave pin port and pin combination
 */
typedef struct{
	GPIO_TypeDef * slave_GPIO_Port;
	uint32_t slave_Pin;
}motorscomm_slaveHandelTypeDef;
/*
 * each wheel enumerated from right front clockwise to left front wheel
 */
enum wheel_number{
	motorscomm_RF,
	motorscomm_RB,
	motorscomm_LB,
	motorscomm_LF,
};
/*
 * abstract message which holds the wheel speeds and their id in the same order
 */
typedef struct{
	float wheel_speed[4];
	uint8_t id[4];
}motorscomm_abstract_messageHandleTypeDef;

/*
 * struct holding all the variable needed for the communication
 */
typedef struct{
	motorscomm_abstract_messageHandleTypeDef TX_message;
	uint8_t UART2TX_buf[16];
	motorscomm_abstract_messageHandleTypeDef RX_message;
	uint8_t UART2RX_buf[16];
	UART_HandleTypeDef *huart;
	uint32_t n_slaves;
	motorscomm_slaveHandelTypeDef* slaves;
	uint8_t UART2_Rx_flag;
}motorscomm_HandleTypeDef;

/*
 * initialization function
 */
void motorscomm_Init(motorscomm_HandleTypeDef* mc);

/*
 * update function which should be called regularly to keep communication going
 */
uint8_t motorscomm_Update(motorscomm_HandleTypeDef* mc);

/*
 * Function to convert the received bytearray into an abstract message
 */
void motorscomm_DecodeBuf(motorscomm_HandleTypeDef* mc);
/*
 * Function to transform an abstract message into a byte array
 */
void motorscomm_EncodeBuf(motorscomm_HandleTypeDef* mc);

/*
 * Start transmit receive operation, if this is the slave, it will wait otherwise it will select the slave and start the communication
 */
HAL_StatusTypeDef motorscomm_UART_StartTransmit(motorscomm_HandleTypeDef* mc, uint32_t slave);

/*
 *
 */
HAL_StatusTypeDef motorscomm_HAL_UART_Receive(motorscomm_HandleTypeDef* mc);

/*
 * function to be called when a UART Callback is received
 */
void motorscomm_UART_Callback(motorscomm_HandleTypeDef* mc, UART_HandleTypeDef *huart);

#endif /* MOTORSCOMM_H_ */
