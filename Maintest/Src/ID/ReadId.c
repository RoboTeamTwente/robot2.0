/*
 * ReadId.c
 *
 *  Created on: Mar 9, 2018
 *      Author: Leon
 */
#include "gpio.h"
#include "ReadId.h"

uint8_t ReadAddress(){
	return 0x0F & ~(HAL_GPIO_ReadPin(ID3_GPIO_Port,ID3_Pin) << 3 | HAL_GPIO_ReadPin(ID2_GPIO_Port,ID2_Pin) << 2 | HAL_GPIO_ReadPin(ID1_GPIO_Port,ID1_Pin) << 1 | HAL_GPIO_ReadPin(ID0_GPIO_Port,ID0_Pin));
}
