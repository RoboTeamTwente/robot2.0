/*
 * userIO.c
 *
 *  Created on: May 23, 2018
 *      Author: Leon
 */

#include "userIO.h"

#include "gpio.h"

inline int ReadAddress(){
	return 0x0F & ~(HAL_GPIO_ReadPin(ID3_GPIO_Port,ID3_Pin) << 3 | HAL_GPIO_ReadPin(ID2_GPIO_Port,ID2_Pin) << 2 | HAL_GPIO_ReadPin(ID1_GPIO_Port,ID1_Pin) << 1 | HAL_GPIO_ReadPin(ID0_GPIO_Port,ID0_Pin));
}

void Uint2Leds(uint8_t uint, uint8_t n_leds){
	if(!n_leds) n_leds = 0xff;
	if(n_leds & 0b00000001) HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, uint & 0b00000001);
	if(n_leds & 0b00000010) HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, uint & 0b00000010);
	if(n_leds & 0b00000100) HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, uint & 0b00000100);
	if(n_leds & 0b00001000) HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin, uint & 0b00001000);
	if(n_leds & 0b00010000) HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin, uint & 0b00010000);
	if(n_leds & 0b00100000) HAL_GPIO_WritePin(LD6_GPIO_Port,LD6_Pin, uint & 0b00100000);
}

inline void ToggleLD(uint8_t LD_){
	switch(LD_){
	case 1:
		HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
		break;
	case 2:
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		break;
	case 3:
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
		break;
	case 4:
		HAL_GPIO_TogglePin(LD4_GPIO_Port,LD4_Pin);
		break;
	case 5:
		HAL_GPIO_TogglePin(LD5_GPIO_Port,LD5_Pin);
		break;
	case 6:
		HAL_GPIO_TogglePin(LD6_GPIO_Port,LD6_Pin);
		break;
	default:
		break;
	}
}

inline void SetLD(uint8_t LD_, uint8_t on){
	switch(LD_){
	case 1:
		HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, on);
		break;
	case 2:
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, on);
		break;
	case 3:
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, on);
		break;
	case 4:
		HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin, on);
		break;
	case 5:
		HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin, on);
		break;
	case 6:
		HAL_GPIO_WritePin(LD6_GPIO_Port,LD6_Pin, on);
		break;
	default:
		break;
	}
}
