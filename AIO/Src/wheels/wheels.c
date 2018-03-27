/*
 * wheels.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include "wheels.h"

#include <math.h>
#include "tim.h"

void wheels_Init(){
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim8);
}
void wheels_SetOutput(wheels_handles wheel, float power){
	uint8_t reverse = 0;
	if(power < 0){
		reverse = 1;
	}
	power = fabs(power);
	if(power > 100){
		power = 100;
	}
	switch(wheel){
	case wheels_RF:
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse);
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, power * MAX_PWM);
		break;
	case wheels_RB:
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse);
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, power * MAX_PWM);
		break;
	case wheels_LB:
		HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, power * MAX_PWM);
		break;
	case wheels_LF:
		HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, power * MAX_PWM);
		break;
	}
}
int wheels_GetEncoder(wheels_handles wheel){
	switch(wheel){
	case wheels_RF:
		return __HAL_TIM_GET_COUNTER(&htim8);
	case wheels_RB:
		return __HAL_TIM_GET_COUNTER(&htim1);
	case wheels_LB:
		return __HAL_TIM_GET_COUNTER(&htim3);
	case wheels_LF:
		return __HAL_TIM_GET_COUNTER(&htim4);
	}
	return 0;
}
float wheels_GetSpeed(wheels_handles wheel){
	return (float)wheels_GetEncoder(wheel)/0.01F;
}
