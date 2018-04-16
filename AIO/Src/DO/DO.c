/*
 * DO.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include "DO.h"
#include "tim.h"

DO_States DO_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	return DO_succes;
}

DO_States DO_Control(){
	return DO_error;
}
