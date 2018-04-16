/*
 * wheels.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include "wheels.h"

#include <math.h>
#include <stdbool.h>
#include "tim.h"
#include "../PuttyInterface/PuttyInterface.h"

#define PWM_CUTOFF 10.0F

void wheels_Init(){
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

#define ROBOT_RADIUS 0.0775F
#define WHEEL_RADIUS 0.0275F
void calcMotorSpeed (float magnitude, float direction, int rotSign, float wRadPerSec, float power[4]){
//	static float cos_a0 = cos(60.0F  * 3.1415F/180.0F); //240
//	static float cos_a1 = cos(120.0F * 3.1415F/180.0F); //300
//	static float cos_a2 = cos(240.0F * 3.1415F/180.0F); //60
//	static float cos_a3 = cos(300.0F * 3.1415F/180.0F); //120
//	static float sin_a0 = sin(60.0F  * 3.1415F/180.0F); //240
//	static float sin_a1 = sin(120.0F * 3.1415F/180.0F); //300
//	static float sin_a2 = sin(240.0F * 3.1415F/180.0F); //60
//	static float sin_a3 = sin(300.0F * 3.1415F/180.0F); //120
//	float xSpeed; //positive x = moving forward
//	float ySpeed; //positive y = moving to the right
//	float angularComponent;
//
//	xSpeed = -cos(direction) * magnitude;
//	ySpeed = -sin(direction) * magnitude;
//
//
//	angularComponent = rotSign * ROBOT_RADIUS * wRadPerSec;
//
//	power[wheels_RF] = (cos_a0 * ySpeed * 1.4 + sin_a0 * xSpeed + angularComponent) / WHEEL_RADIUS;
//	power[wheels_RB] = (cos_a1 * ySpeed * 1.4 + sin_a1 * xSpeed + angularComponent) / WHEEL_RADIUS;
//	power[wheels_LB] = (cos_a2 * ySpeed * 1.4 + sin_a2 * xSpeed + angularComponent) / WHEEL_RADIUS;
//	power[wheels_LF] = (cos_a3 * ySpeed * 1.4 + sin_a3 * xSpeed + angularComponent) / WHEEL_RADIUS;

	// Jelle's variant, using forces instead of velocities (for testing)
	static float cos_a0 = cosf(60.0F  * 3.1415F/180.0F);
	static float sin_a0 = sinf(60.0F  * 3.1415F/180.0F);
	float xForce; //positive x = moving forward
	float yForce; //positive y = moving to the right
	float robotTorque;

	xForce = cosf(direction) * magnitude * 1537.7;
	yForce = sinf(direction) * magnitude * 1537.7;
	robotTorque = rotSign * wRadPerSec * 31.5;

	power[wheels_RF] = ( xForce/(4*sin_a0) + yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_RB] = ( xForce/(4*sin_a0) - yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_LB] = ( -xForce/(4*sin_a0) - yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_LF] = ( -xForce/(4*sin_a0) + yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;

}

void wheels_SetOutput(float power[4]){
//	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, 0);
//	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, 0);
//	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

	static bool reverse[4] = {0};
	bool prev_reverse[4];
	memcpy(prev_reverse, reverse, 4);
	bool delay = false;
	for(wheels_handles i = wheels_RF; i <= wheels_LF; i++){
		if(power[i] < -0.1 ){
			power[i] = -power[i];
			reverse[i] = 1;
		}else if(power[i] > 0.1 ){
			reverse[i] = 0;
		}
		if(power[i] > 100){
			power[i] = 100;
		}else if(power[i] < PWM_CUTOFF){
			power[i] = 0;
		}
	}
	delay = memcmp(prev_reverse, reverse, 4);

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

	if(delay)	HAL_Delay(1);

	HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse[wheels_RF]);
	HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse[wheels_RB]);
	HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse[wheels_LB]);
	HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse[wheels_LF]);

	if(delay)	HAL_Delay(0);

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, power[wheels_RF] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, power[wheels_RB] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, power[wheels_LB] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, power[wheels_LF] / 100 * MAX_PWM);
}

int16_t wheels_GetEncoder(wheels_handles wheel){
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
	float retval = 0;
	static uint prev_time[4];
	static uint prev_enco[4];
	switch(wheel){
	case wheels_RF:
		retval = (float)(wheels_GetEncoder(wheels_RF) - prev_enco[wheels_RF])/( __HAL_TIM_GET_COUNTER(&htim5) - prev_time[wheels_RF]);
		prev_time[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim5);
		prev_enco[wheels_RF] = wheels_GetEncoder(wheels_RF);
		break;
	case wheels_RB:
		retval = (float)(wheels_GetEncoder(wheels_RB) - prev_enco[wheels_RB])/( __HAL_TIM_GET_COUNTER(&htim5) - prev_time[wheels_RB]);
		prev_time[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim5);
		prev_enco[wheels_RB] = wheels_GetEncoder(wheels_RB);
		break;
	case wheels_LB:
		retval = (float)(wheels_GetEncoder(wheels_LB) - prev_enco[wheels_LB])/( __HAL_TIM_GET_COUNTER(&htim5) - prev_time[wheels_LB]);
		prev_time[wheels_LB] = __HAL_TIM_GET_COUNTER(&htim5);
		prev_enco[wheels_LB] = wheels_GetEncoder(wheels_LB);
		break;
	case wheels_LF:
		retval = (float)(wheels_GetEncoder(wheels_LF) - prev_enco[wheels_LF])/( __HAL_TIM_GET_COUNTER(&htim5) - prev_time[wheels_LF]);
		prev_time[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim5);
		prev_enco[wheels_LF] = wheels_GetEncoder(wheels_LF);
		break;
	}
	return retval;
}
