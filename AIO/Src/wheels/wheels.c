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
#define ROBOT_RADIUS 0.0775F
#define WHEEL_RADIUS 0.0275F
#define HTIM5_FREQ 1000000.0F
#define PULSES_PER_ROTATION 1024
#define GEAR_RATIO 2.5F
#define X__ENCODING 4 // counts per pulse X1, X2 or X4

bool reverse[4] = {0};
int pwm[4];

enum {
	wheels_uninitialized,
	wheels_ready,
	wheels_first_brake_period,
	wheels_second_brake_period,
	wheels_third_brake_period
}wheels_state;

/*----------------------------------static functions-------------------------------*/

static inline void ResetEncoder(wheels_handles wheel){
	switch(wheel){
	case wheels_RF:
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		break;
	case wheels_RB:
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		break;
	case wheels_LB:
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		break;
	case wheels_LF:
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		break;
	}
}

/*----------------------------------public functions-------------------------------*/

void wheels_Init(){
	wheels_state = wheels_ready;
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void calcMotorSpeed (float magnitude, float direction, int rotSign, float wRadPerSec, float power[4]){
	// Jelle's variant, using forces instead of velocities (for testing)
	static float cos_a0 = cosf(60.0F  * 3.1415F/180.0F);
	static float sin_a0 = sinf(60.0F  * 3.1415F/180.0F);
	float xForce; //positive x = moving forward
	float yForce; //positive y = moving to the right
	float robotTorque;

	xForce = -cosf(direction) * magnitude * 1537.7;
	yForce = -sinf(direction) * magnitude * 1537.7;
	robotTorque = rotSign * wRadPerSec * 31.5;

	power[wheels_RF] = ( xForce/(4*sin_a0) + yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_RB] = ( xForce/(4*sin_a0) - yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_LB] = ( -xForce/(4*sin_a0) - yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;
	power[wheels_LF] = ( -xForce/(4*sin_a0) + yForce/(4*cos_a0) - robotTorque/(4*ROBOT_RADIUS) ) * WHEEL_RADIUS;

}

void wheels_SetOutput(float power[4]){
	switch(wheels_state){
	bool prev_reverse[4];
	bool delay = false;
	case wheels_uninitialized:
		uprintf("ERROR wheels_uninitialized\n\r");
		return;
	case wheels_ready:
		memcpy(prev_reverse, reverse, 4);
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


		for(wheels_handles wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			pwm[wheel] = power[wheel] / 100 * MAX_PWM;
		}
		if(delay){
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

			wheels_state = wheels_first_brake_period;
			HAL_TIM_Base_Stop(&htim14);											// Stop timer
			__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim14, 0);									// Clear timer
			__HAL_TIM_SET_AUTORELOAD(&htim14, 2000);
			HAL_TIM_Base_Start_IT(&htim14);   									// Start timer for kick off
			//HAL_Delay(1);

		}else{
			HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse[wheels_RF]);
			HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse[wheels_RB]);
			HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse[wheels_LB]);
			HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse[wheels_LF]);

			__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, pwm[wheels_RF]);
			__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, pwm[wheels_RB]);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm[wheels_LB]);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm[wheels_LF]);
		}
		return;
	default:
		break;
	}
}

inline int16_t wheels_GetEncoder(wheels_handles wheel){
	switch(wheel){
	case wheels_RF:
		return __HAL_TIM_GET_COUNTER(&htim8);
	case wheels_RB:
		return __HAL_TIM_GET_COUNTER(&htim1);
	case wheels_LB:
		return __HAL_TIM_GET_COUNTER(&htim3);
	case wheels_LF:
		return __HAL_TIM_GET_COUNTER(&htim4);
	default:
		return 0;
	}
}
float wheels_GetSpeed(wheels_handles wheel){
	static uint prev_time[4];
	int16_t counts;
	uint useconds;
	float seconds;
	float RPS;
	float motor_rotations;
	float wheel_rotations;
	counts = wheels_GetEncoder(wheel);
	ResetEncoder(wheel);
	useconds = __HAL_TIM_GET_COUNTER(&htim5) - prev_time[wheel];
	prev_time[wheel] = __HAL_TIM_GET_COUNTER(&htim5);
	seconds = (float)useconds/HTIM5_FREQ;
	motor_rotations = counts / (float)(PULSES_PER_ROTATION * X__ENCODING);
	wheel_rotations = motor_rotations/GEAR_RATIO;
	RPS = wheel_rotations / seconds;
	return RPS * 2 * M_PI;
}

void wheels_Callback(){
	switch(wheels_state){
	case wheels_uninitialized:
		uprintf("ERROR wheels_uninitialized\n\r");
		return;
	case wheels_ready:
		uprintf("ERROR wheels_ready\n\r");
		return;
	case wheels_first_brake_period:
		wheels_state = wheels_second_brake_period;
		return;
	case wheels_second_brake_period:
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse[wheels_RF]);
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse[wheels_RB]);
		HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse[wheels_LB]);
		HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse[wheels_LF]);
		wheels_state = wheels_third_brake_period;
		return;
	case wheels_third_brake_period:
		HAL_TIM_Base_Stop(&htim14);											// Stop timer
		__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim14, 0);									// Clear timer
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, pwm[wheels_RF]);
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, pwm[wheels_RB]);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm[wheels_LB]);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm[wheels_LF]);
		wheels_state = wheels_ready;
		return;
	}
}
