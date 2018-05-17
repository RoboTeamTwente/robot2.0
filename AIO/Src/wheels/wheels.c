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

#define PWM_CUTOFF 5.00F		// arbitrary treshold below PWM_ROUNDUP
#define PWM_ROUNDUP 10.00F 		// below this value the motor driver is unreliable
#define ROBOT_RADIUS 0.0775F
#define WHEEL_RADIUS 0.0275F
#define HTIM5_FREQ 1000000.0F	//Hz
#define PULSES_PER_ROTATION 1024
#define GEAR_RATIO 2.5F			// motor to wheel
#define X__ENCODING 4			// counts per pulse X1, X2 or X4

bool reverse[N_WHEELS] = {0};
float global_power[N_WHEELS];

enum {
	wheels_uninitialized,
	wheels_ready,
	wheels_braking
}wheels_state;// keeps track of the state of this system

enum wheel_braking_states{
	no_brake,
	first_brake_period,
	second_brake_period,
	third_brake_period,
}brake_state[N_WHEELS];// keeps track of the braking state of each wheel

/*----------------------------------static functions-------------------------------*/
/*	Set an encoder to zero
 *
 */
static inline void ResetEncoder(wheels_handles wheel){
	switch(wheel){
	case wheels_RF:
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		break;
	case wheels_RB:
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		break;
	case wheels_LB:
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		break;
	case wheels_LF:
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		break;
	}
}
/*	Set Pwm for one wheel
 *
 */
static inline void SetPWM(wheels_handles wheel, float power){
	switch(wheel){
	case wheels_RF:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, power / 100 * MAX_PWM);
		break;
	case wheels_RB:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, power / 100 * MAX_PWM);
		break;
	case wheels_LB:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, power / 100 * MAX_PWM);
		break;
	case wheels_LF:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, power / 100 * MAX_PWM);
		break;
	}
}
/*	Set direction for one wheel
 *
 */
static inline void SetDir(wheels_handles wheel, bool reverse){
	switch(wheel){
	case wheels_RF:
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse);
		break;
	case wheels_RB:
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse);
		break;
	case wheels_LB:
		HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse);
		break;
	case wheels_LF:
		HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse);
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
void wheels_DeInit(){
	wheels_state = wheels_uninitialized;
	HAL_TIM_Base_Stop(&htim1);
	HAL_TIM_Base_Stop(&htim3);
	HAL_TIM_Base_Stop(&htim4);
	HAL_TIM_Base_Stop(&htim5);
	HAL_TIM_Base_Stop(&htim8);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
}

void calcMotorSpeeds (float magnitude, float direction, int rotSign, float wRadPerSec, float power[N_WHEELS]){
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

void wheels_SetOutput(float power[N_WHEELS]){
	switch(wheels_state){
	bool prev_reverse[N_WHEELS];
	bool delay[N_WHEELS] = {false};
	case wheels_uninitialized:
		uprintf("ERROR wheels_uninitialized\n\r");
		return;
	case wheels_ready:
		memcpy(prev_reverse, reverse, N_WHEELS);
		for(wheels_handles i = wheels_RF; i <= wheels_LF; i++){
			if(power[i] < -0.1 ){
				power[i] = -power[i];
				reverse[i] = 1;
			}else if(power[i] > 0.1 ){
				reverse[i] = 0;
			}
			if(power[i] > 1){
				power[i] = 10.0F + power[i] * (90.0F / 100.0F);
			}else{
				power[i] = 0;
			}
			if(power[i] > 100){
				power[i] = 100;
			}/*else if(power[i] < PWM_ROUNDUP){
				if(power[i] < PWM_CUTOFF) {
					power[i] = 0;
				} else {
					power[i] = PWM_ROUNDUP;
				}

			}*/
		}

		memcpy(global_power, power, 4 * sizeof(float));

		for(wheels_handles wheel = wheels_RF; wheel < N_WHEELS; wheel++){
			delay[wheel] = prev_reverse[wheel] != reverse[wheel];
		}


		for(wheels_handles wheel = wheels_RF; wheel < N_WHEELS; wheel++){
			if(delay[wheel]){
				SetPWM(wheel, 0);
				brake_state[wheel] = first_brake_period;
			}else{
				SetDir(wheel,reverse[wheel]);

				SetPWM(wheel, global_power[wheel]);
			}
		}
		if(delay[wheels_RF] || delay[wheels_RB] || delay[wheels_LB] || delay[wheels_LF]){
			HAL_TIM_Base_Stop(&htim14);											// Stop timer
			__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim14, 0);									// Clear timer
			__HAL_TIM_SET_AUTORELOAD(&htim14, 3000);
			HAL_TIM_Base_Start_IT(&htim14);
		}
		return;
	default:
		break;
	}
}

inline int16_t wheels_GetEncoder(wheels_handles wheel){
	switch(wheel){
	case wheels_RF:
		return __HAL_TIM_GET_COUNTER(&htim1);
	case wheels_RB:
		return __HAL_TIM_GET_COUNTER(&htim8);
	case wheels_LB:
		return -__HAL_TIM_GET_COUNTER(&htim3); //  minus due to inverted routing (right leon?)		 yes - Leon
	case wheels_LF:
		return __HAL_TIM_GET_COUNTER(&htim4);
	default:
		return 0;
	}
}

float wheels_GetSpeed(wheels_handles wheel){
	static uint prev_time[N_WHEELS];
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
	uint8_t cnt = 0;
	for(wheels_handles wheel = wheels_RF; wheel < N_WHEELS; wheel++){
		switch(brake_state[wheel]){
		case no_brake:
			cnt++;
			break;
		case first_brake_period:
			brake_state[wheel] = second_brake_period;
			//__HAL_TIM_SET_AUTORELOAD(&htim14, 8000/2);
			break;
		case second_brake_period:
			SetDir(wheel,reverse[wheel]);
			brake_state[wheel] = third_brake_period;
			//__HAL_TIM_SET_AUTORELOAD(&htim14, 1000);
			break;
		case third_brake_period:
			cnt++;
			SetPWM(wheel, global_power[wheel]);
			brake_state[wheel] = no_brake;
			break;
		}
	}
	if(cnt == 4){
		HAL_TIM_Base_Stop(&htim14);											// Stop timer
		__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim14, 0);									// Clear timer
	}
}
