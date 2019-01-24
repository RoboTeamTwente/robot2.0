/*
 * wheels.c
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

#include "wheels.h"
#include <stdio.h>
#include <math.h>
#include "tim.h"
#include "stdbool.h"


///////////////////////////////////////////////////// DEFINITIONS


#define PWM_CUTOFF 240.0F			// arbitrary treshold below PWM_ROUNDUP
#define PWM_ROUNDUP 250.0F 		// below this value the motor driver is unreliable

#define gearratio 2.5f
#define max_pwm 2400
#define max_voltage 12//see datasheet
#define speedConstant 374/60 //[RPS/V] see datasheet
#define RPStoPWM (float)(1/speedConstant)*(max_pwm/max_voltage)*gearratio // [pwm/RPS]


///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void SetPWM(wheel_names wheel);

static void SetDir(wheel_names wheel);

static void getEncoderData(int encoderdata[4]);

static float deriveEncoder(int encoderData, int prev_encoderData);

static void limitScale(int output[4], int pwm[4], bool direction[4]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void wheelsInit(){
	wheels_state = wheels_ready;
	HAL_TIM_Base_Start(&htim1); //RF
	HAL_TIM_Base_Start(&htim8); //RB
	HAL_TIM_Base_Start(&htim3); //LB
	HAL_TIM_Base_Start(&htim4); //LF
	HAL_TIM_Base_Start(&htim5); //TIME
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); //RF
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); //RB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); //LB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); //LF
}

void wheelsDeInit(){
	wheels_state = wheels_uninitialized;
	HAL_TIM_Base_Stop(&htim1); //RF
	HAL_TIM_Base_Stop(&htim8); //RB
	HAL_TIM_Base_Stop(&htim3); //LB
	HAL_TIM_Base_Stop(&htim4); //LF
	HAL_TIM_Base_Stop(&htim5); //TIME
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2); //RF
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1); //RB
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1); //LB
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2); //LF
}

void setWheelSpeed(float wheelref[4]){

	//TODO: add braking for rapid switching direction
	//TODO: Add slipping case
	/* dry testing
	wheelref[0] = 0;
	wheelref[1] = 0;
	wheelref[2] = 0;
	wheelref[3] = 0;
	*/
	static int prev_state[4] = {0};
	int state[4] = {0};
	int output[4] = {0};
	float err[4] = {0};
	switch(wheels_state){
		default:
			break;
		case wheels_ready:

			//get encoder data
			getEncoderData(state);

			//derive wheelspeed
			for(int i = wheels_RF; i <= wheels_LF; i++){
				wheelspeed[i] = deriveEncoder(state[i], prev_state[i]);
				err[i] = wheelref[i]-wheelspeed[i];
				prev_state[i] = state[i];
			}

			//combine reference and PID
			output[wheels_RF] = wheelref[wheels_RF] + PID(err[wheels_RF], &RFK);
			output[wheels_RB] = wheelref[wheels_RB] + PID(err[wheels_RB], &RBK);
			output[wheels_LB] = wheelref[wheels_LB] + PID(err[wheels_LB], &LBK);
			output[wheels_LF] = wheelref[wheels_LF] + PID(err[wheels_LF], &LFK);

			limitScale(output, pwm, direction);
			for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
				SetDir(wheel);
				SetPWM(wheel);
			}
			break;
		}
	return;
}

float getWheelSpeed(wheel_names wheel) {
	return wheelspeed[wheel];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void wheelsCallback() {
	uint8_t cnt = 0;
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		switch(brake_state[wheel]){
		case no_brake:
			cnt++;
			break;
		case first_brake_period:
			brake_state[wheel] = second_brake_period;
			__HAL_TIM_SET_AUTORELOAD(&htim14, 1500);
			break;
		case second_brake_period:
			direction[wheel] = !direction[wheel]; // reverse direction
			SetDir(wheel);
			brake_state[wheel] = third_brake_period;
			__HAL_TIM_SET_AUTORELOAD(&htim14, 1500);
			break;
		case third_brake_period:
			cnt++;
			SetPWM(wheel);
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

static void limitScale(int output[4], int pwm[4], bool direction[4]){

	//Convert to PWM
	for(int i = wheels_RF; i <= wheels_LF; i++){
		output[i] = RPStoPWM*output[i];
	}
	//Limit
	for(int i = wheels_RF; i <= wheels_LF; i++){
		if(output[i] <= -1.0F){
			pwm[i] = -output[i];
			direction[i] = 1;
		}else if(output[i] >= 1.0F){
			pwm[i] = output[i];
			direction[i] = 0;
		}
		else {
			pwm[i] = 0.0F;
		}
		if(pwm[i] < PWM_CUTOFF){
			pwm[i] = 0.0F;
		}else if(pwm[i] < PWM_ROUNDUP){
			pwm[i] = PWM_ROUNDUP;
		}else if(pwm[i] > max_pwm){
			pwm[i] = max_pwm;
		}
	}
}

static void getEncoderData(int encoderData[4]){
	encoderData[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim1);
	encoderData[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim8);
	encoderData[wheels_LB] = -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	encoderData[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim4);
}

static float deriveEncoder(int encoderData, int prev_encoderData){
	float wheel_speed = ((encoderData-prev_encoderData)/TIME_DIFF)/(gearratio*1024*4);
	return wheel_speed;
}

static void SetPWM(wheel_names wheel){
	switch (wheel) {
	case wheels_RF:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, pwm[wheels_RF]);
		break;
	case wheels_RB:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, pwm[wheels_RB]);
		break;
	case wheels_LB:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm[wheels_LB]);
		break;
	case wheels_LF:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm[wheels_LF]);
		break;
	}
}

static void SetDir(wheel_names wheel){
	switch(wheel) {
	case wheels_RF:
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, direction[wheels_RF]);
		break;
	case wheels_RB:
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, direction[wheels_RB]);
			break;
	case wheels_LB:
			HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, direction[wheels_LB]);
			break;
	case wheels_LF:
			HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, direction[wheels_LF]);
			break;
	}
}
