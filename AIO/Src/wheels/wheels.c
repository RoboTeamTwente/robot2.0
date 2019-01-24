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

#define GEAR_RATIO 2.5F // gear ratio between motor and wheel
#define MAX_PWM 2400
#define MAX_VOLTAGE 12//see datasheet
#define SPEED_CONSTANT 374/60 //[RPS/V] see datasheet
#define RPStoPWM (float)(1/SPEED_CONSTANT)*(MAX_PWM/MAX_VOLTAGE)*GEAR_RATIO // conversion factor from rotations per second of the wheel to required PWM on the motor
#define PULSES_PER_ROTATION (float)4*1024 // number of pulses of the encoder per rotation of the motor (see datasheet)
#define ENCODERtoSPEED (float)1/(TIME_DIFF*GEAR_RATIO*PULSES_PER_ROTATION) // conversion factor from number of encoder pulses to RPS of the wheel

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void SetPWM(wheel_names wheel);

static void SetDir(wheel_names wheel);

static void getEncoderData(int encoderdata[4]);

static float computeWheelSpeed(int encoderData, int prev_encoderData);

static void limitScale(int output, wheel_names wheel);

static bool directionSwitched(wheel_names wheel);

static void restartCallbackTimer();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void wheelsInit(){
	wheels_state = wheels_ready;
	wheelsK[0] = RFK;
	wheelsK[1] = RBK;
	wheelsK[2] = LBK;
	wheelsK[3] = LFK;
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

	//TODO: Add slipping case
	/* dry testing
	wheelref[0] = 0;
	wheelref[1] = 0;
	wheelref[2] = 0;
	wheelref[3] = 0;
	 */
	static int prev_state[4] = {0};
	int state[4] = {0};
	float err[4] = {0};
	switch(wheels_state){
	case wheels_ready:
		getEncoderData(state);

		//derive wheelspeed
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			wheelspeed[wheel] = computeWheelSpeed(state[wheel], prev_state[wheel]);
			err[wheel] = wheelref[wheel]-wheelspeed[wheel];
			prev_state[wheel] = state[wheel];

			int output = wheelref[wheel] + PID(err[wheel], &wheelsK[wheel]); // add PID to reference RPS

			limitScale(output, wheel);
			if (directionSwitched(wheel)) {
				brake_state[wheel] = first_brake_period;
				restartCallbackTimer();
			}
			SetDir(wheel);
			SetPWM(wheel);
		}
		break;
	default:
		break;
	}
}

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
	// TODO: why at cnt == 4 ?
	// TODO: can restartCallbackTimer() be used here?
	if(cnt == 4){
		HAL_TIM_Base_Stop(&htim14);
		__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim14, 0);
	}
}

float getWheelSpeed(wheel_names wheel) {
	return wheelspeed[wheel];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static bool directionSwitched(wheel_names wheel) {
	static bool prevDirection[4] = {0};
	if (direction[wheel] != prevDirection[wheel]) {
		prevDirection[wheel] = direction[wheel];
		return true;
	}
	return false;
}

static void restartCallbackTimer() {
	HAL_TIM_Base_Stop(&htim14);
	__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim14, 1500);
	HAL_TIM_Base_Start_IT(&htim14);
}

static void limitScale(int output, wheel_names wheel){
	output *= RPStoPWM;
	if(output <= -1.0F){
		pwm[wheel] = -output;
		direction[wheel] = 1;
	} else if(output >= 1.0F){
		pwm[wheel] = output;
		direction[wheel] = 0;
	} else {
		pwm[wheel] = 0.0F;
	}
	if(pwm[wheel] < PWM_CUTOFF){
		pwm[wheel] = 0.0F;
	} else if(pwm[wheel] < PWM_ROUNDUP){
		pwm[wheel] = PWM_ROUNDUP;
	} else if(pwm[wheel] > MAX_PWM){
		pwm[wheel] = MAX_PWM;
	}
}

static void getEncoderData(int encoderData[4]){
	encoderData[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim1);
	encoderData[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim8);
	encoderData[wheels_LB] = -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	encoderData[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim4);
}

static float computeWheelSpeed(int encoderData, int prev_encoderData){
	return ENCODERtoSPEED * (encoderData-prev_encoderData);
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
