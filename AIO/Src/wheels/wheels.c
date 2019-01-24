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
#define ENCODERtoRPS (float)1/(TIME_DIFF*GEAR_RATIO*PULSES_PER_ROTATION) // conversion factor from number of encoder pulses to RPS of the wheel

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void SetPWM(wheel_names wheel);
static void SetDir(wheel_names wheel);
static int getEncoderData(wheel_names wheel);
static float computeWheelSpeed(wheel_names wheel);
static void limitScale(wheel_names wheel);
static bool directionSwitched(wheel_names wheel);
static void restartCallbackTimer();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

// Initialize wheels
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

// Deinitialize wheels
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

// Set the desired rotations per second for every wheel
void setWheelSpeed(float wheelref[4]){
	//TODO: Add slipping case
	float err[4] = {0};
	switch(wheels_state){
	case wheels_ready:
		//derive wheelspeed
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			wheelspeed[wheel] = computeWheelSpeed(wheel);
			err[wheel] = wheelref[wheel]-wheelspeed[wheel];

			int output = wheelref[wheel] + PID(err[wheel], &wheelsK[wheel]); // add PID to reference RPS
			pwm[wheel] = RPStoPWM*output; // convert to pwm

			limitScale(wheel);
			if (directionSwitched(wheel)) {
				brake_state[wheel] = first_brake_period;
				restartCallbackTimer();
			}
			SetDir(wheel);
			SetPWM(wheel);
		}
	}
}

// Make direction switch smooth by using multiple stages
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

// Get the current wheel speed in RPS
float getWheelSpeed(wheel_names wheel) {
	return wheelspeed[wheel];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

// Check if the direction switched for a certain wheel
static bool directionSwitched(wheel_names wheel) {
	static bool prevDirection[4] = {0};
	if (direction[wheel] != prevDirection[wheel]) {
		prevDirection[wheel] = direction[wheel];
		return true;
	}
	return false;
}

// Restart the timer that is set for wheelsCallback()
static void restartCallbackTimer() {
	HAL_TIM_Base_Stop(&htim14);
	__HAL_TIM_CLEAR_IT(&htim14,TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim14, 1500);
	HAL_TIM_Base_Start_IT(&htim14);
}

// Limit or scale the PWM such that it can be passed to the motors
static void limitScale(wheel_names wheel){
	// Determine direction
	if(pwm[wheel] <= -1.0F){
		pwm[wheel] *= -1;
		direction[wheel] = 1;
	} else if(pwm[wheel] >= 1.0F){
		direction[wheel] = 0;
	} else {
		pwm[wheel] = 0.0F;
	}
	// Limit PWM
	if(pwm[wheel] < PWM_CUTOFF){
		pwm[wheel] = 0.0F;
	} else if(pwm[wheel] < PWM_ROUNDUP){
		pwm[wheel] = PWM_ROUNDUP;
	} else if(pwm[wheel] > MAX_PWM){
		pwm[wheel] = MAX_PWM;
	}
}

// Get the current encoder data for all wheels
static int getEncoderData(wheel_names wheel){
	switch (wheel) {
	case wheels_RF:
		return __HAL_TIM_GET_COUNTER(&htim1);
	case wheels_RB:
		return __HAL_TIM_GET_COUNTER(&htim8);
	case wheels_LB:
		return -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	case wheels_LF:
		return __HAL_TIM_GET_COUNTER(&htim4);
	default:
		return 0;
	}
}

// Compute wheel speed for a certain wheel in RPS
static float computeWheelSpeed(wheel_names wheel){
	static int prevEncoderData[4] = {0};
	int encoderData = getEncoderData(wheel);
	float wheelSpeed = ENCODERtoRPS * (encoderData-prevEncoderData[wheel]);
	prevEncoderData[wheel] = encoderData;
	return wheelSpeed;
}

// Set PWM to the motor
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

// Set direction to the motor
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
