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
#include "../DO/control_util.h"

///////////////////////////////////////////////////// VARIABLES

static int wheels_state = wheels_uninitialized;
static int pwm[4] = {0};
static bool direction[4] = {0}; // 0 is counter clock-wise TODO:confirm
static float wheelspeed[4] = {0};
static PIDvariables wheelsK[4];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void SetPWM(wheel_names wheel);
static void SetDir(wheel_names wheel);
static short int getEncoderData(wheel_names wheel);
static void ResetEncoder(wheel_names wheel);
static float computeWheelSpeed(wheel_names wheel);
static void limitScale(wheel_names wheel);
static void initPID(float kP, float kI, float kD);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

// Initialize wheels
void wheelsInit(){
	initPID(5.0, 0.0, 0);
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
	if (wheels_state == wheels_ready) {
		float err[4] = {0};
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			wheelspeed[wheel] = computeWheelSpeed(wheel);
			err[wheel] = wheelref[wheel]-wheelspeed[wheel];

			int output = wheelref[wheel] + PID(err[wheel], &wheelsK[wheel]); // add PID to wheels reference angular velocity
			pwm[wheel] = OMEGAtoPWM*output; // convert to pwm

			limitScale(wheel);
			SetDir(wheel);
			SetPWM(wheel);
		}
	}
}

// Get the current wheel speed in radians per second
float getWheelSpeed(wheel_names wheel) {
	return wheelspeed[wheel];
}

// Get the current PWM that is sent to the wheels
int getPWM(wheel_names wheel) {
	return pwm[wheel];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

// Set PID values
static void initPID(float kP, float kI, float kD) {
	wheelsK[0] = RFK;
	wheelsK[1] = RBK;
	wheelsK[2] = LBK;
	wheelsK[3] = LFK;

	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		wheelsK[wheel].kP = kP;
		wheelsK[wheel].kI = kI;
		wheelsK[wheel].kD = kD;
	}
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
		pwm[wheel] = 0; // the motor does not brake if pwm 0 is sent
	}
	// Limit PWM
	if(pwm[wheel] < PWM_CUTOFF){
		pwm[wheel] = 0.0F;
	} else if(pwm[wheel] > PWM_LIMIT){
		pwm[wheel] = PWM_LIMIT;
	}
}

// Get the current encoder data for all wheels
static short int getEncoderData(wheel_names wheel){
	// NOTE: RF and RB are swapped to match with wheel reference
	switch (wheel) {
	case wheels_RF:
		return __HAL_TIM_GET_COUNTER(&htim8);
	case wheels_RB:
		return __HAL_TIM_GET_COUNTER(&htim1);
	case wheels_LB:
		return -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	case wheels_LF:
		return __HAL_TIM_GET_COUNTER(&htim4);
	default:
		return 0;
	}
}

// Set motor encoder to zero
static void ResetEncoder(wheel_names wheel) {
	// NOTE: RF and RB are swapped to match with wheel reference
	switch (wheel) {
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

// Compute wheel speed for a certain wheel in radians per second
static float computeWheelSpeed(wheel_names wheel){
	short int encoderData = getEncoderData(wheel);
	float wheelSpeed = ENCODERtoOMEGA * (encoderData);
	ResetEncoder(wheel);
	return wheelSpeed;
}

// Set PWM to the motor
static void SetPWM(wheel_names wheel){
	switch (wheel) {
	case wheels_RF:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, MAX_PWM-pwm[wheels_RF]);
		break;
	case wheels_RB:
		__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, MAX_PWM-pwm[wheels_RB]);
		break;
	case wheels_LB:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, MAX_PWM-pwm[wheels_LB]);
		break;
	case wheels_LF:
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, MAX_PWM-pwm[wheels_LF]);
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
