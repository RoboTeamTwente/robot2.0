/*
 * control.c
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */

#include <stdio.h>
#include <math.h>
#include "DO.h"
#include "tim.h"
#include "../wheels/wheels.h"
#include "../MTi/MTiControl.h"
#include "yawCalibration.h"

///////////////////////////////////////////////////// DEFINITIONS
// Basically set constants
#define CALIBRATE_AFTER 6500 // time after which yaw calibration will start (ms)
#define rad_robot 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.0275F 	// wheel radius (m)
#define cos60 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define sin60 0.866F	// sine of 60 degrees

uint start_time;

PIDvariables angleK = {
		.kP = 2,//kp
		.kI = 0,//ki
		.kD = 0.5,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velxK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velyK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};


///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

//scales and limit the signal
static void scaleAndLimit(float wheel_ref[4]);

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x);

static void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]);

static void wheelFilter(float w_wheels[4]);

static float* getXsensData();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return 0;
}

void DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float output[4]){
	// get and filter wheel speeds
//	float w_wheels[4] = {0,0,0,0};
//	wheelFilter(w_wheels);//edits the above array

	// get and offset xsens data
	float* xsensData = getXsensData();

	// calibration of xsens data by calculating yaw offset
	calibrateXsens(xsensData, vision_yaw, vision_available);

	// control part
	vel_control_Callback(output, xsensData, velocityRef);
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

float* getXsensData(){

	float xsensData[3];
	xsensData[body_x] = -MT_GetAcceleration()[0]; // TODO: why the minus signs?
	xsensData[body_y] = -MT_GetAcceleration()[1];
	xsensData[body_w] = MT_GetAngles()[2]/180*M_PI;

	return xsensData;
}

void wheelFilter(float w_wheels[4]){
	// get and filter wheel speeds
	static float w_prev[4] = {0,0,0,0};
		 // filtering wheel speeds
		for(wheels_handles i = wheels_RF; i <= wheels_LF; i++){
			if (!isnan(w_prev[i])){
				//TODO Test ratio
				w_wheels[i] = 0.4*(-wheels_GetSpeed(i)) + 0.6*w_prev[i];
			} else {
				w_wheels[i] = -wheels_GetSpeed(i);
			}
			w_prev[i] = w_wheels[i];
		}
}

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]){


	float angleErr = constrainAngle((vel_ref[body_w] - State[body_w]));//constrain it to one circle turn
	float angleComp = PID(angleErr, &angleK);// PID control from control_util.h
	static float velLocalRef[3] = {0};
	global2Local(vel_ref, velLocalRef, State[body_w]); //transfer global to local

	// PID control from control_util.h
	velLocalRef[body_x] += PID((velLocalRef[body_x]-State[body_x]), &velxK); //error compensation plus requested velocity
	velLocalRef[body_y] += PID((velLocalRef[body_y]-State[body_y]), &velyK);

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

	for (int i = 0; i < 4; ++i){
		wheel_ref[i] += angleComp; //add necessary rotation value
	}

	return;
}

static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	//TODO check minuses
	float velx2wheel = (sin60*velocity[body_x]/rad_wheel);
	float vely2wheel = (cos60*velocity[body_y]/rad_wheel);
	float rot2wheel =  (rad_robot*velocity[body_w]/rad_wheel);
	wheelspeed[wheels_RF] = -(velx2wheel + vely2wheel);
	wheelspeed[wheels_RB] = -(velx2wheel - vely2wheel);
	wheelspeed[wheels_LB] = -(-velx2wheel - vely2wheel);
	wheelspeed[wheels_LF] = -(-velx2wheel + vely2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cos(yaw)*global[body_x]-sin(yaw)*global[body_y];
	local[body_y] = sin(yaw)*global[body_x]+cos(yaw)*global[body_y];
}

static void scaleAndLimit(float wheel_ref[4]){
	//TODO: add some limitation stuff here
}

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
