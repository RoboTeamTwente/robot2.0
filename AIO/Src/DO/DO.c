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
#include "vel_control.h"

uint start_time;
static float xsensData[3];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//scales and limit the signal
static void scaleAndLimit(float wheel_ref[4]);

static void wheelFilter(float w_wheels[4]);

static void getXsensData(float xsensData[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return 0;
}

void DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float output[4]){
	// get and filter wheel speeds
	float w_wheels[4] = {0,0,0,0};
	wheelFilter(w_wheels);//edits the above array

	// get and offset xsens data
	getXsensData(xsensData);

	// calibration of xsens data by calculating yaw offset
	calibrateXsens(xsensData, vision_yaw, vision_available);

	// control part
	vel_control_Callback(output, xsensData, velocityRef);
}

float getYaw() {
	return xsensData[2];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void getXsensData(float xsensData[3]){

	xsensData[body_x] = -MT_GetAcceleration()[0]; // TODO: why the minus signs?
	xsensData[body_y] = -MT_GetAcceleration()[1];
	xsensData[body_w] = MT_GetAngles()[2]/180*M_PI;

}

static void wheelFilter(float w_wheels[4]){
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

static void scaleAndLimit(float wheel_ref[4]){
	//TODO: add some limitation stuff here
}
