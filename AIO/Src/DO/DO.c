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
#include "stateEstimation.h"

uint start_time;
static float xsensData[3];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void scaleAndLimit(float wheel_ref[4]); //scales and limit the signal
//static void wheelFilter(float w_wheels[4]);
static void getXsensData(float xsensData[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

// TODO: Koen what's the point of this? start_time is not used....
int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return 0;
}

void DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float wheel_ref[4]){
	// get and filter wheel speeds
//	float w_wheels[4] = {0,0,0,0};
//	wheelFilter(w_wheels);//edits the above array

	// get and offset xsens data
	getXsensData(xsensData);

	// calibration of xsens data by calculating yaw offset
	calibrateXsens(xsensData, vision_yaw, vision_available);

	static float State[3] = {0};
	estimateState(State, xsensData);

	// control part
	vel_control_Callback(wheel_ref, State, velocityRef);

	// limit and scale wheel outputs
	scaleAndLimit(wheel_ref);
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

static void scaleAndLimit(float wheel_ref[4]){
	// TODO: it doesn't work if you use the defined constants from control_util, go fix!
	float margin = 0.3; // give the PID a little space to adjust
	float rpsFactor = RPStoPWM;
	float rps_limit = PWM_LIMIT/rpsFactor;
	float rps_cutoff = PWM_CUTOFF/rpsFactor + margin;
	for (wheel_names i = wheels_RF; i <= wheels_LF; i++) {
		if (fabs(wheel_ref[i]) > rps_limit) {
			wheel_ref[i] = rps_limit*fabs(wheel_ref[i])/wheel_ref[i];
		} else if (fabs(wheel_ref[i]) < margin) {
			wheel_ref[i] = 0;
		} else if (fabs(wheel_ref[i]) < rps_cutoff) {
			wheel_ref[i] = rps_cutoff*fabs(wheel_ref[i])/wheel_ref[i];
		}
	}
}
