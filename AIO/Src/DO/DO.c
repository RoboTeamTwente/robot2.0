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
#include "control_util.h"

static float xsensData[3];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void getXsensData(float xsensData[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float wheel_ref[4]){
	// get xsens data
	getXsensData(xsensData);

	// calibration of xsens data by calculating yaw offset
	calibrateXsens(xsensData, vision_yaw, vision_available);

	static float State[3] = {0};
	State[2] = xsensData[2];
	//estimateState(State, xsensData);

	// control part
	vel_control_Callback(wheel_ref, State, velocityRef);
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
