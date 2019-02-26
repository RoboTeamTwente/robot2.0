/*
 * stateEstimation.c
 *
 *  Created on: Jan 25, 2019
 *      Author: simen
 */

#include "stateEstimation.h"

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

void wheels2Body(float wheelSpeeds[4], float output[3]);
void wheelFilter(float wheelSpeeds[4]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void estimateState(float State[3], float xsensData[3]) {
	// Integrate acceleration to get velocity
//	State[0] += xsensData[0] * TIME_DIFF;
//	State[1] += xsensData[1] * TIME_DIFF;

	// Get wheel speeds
	float wheelSpeeds[4];
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		wheelSpeeds[wheel] = getWheelSpeed(wheel);
	}
	//wheelFilter(wheelSpeeds);

	// Transform to velocities
	wheels2Body(wheelSpeeds, velocities);

	// Put data into State variable
	State[body_x] = velocities[body_x];
	State[body_y] = velocities[body_y];
	State[body_w] = xsensData[body_w];
}

void getvel(float Vel[2]){
	for (int i=0; i<2; i++){
		Vel[i] = velocities[i];
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

//multiplies a 3*4 matrix by a vector of 4 elements.
void wheels2Body(float wheelSpeeds[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = -(sin60*wheelSpeeds[wheels_RF] + sin60*wheelSpeeds[wheels_RB] - sin60*wheelSpeeds[wheels_LB] - sin60*wheelSpeeds[wheels_LF])*rad_wheel/4;
	output[body_y] = -(cos60*wheelSpeeds[wheels_RF] - cos60*wheelSpeeds[wheels_RB] - cos60*wheelSpeeds[wheels_LB] + cos60*wheelSpeeds[wheels_LF])*rad_wheel/4;
	output[body_w] = -(wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] + wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/rad_robot*rad_wheel/4;
}

//TODO: describe what this does
void wheelFilter(float wheelSpeeds[4]){
	// get and filter wheel speeds
	static float prevWheelSpeeds[4] = {0,0,0,0};
	// filtering wheel speeds
	for(wheel_names i = wheels_RF; i <= wheels_LF; i++){
		if (!isnan(prevWheelSpeeds[i])){
			//TODO Test ratio
			wheelSpeeds[i] = 0.4*(-getWheelSpeed(i)) + 0.6*prevWheelSpeeds[i];
		} else {
			wheelSpeeds[i] = -getWheelSpeed(i);
		}
		prevWheelSpeeds[i] = wheelSpeeds[i];
	}
}
