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
	wheelFilter(wheelSpeeds);

	// Transform to velocities
	float velocities[3];
	wheels2Body(wheelSpeeds, velocities);

	// Put into State variable
	State[body_x] = velocities[body_x];
	State[body_y] = velocities[body_y];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

//multiplies a 3*4 matrix by a vector of 4 elements.
void wheels2Body(float wheelSpeeds[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = (1/sin60*wheelSpeeds[wheels_RF] + 1/sin60*wheelSpeeds[wheels_RB] - 1/sin60*wheelSpeeds[wheels_LB] - 1/sin60*wheelSpeeds[wheels_LF])*rad_wheel/4;
	output[body_y] = (1/cos60*wheelSpeeds[wheels_RF] - 1/cos60*wheelSpeeds[wheels_RB] - 1/cos60*wheelSpeeds[wheels_LB] + 1/cos60*wheelSpeeds[wheels_LF])*rad_wheel/4;
	output[body_w] = -(-1/rad_robot*wheelSpeeds[wheels_RF] - 1/rad_robot*wheelSpeeds[wheels_RB] - 1/rad_robot*wheelSpeeds[wheels_LB] - 1/rad_robot*wheelSpeeds[wheels_LF])*rad_wheel/4;
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
