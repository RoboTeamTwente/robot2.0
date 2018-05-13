/*
 * DO.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include <stdio.h>
#include <math.h>
#include "DO.h"
#include "tim.h"
#include "../wheels/wheels.h"
#include "../PuttyInterface/PuttyInterface.h"
#include "../MTi/MTiControl.h"


static float do_output[3];

DO_States DO_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	MT_SetFilterProfile(2); // dynamic profile for xsens to keep up with fast rotation

	do_output[body_x] = 0;
	do_output[body_y] = 0;
	do_output[body_w] = 0;

	return DO_succes;
}

float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

void disturbanceObserver(float yaw, float localInput[3], float globalAcc[2], float output[3]){

	// Requires transformation to global coordinates and back
	float globalInput[3];
	rotate(-yaw, localInput, globalInput);
	// accelerations are already global (free acceleration)
	float accX = globalAcc[body_x];
	float accY = globalAcc[body_y];
//	if (fabs(accX)<0.3) {
//		accX = 0;
//	}
//	if (fabs(accY)<0.3) {
//		accY=0;
//	}

	float globalOut[3] = {0,0,0};
	static float prevOut[3] = {0,0,0};
	if (!isnan(prevOut[0])){ // lowpass filtering
		globalOut[body_x] = 0.07*(accX*3000 - globalInput[body_x]) + 0.93*prevOut[body_x];
		globalOut[body_y] = 0.07*(accY*3000 - globalInput[body_y]) + 0.93*prevOut[body_y];
	}
	prevOut[body_x] = globalOut[body_x];
	prevOut[body_y] = globalOut[body_y];

	// safety: reduce effect, while testing
	globalOut[body_x] = globalOut[body_x]*0.25;
	globalOut[body_y] = globalOut[body_y]*0.25;

	// rotate back to local and fill in the output
	rotate(yaw, globalOut, output);
}

//multiplies a 4*3 matrix by a vector of 3 elements.
void body2Wheels(float v[3], float output[4]){

	float c = 0.5;
	float s = 0.866;
	float R = 0.0775;
	float r = 0.0275;

	//Applying M_inv matrix. (minus sign in front is due to wheels spinning in opposite direction to motors)
	output[wheels_RF] = -(1/s*v[body_x] + 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_RB] = -(1/s*v[body_x] - 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_LB] = -(-1/s*v[body_x] - 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_LF] = -(-1/s*v[body_x] + 1/c*v[body_y] - 1/R*v[body_w])*r/4;
}

//multiplies a 3*4 matrix by a vector of 4 elements.
void wheels2Body(float w[4], float output[3]){

	float c = 0.5;
	float s = 0.866;
	float R = 0.0775;
	float r = 0.0275;

	//Applying transpose(M_inv) matrix
	output[body_x] = (1/s*w[wheels_RF] + 1/s*w[wheels_RB] - 1/s*w[wheels_LB] - 1/s*w[wheels_LF])*r/4;
	output[body_y] = (1/c*w[wheels_RF] - 1/c*w[wheels_RB] - 1/c*w[wheels_LB] + 1/c*w[wheels_LF])*r/4;
	output[body_w] = (-1/R*w[wheels_RF] - 1/R*w[wheels_RB] - 1/R*w[wheels_LB] - 1/R*w[wheels_LF])*r/4;
}

// rotates coordinate system by yaw
void rotate(float yaw, float input[3], float output[3]){

	float outX = cosf(yaw)*input[body_x] + sinf(yaw)*input[body_y];
	float outY = -sinf(yaw)*input[body_x] + cosf(yaw)*input[body_y];

	output[body_x] = outX;
	output[body_y] = outY;
	output[body_w] = input[body_w];

}

void pController(float input[3], float kp[3], float output[3]){
	// These limits are meant to prevent slipping
	float w_limit = 500;
	float PWM_limit = 50;

	float pre_out[3];
	pre_out[body_x] = kp[body_x]*input[body_x];
	pre_out[body_y] = kp[body_y]*input[body_y];
	pre_out[body_w] = kp[body_w]*input[body_w];

	float scale = compute_limit_scale(pre_out, PWM_limit);

	if (pre_out[body_w] > w_limit) {
		pre_out[body_w] = w_limit;
	} else if (pre_out[body_w] < -w_limit) {
		pre_out[body_w] = -w_limit;
	}

	output[body_x] = pre_out[body_x] * scale;
	output[body_y] = pre_out[body_y] * scale;
	output[body_w] = pre_out[body_w];

}

float compute_limit_scale(float input[3], float limit){

	float scale;
	float intermediaryOutput[4];
	body2Wheels(input, intermediaryOutput);
	float maxEl = fmax(fmax(intermediaryOutput[wheels_RF],intermediaryOutput[wheels_RB]),fmax(intermediaryOutput[wheels_LB],intermediaryOutput[wheels_LF]));

	if (fabs(maxEl) > limit){
		scale = limit/fabs(maxEl);
	}
	else {
		scale = 1;
	}

	return scale;
}

//observer output should just be pre-declared as 0 before any looping starts
void controller(float localVelocityRef[3], float w_wheels[4], float xsensData[3], bool DO_enabled, float output[4]){
	// uses global variable: do_output which is reset to 0 in the init function

	// Compute the error in local body coordinates
	float localVel[3];
	wheels2Body(w_wheels, localVel);
	float error[3];
	error[body_x] = localVelocityRef[body_x] - localVel[body_x];
	error[body_y] = localVelocityRef[body_y] - localVel[body_y];
	error[body_w] = localVelocityRef[body_w] - localVel[body_w];
	//	uprintf("[%f, %f, %f]\n\r", localVel[body_x], localVel[body_y],  localVel[body_w]);
	//	uprintf("[%f %f %f]\n\r", xsensData[body_x], xsensData[body_y], xsensData[body_w]);
		//uprintf("[%f, %f, %f]\n\r", localReference[body_x], localReference[body_y],  localReference[body_w]);

	// P-control
	float controllerGain[3];
	controllerGain[body_x] = 20000/10;
	controllerGain[body_y] = 20000/10;
	controllerGain[body_w] = 200/10;
	float pOut[3];
	pController(error, controllerGain, pOut);

	// Apply the Disturbance Observer output
	float postObserverSignal[3];
	postObserverSignal[body_x] = pOut[body_x] - do_output[body_x];
	postObserverSignal[body_y] = pOut[body_y] - do_output[body_y];
	postObserverSignal[body_w] = pOut[body_w] - do_output[body_w];

	// Limiting the output to prevent saturation of the PWM signals for any of the wheels
	float scaledInput[3];
	float scale = compute_limit_scale(postObserverSignal, 95);
	scaledInput[body_x] = scale*postObserverSignal[body_x];
	scaledInput[body_y] = scale*postObserverSignal[body_y];
	scaledInput[body_w] = 1*postObserverSignal[body_w]; // rotation gets some extra room by not scaling it down

	// Output the wheel PWMs (by filling in the output array)
	body2Wheels(scaledInput, output);

	// To make sure the robot does not do anything due to the DO when wheels are stationary and acceleration is measured
	float accData[2] = {xsensData[body_x],xsensData[body_y]};
	if (sqrt(localVel[body_x]*localVel[body_x] + localVel[body_y]*localVel[body_y])<0.05) {
		accData[body_x] = 0;
		accData[body_y] = 0;
	}
	// Compute disturbance observer output for next iteration, filling in the DO array (do_output)
	if (DO_enabled) {
		disturbanceObserver(xsensData[body_w], scaledInput, accData, do_output);
	}

//	uprintf("[%f, %f]\n\r", w_wheels[wheels_RF], output[wheels_RF]);
//	uprintf("[%f, %f, %f, %f]\n\r", output[wheels_RF],  output[wheels_RB], output[wheels_LB], output[wheels_LF]);
//	uprintf("[%f, %f, %f, %f, %f, %f]\n\r", localVel[body_x], scaledInput[body_x], localVel[body_y], scaledInput[body_y], localVel[body_w], scaledInput[body_w]);
//	uprintf("[%f, %f, %f, %f, %f, %f]\n\r", localVel[body_x], error[body_x], localVel[body_y], error[body_y], localVel[body_w], error[body_w]);
}

float angleController(float angleRef, float yaw){
	float angleError = constrainAngle(angleRef - yaw);
	uprintf("[%f, %f, %f]\n\r", angleRef, yaw, angleError);
	float output = -angleError*20.0;
	float limit = 20;
	if (output > limit) {
		output = limit;
	} else if (output < -limit) {
		output = -limit;
	}
	return output;
}


DO_States DO_Control(float velocityRef[3], float xsensData[3], bool DO_enabled, bool useYawControl, bool refIsAngle, float output[4]){
	//TODO: ADD ANGULAR TO DISTURBANCE OBSERVER

//	velocityRef[0] = 1;
//	velocityRef[1] = 0;
//	velocityRef[2] = 0;
//	xsensData[2] = 0;

	static float w_prev[4] = {0,0,0,0};
	float w_wheels[4];
	 // filtering wheel speeds
	for(wheels_handles i = wheels_RF; i <= wheels_LF; i++){
		if (!isnan(w_prev[i])){
			w_wheels[i] = 0.4*(-wheels_GetSpeed(i)) + 0.6*w_prev[i];
		} else {
			w_wheels[i] = -wheels_GetSpeed(i);
		}
		w_prev[i] = w_wheels[i];
	}

	// yaw controller
	if (useYawControl) {
		float newVelocityRef[3] = {velocityRef[body_x], velocityRef[body_y], 0};
		float angleRef;
		if (refIsAngle) { // the joystick could be used here to directly set the angle reference
			angleRef = velocityRef[body_w]*180/2048;
		} else { // for the keyboard the angle reference is ramped up by integrating the signal
			static float prevAngleRef = 0;
			angleRef = prevAngleRef + 0.01 * velocityRef[body_w];
			prevAngleRef = angleRef;
		}
		newVelocityRef[body_w] = angleController(angleRef, xsensData[body_w]);
		controller(newVelocityRef, w_wheels, xsensData, DO_enabled, output);
	} else {
		controller(velocityRef, w_wheels, xsensData, DO_enabled, output);
	}

//	uprintf("[%f, %f, %f, %f]\n\r", w_wheels[wheels_RF], w_wheels[wheels_RB],  w_wheels[wheels_LB], w_wheels[wheels_LF]);
//	uprintf("[%f, %f, %f]\n\r", velocityRef[body_x], velocityRef[body_y],  velocityRef[body_w]);
//	static float counter = 0;
//	counter = counter+0.01;
	//uprintf("[%f]\n\r", counter);


	return DO_error;
}
