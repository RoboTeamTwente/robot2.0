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


//These structs speak for themselves. Theyre just here so I can return these from functions.
struct Vector3 {
	float x;
	float y;
	float w;
};

//ditto
struct Vector4 {
	float a;
	float b;
	float c;
	float d;
};



static float do_output[3];

DO_States DO_Init(){
	HAL_TIM_Base_Start_IT(&htim7);

	do_output[body_x] = 0;
	do_output[body_y] = 0;
	do_output[body_w] = 0;

	return DO_succes;
}


void disturbanceObserver(float localBodyReference[3], float localBodyVelocity[3], float output[3]){

	// Requires transformation to global coordinates and back

	output[body_x]= 0;
	output[body_y] = 0;
	output[body_w] = 0;								//Will be made into an actual disturbance observer now :)
}

//multiplies a 4*3 matrix by a vector of 3 elements.
void body2Wheels(float v[3], float output[4]){

	float c = 0.5;
	float s = 0.866;
	float R = 0.0775;
	float r = 0.0275;

	//Applying M_inv matrix.
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

void rotate(float yaw, float input[3], float output[3]){

	output[body_x] = cosf(yaw)*input[body_x] + sinf(yaw)*input[body_y];
	output[body_y] = -sinf(yaw)*input[body_x] + cosf(yaw)*input[body_y];
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
void controller(float velocityRef[3], float w_wheels[4], float xsensData[3], float output[4]){
	// uses global variable: do_output which is reset to 0 in the init function

	// Compute the error in local body coordinates
	float localReference[3];
	rotate(xsensData[body_w], velocityRef, localReference);

	float localVel[3];
	wheels2Body(w_wheels, localVel);
//	uprintf("[%f, %f, %f]\n\r", localVel[body_x], localVel[body_y],  localVel[body_w]);
	uprintf("[%f]\n\r", xsensData[body_w]);
	uprintf("[%f, %f, %f]\n\r", localReference[body_x], localReference[body_y],  localReference[body_w]);


	float error[3];
	error[body_x] = localReference[body_x] - localVel[body_x];
	error[body_y] = localReference[body_y] - localVel[body_y];
	error[body_w] = localReference[body_w] - localVel[body_w];

	// P-control
	float controllerGain[3];
	controllerGain[body_x] = 20000/30;//100;
	controllerGain[body_y] = 20000/30;///100;
	controllerGain[body_w] = 200/30;///100;
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
	scaledInput[body_w] = scale*postObserverSignal[body_w];

	// Output the wheel PWMs (by filling in the output array)
	body2Wheels(scaledInput, output);

	// Compute disturbance observer output for next iteration, filling in the DO array (do_output)
	disturbanceObserver(xsensData, scaledInput, do_output);

//	uprintf("[%f, %f]\n\r", w_wheels[wheels_RF], output[wheels_RF]);
	//uprintf("[%f, %f, %f, %f]\n\r", output[wheels_RF],  output[wheels_RB], output[wheels_LB], output[wheels_LF]);
	//uprintf("[%f, %f, %f, %f, %f, %f]\n\r", localVel[body_x], scaledInput[body_x], localVel[body_y], scaledInput[body_y], localVel[body_w], scaledInput[body_w]);
	//uprintf("[%f, %f, %f, %f, %f, %f]\n\r", localVel[body_x], error[body_x], localVel[body_y], error[body_y], localVel[body_w], error[body_w]);
}


DO_States DO_Control(float velocityRef[3], float xsensData[3]){

//	velocityRef[0] = 0;
//	velocityRef[1] = 1;
	static float w_prev[4] = {0,0,0,0};

	float w_wheels[4];
	if (!isnan(w_prev[0])){ // filtering
		w_wheels[wheels_RF] = 0.4*(-wheels_GetSpeed(wheels_RF)) + 0.6*w_prev[wheels_RF];
		w_wheels[wheels_RB] = 0.4*(-wheels_GetSpeed(wheels_RB)) + 0.6*w_prev[wheels_RB];
		w_wheels[wheels_LB] = 0.4*(-wheels_GetSpeed(wheels_LB)) + 0.6*w_prev[wheels_LB];
		w_wheels[wheels_LF] = 0.4*(-wheels_GetSpeed(wheels_LF)) + 0.6*w_prev[wheels_LF];
	} else {
		w_wheels[wheels_RF] = (-wheels_GetSpeed(wheels_RF));
		w_wheels[wheels_RB] = (-wheels_GetSpeed(wheels_RB));
		w_wheels[wheels_LB] = (-wheels_GetSpeed(wheels_LB));
		w_wheels[wheels_LF] = (-wheels_GetSpeed(wheels_LF));
	}

	w_prev[wheels_RF] = w_wheels[wheels_RF];
	w_prev[wheels_RB] = w_wheels[wheels_RB];
	w_prev[wheels_LB] = w_wheels[wheels_LB];
	w_prev[wheels_LF] = w_wheels[wheels_LF];

	float wheelsPWM[4];
	controller(velocityRef, w_wheels, xsensData, wheelsPWM);

	//uprintf("[%f, %f, %f, %f]\n\r", w_wheels[wheels_RF], w_wheels[wheels_RB],  w_wheels[wheels_LB], w_wheels[wheels_LF]);
	//uprintf("[%f, %f, %f]\n\r", velocityRef[body_x], velocityRef[body_y],  velocityRef[body_w]);

//	float wheelsPWM2[4] = {0,0,-0,-0};
//	wheelsPWM2[wheels_LB] = -(30*1.5 - w_wheels[wheels_LB])*0.0;
	wheels_SetOutput(wheelsPWM);

//	static float counter = 0;
//	counter = counter+0.01;
	//uprintf("[%f]\n\r", counter);
	return DO_error;
}
