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


//struct Vector3 observerOutput;
//struct Vector3 *ptrdo = &observerOutput;
float observerOutput[3];

DO_States DO_Init(){
	HAL_TIM_Base_Start_IT(&htim7);

//	observerOutput.x = 0;
//	observerOutput.y = 0;
//	observerOutput.w = 0;

	return DO_succes;
}


void disturbanceObserver(float localBodyReference[3], float localBodyVelocity[3], float output[3]){

	output[body_x]= 0;
	output[body_y] = 0;
	output[body_w] = 0;								//Will be made into an actual disturbance observer now :)
}

//multiplies a 4*3 matrix by a vector of 3 elements.
void body2Wheels(float v[3], float output[4]){

	float s = 0.5;
	float c = 0.866;
	float R = 0.0775;
	float r = 0.0275;

	//Applying M_inv matrix.
	output[wheels_RF] = (1/s*v[body_x] + 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_RB] = (1/s*v[body_x] - 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_LB] = (-1/s*v[body_x] - 1/c*v[body_y] - 1/R*v[body_w])*r/4;
	output[wheels_LF] = (-1/s*v[body_x] + 1/c*v[body_y] - 1/R*v[body_w])*r/4;
}

//multiplies a 3*4 matrix by a vector of 4 elements.
void wheels2Body(float w[4], float output[3]){

	float s = 0.5;
	float c = 0.866;
	float R = 0.0775;
	float r = 0.0275;

	//Applying transpose(M_inv) matrix
	output[body_x] = (1/s*w[wheels_RF] + 1/s*w[wheels_RB] - 1/s*w[wheels_LB] - 1/s*w[wheels_LF])*r/4;
	output[body_y] = (1/c*w[wheels_RF] - 1/c*w[wheels_RB] - 1/c*w[wheels_LB] + 1/c*w[wheels_LF])*r/4;
	output[body_w] = (-1/R*w[wheels_RF] - 1/R*w[wheels_RB] - 1/R*w[wheels_LB] - 1/R*w[wheels_LF])*r/4;
}

void rotate(float yaw, float input[3], float output[3]){

	output[body_x] = cos(yaw)*input[body_x] - sin(yaw)*input[body_x];
	output[body_y] = sin(yaw)*input[body_y] + cos(yaw)*input[body_y];
	output[body_w] = input[body_w];

}

void pController(float input[3], float kp[3], float output[3]){

	output[body_x] = kp[body_x]*input[body_x];
	output[body_y] = kp[body_y]*input[body_y];
	output[body_w] = kp[body_w]*input[body_w];

}

void limiter(float input[3], float maxEl, float output[3]){

	float scale;

	if (fabs(maxEl) > 95){
		scale = 95/fabs(maxEl);
	}
	else{
		scale = 0;
	}

	output[body_x] = scale*input[body_x];
	output[body_y] = scale*input[body_y];
	output[body_w] = input[body_w];
}

//observer output should just be pre-declared as 0 before any looping starts
void controller(float velocityRef[3], float w_wheels[4], float xsensData[3], float ptrdo[3], float output[4]){

	float localReference[3];
	rotate(xsensData[body_w], velocityRef, localReference);

	float localVel[3];
	wheels2Body(w_wheels, localVel);

	float error[3];
	error[body_x] = localReference[body_x] - localVel[body_x];
	error[body_y] = localReference[body_y] - localVel[body_y];
	error[body_w] = localReference[body_w] - localVel[body_w];

	float controllerGain[3];
	controllerGain[body_x] = 2000;
	controllerGain[body_y] = 2000;
	controllerGain[body_w] = 400;
	float pOut[3];
	pController(error, controllerGain, pOut);

	float postObserverSignal[3];
	postObserverSignal[body_x] = pOut[body_x] - ptrdo[body_x];
	postObserverSignal[body_y] = pOut[body_y] - ptrdo[body_y];
	postObserverSignal[body_w] = pOut[body_w] - ptrdo[body_w];

	// Limiting the output to prevent saturation of the PWM signals for any of the wheels
	float intermediaryOutput[4];
	body2Wheels(postObserverSignal, intermediaryOutput);
	float maxEl = fmax(fmax(intermediaryOutput[wheels_RF],intermediaryOutput[wheels_RB]),fmax(intermediaryOutput[wheels_LB],intermediaryOutput[wheels_LF]));
	float scaledInput[3];
	limiter(postObserverSignal, maxEl, scaledInput);

	// output the wheel PWMs
	body2Wheels(scaledInput, output);

	// compute disturbance observer output for next iteration
	disturbanceObserver(xsensData, scaledInput, ptrdo);
}


DO_States DO_Control(float velocityRef[3], float xsensData[3]){

	float w_wheels[4];
	w_wheels[wheels_RF] = wheels_GetSpeed(wheels_RF);
	w_wheels[wheels_RB] = wheels_GetSpeed(wheels_RB);
	w_wheels[wheels_LB] = wheels_GetSpeed(wheels_LB);
	w_wheels[wheels_LF] = wheels_GetSpeed(wheels_LF);

	float wheelsPWM[4];
	controller(velocityRef, w_wheels, xsensData, observerOutput, wheelsPWM);

	wheels_SetOutput(wheelsPWM);

	return DO_error;
}
