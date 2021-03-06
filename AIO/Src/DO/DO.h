/*
 * DO.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef DO_DO_H_
#define DO_DO_H_

#include <stdbool.h>

typedef enum {
	DO_succes,
	DO_error
}DO_States;

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

DO_States DO_Init();

bool DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float output[4]);

float constrainAngle(float x);
void body2Wheels(float F[3], float output[4]);
void wheels2Body(float w[4], float output[3]);
void rotate(float yaw, float input[3], float output[3]);
float compute_limit_scale(float input[3], float limit);
void disturbanceObserver(float yaw, float localInput[3], float globalAcc[2], float output[3]);
void pController(float input[3], float kp[3], float output[3]);
void controller(float velocityRef[3], float w_wheels[4], float xsensData[3], float output[4]);
float angleController(float angleRef, float yaw);

#endif /* DO_DO_H_ */
