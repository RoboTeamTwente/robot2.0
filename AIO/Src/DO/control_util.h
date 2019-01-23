/*
 * control_util.h
 *
 *  Created on: Nov 14, 2018
 *      Author: kjhertenberg
 */

#ifndef UTILS_CONTROL_UTIL_H_
#define UTILS_CONTROL_UTIL_H_

#include <math.h>

///////////////////////////////////////////////////// DEFINITIONS
// Basically set constants
#define rad_robot 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.0275F 	// wheel radius (m)
#define cos60 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define sin60 0.866F	// sine of 60 degrees
#define TIME_DIFF 0.01F // time difference due to 100Hz frequency

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

typedef struct {
	float kP;
	float kI;
	float kD;
	float I;
	float prev_e;
	float timeDiff;
}PIDvariables;

static PIDvariables angleK = {
		.kP = 2,//kp
		.kI = 0,//ki
		.kD = 0.5,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables velxK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables velyK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};

//PID control, static to not have multiple implementation error
inline float PID(float err, PIDvariables* K){
	float P = K->kP*err;
	K->I += err*K->timeDiff;
	float I = K->kI*K->I;
	float D = K->kD*((err-K->prev_e)/K->timeDiff);
	K->prev_e = err;
	float PIDvalue = P + I + D;
	return PIDvalue;
}

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

#endif /* UTILS_CONTROL_UTIL_H_ */
