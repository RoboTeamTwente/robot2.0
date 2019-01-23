/*
 * control_util.h
 *
 *  Created on: Nov 14, 2018
 *      Author: kjhertenberg
 */

#ifndef UTILS_CONTROL_UTIL_H_
#define UTILS_CONTROL_UTIL_H_

#include <math.h>

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
