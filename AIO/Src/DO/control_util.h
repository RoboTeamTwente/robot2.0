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

// System
#define TIME_DIFF 0.01F // time difference due to 100Hz frequency

// Robot
#define rad_robot 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.0275F 	// wheel radius (m)
#define cos60 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define sin60 0.866F	// sine of 60 degrees

// Wheels
#define PWM_CUTOFF 240.0F			// arbitrary treshold below PWM_ROUNDUP
#define PWM_ROUNDUP 250.0F 		// below this value the motor driver is unreliable
#define GEAR_RATIO 2.5F // gear ratio between motor and wheel
#define MAX_PWM 2400
#define PWM_LIMIT 1000 // should be equal to MAX_PWM by default
#define MAX_VOLTAGE 12//see datasheet
#define SPEED_CONSTANT (374.0/60.0) //[RPS/V] see datasheet
#define PULSES_PER_ROTATION (float)4*1024 // number of pulses of the encoder per rotation of the motor (see datasheet)

#define RPStoPWM (1.0/SPEED_CONSTANT)*(MAX_PWM/MAX_VOLTAGE)*GEAR_RATIO // conversion factor from rotations per second of the wheel to required PWM on the motor
#define ENCODERtoRPS (float)1/(TIME_DIFF*GEAR_RATIO*PULSES_PER_ROTATION) // conversion factor from number of encoder pulses to RPS of the wheel
#define RPS_LIMIT (PWM_LIMIT/RPStoPWM) // Highest wheel speed that is allowed
#define RPS_CUTOFF (PWM_CUTOFF/RPStoPWM) // Lowest wheel speed that is allowed

///////////////////////////////////////////////////// STRUCTS AND VARIABLES

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF,
}wheel_names;

typedef struct {
	float kP;
	float kI;
	float kD;
	float I;
	float prev_e;
	float timeDiff;
}PIDvariables;

static PIDvariables angleK = {
		.kP = 7,//kp
		.kI = 0,//ki
		.kD = 0.0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables velxK = {
		.kP = 1,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables velyK = {
		.kP = 1,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};

///////////////////////////////////////////////////// FUNCTIONS

//PID control, static to not have multiple implementation error
static float PID(float err, PIDvariables* K){
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
