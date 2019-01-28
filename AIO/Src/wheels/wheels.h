/*
 * wheels.h
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

/*Description:
Instructions:
1)
Extra functions:
Notes:
*/


#ifndef WHEELS_H_
#define WHEELS_H_
#define TIME_DIFF 0.01F // time difference due to 100Hz
#include <stdbool.h>
#include "../DO/control_util.h"

///////////////////////////////////////////////////// TYPES/STRUCTS

typedef enum {
	wheels_uninitialized,
	wheels_ready
}wheels_states;// keeps track of the state of this system

typedef enum {
	no_brake,
	first_brake_period,
	second_brake_period,
	third_brake_period
}brake_states;// keeps track of the brake state of this system

//TODO: add control values based on tests
static PIDvariables RFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables RBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables LBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
static PIDvariables LFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

void wheelsInit();

void wheelsDeInit();

void setWheelSpeed(float wheelref[4]);

float getWheelSpeed(wheel_names wheel);

int getPWM(wheel_names wheel);

void wheelsCallback();

#endif /* WHEELS_H_ */
