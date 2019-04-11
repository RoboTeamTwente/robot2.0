/*
 * vel_control.c
 *
 *  Created on: Jan 23, 2019
 *      Author: simen
 */

#include "vel_control.h"
#include "stdbool.h"

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

static void calcScaleFactors(float translationalRef[4], float angularRef, float scaleFactors[2]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3], bool use_global_ref){

	/*----------------------
	 * Translational control
	 ----------------------*/
	float velLocalRef[3] = {0, 0, 0};
	use_global_ref = true;
	global2Local(vel_ref, velLocalRef, use_global_ref ? State[body_w] : 0); //transfer global to local

	// Manually adjusting velocity command
	//     Explanation: see Velocity Difference file on drive (https://docs.google.com/document/d/1pGKysiwpu19DKLpAZ4GpluMV7UBhBQZ65YMTtI7bd_8/)
	velLocalRef[body_x] = 1.063 * velLocalRef[body_x];
	velLocalRef[body_y] = 1.308 * velLocalRef[body_y];

	// Local control
	float velxErr = (velLocalRef[body_x] - State[body_x]);
	float velyErr = (velLocalRef[body_y] - State[body_y]);
	velLocalRef[body_x] += PID(velxErr, &velxK);
	velLocalRef[body_y] += PID(velyErr, &velyK);

	float translationalRef[4] = {0, 0, 0, 0};
	body2Wheels(translationalRef, velLocalRef); //translate velocity to wheel speed


	/*----------------
	 * Angular control
	 ----------------*/
	float angleErr = constrainAngle(vel_ref[body_w] - State[body_w]);//constrain it to one circle turn
	if (fabs(angleErr) < YAW_MARGIN) { // reset the I to zero everytime the target has been reached
		angleErr = 0;
		angleK.I = 0;
	}
	float angularRef = PID(angleErr, &angleK);// PID control from control_util.h


	/*---------------------------
	 * Scale and mix both parts
	 ---------------------------*/
	float scaleFactors[2] = {1, 1};
	calcScaleFactors(translationalRef, angularRef, scaleFactors);
	for (int i=0; i<4; ++i) {
		wheel_ref[i] = scaleFactors[0] * translationalRef[i] + scaleFactors[1] * angularRef;
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	float velx2wheel = (velocity[body_x]*sin60/rad_wheel);
	float vely2wheel = (velocity[body_y]*cos60/rad_wheel);
	//float rot2wheel =  (rad_robot*velocity[body_w]/rad_wheel);
	wheelspeed[wheels_RF] = (velx2wheel + vely2wheel);
	wheelspeed[wheels_RB] = (velx2wheel - vely2wheel);
	wheelspeed[wheels_LB] = (-velx2wheel - vely2wheel);
	wheelspeed[wheels_LF] = (-velx2wheel + vely2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cosf(yaw)*global[body_x]+sinf(yaw)*global[body_y];
	local[body_y] = -sinf(yaw)*global[body_x]+cosf(yaw)*global[body_y];
	local[body_w] = global[body_w];
}

static void calcScaleFactors(float translationalRef[4], float angularRef, float scaleFactors[2]) {
	float f = 1; // importance factor: 0 -> all rotational, 1 -> all translational

	// Find highest difference between necessary wheel speed and limit
	float highestOvershoot = 0, overshoot = 0;
	int index = 0;
	for (int i=0; i<4; ++i) {
		overshoot = fabs(translationalRef[i] + angularRef) - WHEEL_REF_LIMIT;
		if (overshoot > highestOvershoot) {
			highestOvershoot = overshoot;
			index = i;
		}
	}
	if (highestOvershoot == 0) {
		// Necessary wheel speed does not exceed the limit
		return;
	}

	scaleFactors[0] = 1 - (1-f)*highestOvershoot/fabs(translationalRef[index]); // Translational factor
	scaleFactors[1] = 1 - f*highestOvershoot/fabs(angularRef); // Rotational factor
	scaleFactors[0] = scaleFactors[0] < 0 ? 0 : scaleFactors[0];
	scaleFactors[1] = scaleFactors[1] < 0 ? 0 : scaleFactors[1];
	return;
}

