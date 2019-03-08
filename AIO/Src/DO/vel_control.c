/*
 * vel_control.c
 *
 *  Created on: Jan 23, 2019
 *      Author: simen
 */

#include "vel_control.h"

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]){

	/*---------------------------
	 * 			TESTING
	  ---------------------------

	float minP = 10.0;
	float maxP = 20.0, maxI = 2.0;
	float stepP = 1.0, stepI = 0.2;

	static float prevAngle = 0;
	if (prevAngle - vel_ref[2] > 1) {
		if (angleK.kP < maxP) {
			angleK.kP += stepP;
		} else if(angleK.kI < maxI) {
			angleK.kP = minP;
			angleK.kI += stepI;
		}
	}

	if (angleK.kP == maxP && angleK.kI == maxI) {
		vel_ref[2] = 0;
	}

	prevAngle = vel_ref[2];

 	 -----------------------------*/


	// Translational control
//	float velxErr = vel_ref[body_x] - State[body_x];
//	float velyErr = vel_ref[body_y] - State[body_y];
//
//	float velGlobalRef[3];
//	velGlobalRef[body_x] = vel_ref[body_x];
//	velGlobalRef[body_y] = vel_ref[body_y];

//	velGlobalRef[body_x] = vel_ref[body_x] + PID(velxErr, &velxK);
//	velGlobalRef[body_y] = vel_ref[body_y] + PID(velyErr, &velyK);

	float velLocalRef[3] = {0};
	//global2Local(velGlobalRef, velLocalRef, State[body_w]); //transfer global to local
	global2Local(vel_ref, velLocalRef, 0); //transfer global to local

	float velxErr = velLocalRef[body_x] - State[body_x];
	float velyErr = velLocalRef[body_y] - State[body_y];
//	velLocalRef[body_x] += PID(velxErr, &velxK);
//	velLocalRef[body_y] += PID(velyErr, &velyK);

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

	// Angular control
	float angleErr = constrainAngle(State[body_w] - vel_ref[body_w]);//constrain it to one circle turn
	if (fabs(angleErr) < M_PI/180) { // allow 1 degree deviation
		angleErr = 0;
		angleK.I = 0;
	}
	float angleComp = PID(angleErr, &angleK);// PID control from control_util.h
	for (int i = 0; i < 4; ++i){
		wheel_ref[i] += angleComp; //add necessary rotation value
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	//TODO check minuses
	float velx2wheel = (velocity[body_x]/(sin60*rad_wheel));
	float vely2wheel = (velocity[body_y]/(cos60*rad_wheel));
	//float rot2wheel =  (rad_robot*velocity[body_w]/rad_wheel);
	wheelspeed[wheels_RF] = -(velx2wheel + vely2wheel);
	wheelspeed[wheels_RB] = -(velx2wheel - vely2wheel);
	wheelspeed[wheels_LB] = -(-velx2wheel - vely2wheel);
	wheelspeed[wheels_LF] = -(-velx2wheel + vely2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cosf(yaw)*global[body_x]-sinf(yaw)*global[body_y];
	local[body_y] = sinf(yaw)*global[body_x]+cosf(yaw)*global[body_y];
}
