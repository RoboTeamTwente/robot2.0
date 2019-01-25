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
	float angleErr = constrainAngle(State[body_w] - vel_ref[body_w]);//constrain it to one circle turn
	float angleComp = PID(angleErr, &angleK);// PID control from control_util.h
	static float velLocalRef[3] = {0};
	global2Local(vel_ref, velLocalRef, State[body_w]); //transfer global to local

	// PID control from control_util.h
	velLocalRef[body_x] += PID((velLocalRef[body_x]-State[body_x]), &velxK); //error compensation plus requested velocity
	velLocalRef[body_y] += PID((velLocalRef[body_y]-State[body_y]), &velyK);

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

	for (int i = 0; i < 4; ++i){
		//wheel_ref[i] += angleComp; //add necessary rotation value
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	//TODO check minuses
	float velx2wheel = (sin60*velocity[body_x]/rad_wheel);
	float vely2wheel = (cos60*velocity[body_y]/rad_wheel);
	float rot2wheel =  (rad_robot*velocity[body_w]/rad_wheel);
	wheelspeed[wheels_RF] = -(velx2wheel + vely2wheel);
	wheelspeed[wheels_RB] = -(velx2wheel - vely2wheel);
	wheelspeed[wheels_LB] = -(-velx2wheel - vely2wheel);
	wheelspeed[wheels_LF] = -(-velx2wheel + vely2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cos(yaw)*global[body_x]-sin(yaw)*global[body_y];
	local[body_y] = sin(yaw)*global[body_x]+cos(yaw)*global[body_y];
}
