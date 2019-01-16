/*
 * control.c
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */

#include <stdio.h>
#include <math.h>
#include "DO.h"
#include "tim.h"
#include "../wheels/wheels.h"
#include "../MTi/MTiControl.h"

///////////////////////////////////////////////////// DEFINITIONS
// Basically set constants
#define CALIBRATE_AFTER 6500 // time after which yaw calibration will start (ms)
#define rad_robot 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.0275F 	// wheel radius (m)
#define cos60 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define sin60 0.866F	// sine of 60 degrees

uint start_time;

PIDvariables angleK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velxK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velyK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};


///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

//scales and limit the signal
static void scaleAndLimit(float wheel_ref[4]);

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x);

static void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]);

static bool checkYawcalibration(bool calibration_needed, bool vision_available, float vision_yaw, float bodyw, int last_calibration_time);

static bool yawcalibration(bool calibration_needed, bool vision_available, float xsens_yaw, float vision_yaw, float yaw_offset[1], uint last_calibration_time[1]);

static void wheelFilter(float w_wheels[4]);

static void getXsensData(float xsensData[3], float yaw_offset[1], float xsens_yaw);

void rotate(float yaw, float input[3], float output[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return 0;
}

bool DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float output[4]){
	static bool calibration_needed = true; //Has to be static, if reset every time this function is run the wheels hamper

	// get xsens yaw
	float xsens_yaw = MT_GetAngles()[2]/180*M_PI;

	// get and filter wheel speeds
	float w_wheels[4] = {0,0,0,0};
	//wheelFilter(w_wheels);//edits the above array

	// calibration of yaw offset
	// we use the xsens data for the control, but that has drift
	//so we compare it to the vision data, and based on that we decide if we need to calibrate.

	static float yaw_offset[1] = {0};
	static uint last_calibration_time[1] = {0};

	calibration_needed = yawcalibration(calibration_needed, vision_available, xsens_yaw, vision_yaw, yaw_offset, last_calibration_time);

	// get and offset xsens data
	float xsensData[3];
	getXsensData(xsensData, yaw_offset, xsens_yaw);


	// check whether recalibration of yaw is highly necessary. If so, calibration needed is set to true, which leads to halting the robot until calibrated.
	if (vision_available && !calibration_needed) {
		calibration_needed = checkYawcalibration(calibration_needed, vision_available, vision_yaw, xsensData[body_w], last_calibration_time[0]);
	}

	// control part

	if (!calibration_needed){
		vel_control_Callback(output, xsensData, velocityRef);
	}

	return calibration_needed;
}


///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void rotate(float yaw, float input[3], float output[3]){

	float outX = cosf(yaw)*input[body_x] + sinf(yaw)*input[body_y];
	float outY = -sinf(yaw)*input[body_x] + cosf(yaw)*input[body_y];

	output[body_x] = outX;
	output[body_y] = outY;
	output[body_w] = input[body_w];

}

void getXsensData(float xsensData[3], float yaw_offset[1], float xsens_yaw){

	float accptr[2] = {MT_GetAcceleration()[0],MT_GetAcceleration()[1]};
	xsensData[body_x] = -accptr[0];
	xsensData[body_y] = -accptr[1];
	xsensData[body_w] = constrainAngle(xsens_yaw + yaw_offset[0]);
	rotate(-yaw_offset[0], xsensData, xsensData); // the free accelerations should also be calibrated to the offset yaw

}

void wheelFilter(float w_wheels[4]){
	// get and filter wheel speeds
	static float w_prev[4] = {0,0,0,0};
		 // filtering wheel speeds
		for(wheels_handles i = wheels_RF; i <= wheels_LF; i++){
			if (!isnan(w_prev[i])){
				//TODO Test ratio
				w_wheels[i] = 0.4*(-wheels_GetSpeed(i)) + 0.6*w_prev[i];
			} else {
				w_wheels[i] = -wheels_GetSpeed(i);
			}
			w_prev[i] = w_wheels[i];
		}
}

bool yawcalibration(bool calibration_needed, bool vision_available, float xsens_yaw, float vision_yaw, float yaw_offset[1], uint last_calibration_time[1]){

	static float avg_xsens_vec[2] = {0,0}; 	// vector describing the average yaw measured by xsens over a number of time steps
	static float avg_vision_vec[2] = {0,0}; 	// vector describing the average yaw measured by vision over a number of time steps
	static int no_rot_counter = 0;	// keeps track of the amount of time steps without rotation
	int no_rot_duration = 20;		// time steps of no rotation required before calibrating
	int avg_size = 10; 				// amount of samples to take the average of, for the yaws

	// if calibration is necessary
	if ((calibration_needed || vision_available) && HAL_GetTick() - start_time > CALIBRATE_AFTER) {
		static float yaw0 = 0; 	// starting yaw, which is used to detect a change in yaw within the duration
		// check for calibration possibility (which is when the yaw has not changed much for sufficiently long)
		if (fabs(yaw0 - xsens_yaw) < 0.01) {
			if (no_rot_counter > no_rot_duration) {
				// calculate offset (calibrate)
				float avg_xsens_yaw = atan2f(avg_xsens_vec[1], avg_xsens_vec[0]);
				if (vision_available) { // if vision would not be availabe, the yaw would simply be reset to 0;
					float avg_vision_yaw = atan2f(avg_vision_vec[1], avg_vision_vec[0]);
					yaw_offset[0] = constrainAngle(avg_vision_yaw - avg_xsens_yaw);
				} else {
					yaw_offset[0] = -avg_xsens_yaw;
				}
				calibration_needed = false; // done with calibrating
				last_calibration_time[0] = HAL_GetTick();
				no_rot_counter = 0;			// reset timer
			} else if (no_rot_counter > no_rot_duration - avg_size) {
				// averaging angles requires summing their unit vectors
				avg_xsens_vec[0] += cosf(xsens_yaw);
				avg_xsens_vec[1] += sinf(xsens_yaw);
				avg_vision_vec[0] += cosf(vision_yaw);
				avg_vision_vec[1] += sinf(vision_yaw);
			}
		} else { // so fabs(yaw0 - xsens_yaw) > 0.01
			// reset comparation yaw to current yaw
			yaw0 = xsens_yaw;
			// reset timer and averages
			no_rot_counter = 0;
			avg_xsens_vec[0] = 0; avg_xsens_vec[1] = 0;
			avg_vision_vec[0] = 0; avg_vision_vec[1] = 0;
		}
		no_rot_counter++;
	} else {
		// reset timer and averages
		no_rot_counter = 0;
		avg_xsens_vec[0] = 0; avg_xsens_vec[1] = 0;
		avg_vision_vec[0] = 0; avg_vision_vec[1] = 0;
	}

	return calibration_needed;
}

bool checkYawcalibration(bool calibration_needed, bool vision_available, float vision_yaw, float bodyw, int last_calibration_time){

	// assuming 7 steps of delay between vision and xsens (so 70 ms) (might be off)
	static float xsens_yaw_buffer[7] = {0,0,0,0,0,0,0};
	for (int i = 0; i < 6; i++) {
		xsens_yaw_buffer[i] = xsens_yaw_buffer[i+1];
	}
	xsens_yaw_buffer[6] = bodyw;

	// if vision yaw and xsens yaw deviate too much for several time steps, set calibration needed to true
	static int check_counter = 0;
	if (HAL_GetTick() - last_calibration_time > 100 && fabs(constrainAngle(vision_yaw - xsens_yaw_buffer[0])) > 0.2) {
		check_counter++;
	} else {
		check_counter = 0;
	}
	if (check_counter > 10) {
		calibration_needed = true;
		check_counter = 0;
	}

	return calibration_needed;
}


void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]){

	/*For dry testing
	State[body_w] = MT_GetAngles()[2]/180*M_PI;
	vel_ref[body_x] = 0;
	vel_ref[body_y] = 0;
	vel_ref[body_w] = 0*M_PI;
	State[body_x] = 0;
	State[body_y] = 0;
	State[body_w] = 0*M_PI;
	*/

	float angleErr = constrainAngle((vel_ref[body_w] - State[body_w]));//constrain it to one circle turn
	float angleComp = PID(angleErr, &angleK);// PID control from control_util.h
	float velLocalRef[3] = {0,0,0};
	global2Local(vel_ref, velLocalRef, State[body_w]); //transfer global to local

	// PID control from control_util.h
	//velLocalRef[body_x] += PID((velLocalRef[body_x]-State[body_x]), &velxK); //error compensation plus requested velocity
	//velLocalRef[body_y] += PID((velLocalRef[body_y]-State[body_y]), &velyK);

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

	float flo = 2;
	for (int i = 0; i < 4; ++i){
		wheel_ref[i] = flo; //add necessary rotation value
	}

	return;
}


static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	//TODO check minuses
	float velx2wheel = 1/(sin60*velocity[body_x]*rad_wheel);
	float vely2wheel = 1/(cos60*velocity[body_y]*rad_wheel);
	float rot2wheel = 1/(rad_robot*velocity[body_w]*rad_wheel);
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

static void scaleAndLimit(float wheel_ref[4]){
	//TODO: add some limitation stuff here
}

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
