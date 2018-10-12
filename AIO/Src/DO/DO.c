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
#include "../PuttyInterface/PuttyInterface.h"
#include "../MTi/MTiControl.h"

#define CALIBRATE_AFTER 6500 // time after which yaw calibration will start (ms)
#define TIME_DIFF 0.01F // time difference due to 100Hz
#define R 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define r 0.0275F 	// wheel radius (m)
#define c 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define s 0.866F	// sine of 60 degrees
#define PWM_CUTOFF 3.0F // below this value the motor PWM is set to 0 (see wheels.c)

uint start_time;

// controller settings
//TODO: some of these should be adaptable through wireless communication
bool DO_enabled = false;
bool use_yaw_control = true;
bool ref_is_angle = true;
bool use_global_ref = true;
bool no_vel_control = true;

DO_States DO_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return DO_succes;
}

//Scales the angle to the range Pi to -Pi in radians
float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

//multiplies a 4*3 matrix (M_inv) by a vector of 3 elements, to go from body force to wheel torque. (minus sign in front is due to wheels spinning in opposite direction to motors)
void body2Wheels(float F[3], float output[4]){
	float T_cutoff = (PWM_CUTOFF + 0.1F)*4*R/r;	// if the rotational force gets below this value, the motors wont deliver any torque
	if (fabs(F[body_w]) < T_cutoff && fabs(F[body_w]) > T_cutoff/2 - 0.1F) {
		// SPECIAL CASE: when 4 wheels deliver too much torque, we will use only 2 wheels for rotation
		output[wheels_RF] = -(1/s*F[body_x] + 1/c*F[body_y] + 1/R*F[body_w]*2)*r/4;
		output[wheels_RB] = -(1/s*F[body_x] - 1/c*F[body_y])*r/4;
		output[wheels_LB] = -(-1/s*F[body_x] - 1/c*F[body_y] + 1/R*F[body_w]*2)*r/4;
		output[wheels_LF] = -(-1/s*F[body_x] + 1/c*F[body_y])*r/4;
	} else { // REGULAR CASE
		output[wheels_RF] = -(1/s*F[body_x] + 1/c*F[body_y] + 1/R*F[body_w])*r/4;
		output[wheels_RB] = -(1/s*F[body_x] - 1/c*F[body_y] + 1/R*F[body_w])*r/4;
		output[wheels_LB] = -(-1/s*F[body_x] - 1/c*F[body_y] + 1/R*F[body_w])*r/4;
		output[wheels_LF] = -(-1/s*F[body_x] + 1/c*F[body_y] + 1/R*F[body_w])*r/4;
	}

}

//multiplies a 3*4 matrix by a vector of 4 elements, to go from wheel torque to body force
void wheels2Body(float w[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = (1/s*w[wheels_RF] + 1/s*w[wheels_RB] - 1/s*w[wheels_LB] - 1/s*w[wheels_LF])*r/4;
	output[body_y] = (1/c*w[wheels_RF] - 1/c*w[wheels_RB] - 1/c*w[wheels_LB] + 1/c*w[wheels_LF])*r/4;
	output[body_w] = -(-1/R*w[wheels_RF] - 1/R*w[wheels_RB] - 1/R*w[wheels_LB] - 1/R*w[wheels_LF])*r/4;
}

// rotates coordinate system by yaw
void rotate(float yaw, float input[3], float output[3]){

	float outX = cosf(yaw)*input[body_x] + sinf(yaw)*input[body_y];
	float outY = -sinf(yaw)*input[body_x] + cosf(yaw)*input[body_y];

	output[body_x] = outX;
	output[body_y] = outY;
	output[body_w] = input[body_w];

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

void getXsensData(float xsensData[3], float yaw_offset[1], float xsens_yaw){

	float accptr[2] = {MT_GetAcceleration()[0],MT_GetAcceleration()[1]};
	xsensData[body_x] = -accptr[0];
	xsensData[body_y] = -accptr[1];
	xsensData[body_w] = constrainAngle(xsens_yaw + yaw_offset[0]);
	rotate(-yaw_offset[0], xsensData, xsensData); // the free accelerations should also be calibrated to the offset yaw

}

float compute_limit_scale(float input[3], float limit){

	float scale;
	float intermediaryOutput[4];
	body2Wheels(input, intermediaryOutput);
	float maxEl = fmax(fmax(fabs(intermediaryOutput[wheels_RF]),fabs(intermediaryOutput[wheels_RB])),fmax(fabs(intermediaryOutput[wheels_LB]),fabs(intermediaryOutput[wheels_LF])));

	if ((maxEl) > limit){
		scale = limit/(maxEl);
	}
	else {
		scale = 1;
	}

	return scale;
}

float angleController(float angleRef, float yaw){
	// TODO: FEW TWEAKS MADE THAT NEED TESTING
	// PD Control of the yaw
//		uprintf("[%f, %f, %f]\n\r", angleRef, yaw, yaw/M_PI*180);
	float angleError = constrainAngle(angleRef - yaw);
	static float prevError = 0;
	float dError = constrainAngle(angleError-prevError)/TIME_DIFF;
	prevError = angleError;

	float output;
	static int threshold_switch; // prevents rapid switching of motor due to yaw sensor noise
	threshold_switch = -1;

	if (no_vel_control) {
		// no velocity control will be applied on the output calculated here. It should therefore be interpreted as a torque on the robot
		output = angleError*300.0F + dError*13.0F;
		float mag = fabs(output);
		float upper_lim = 300.0F;
		float deadzone = 0.03;	// (rad) if the robot gets this close, the motors should stop delivering torque
		float T_cutoff = (PWM_CUTOFF + 0.1F)*4*R/r;	// if the output gets below this value, the motors wont deliver any torque
		if (mag > upper_lim) {
			output = output / mag * upper_lim;
		} else if (fabs(angleError) < deadzone + threshold_switch*0.003F) {
			output = 0;
			threshold_switch = 1;
		} else if (mag < T_cutoff/2 && mag > 0.001F) {
			output = output / mag * T_cutoff/2;
		}
//		uprintf("[%f, %f, %f]\n\r", T_cutoff, output, angleError);
	} else {
		// the output should be interpreted as an angular velocity reference
		output = angleError*15.0F + dError*0.2F;
		float mag = fabs(output);
		float upper_lim = 20.0;
		float lower_lim = 1.0;
		float lower_roundup = 10.0;
		if (mag > upper_lim) {
			output = output / mag * upper_lim;
		} else if (mag < lower_lim + threshold_switch*0.1F) {
			output = 0;
			threshold_switch = 1;
		} else if (mag < lower_roundup) {
			output = output / mag * lower_roundup;
		}
	}

	return output;
}

void rotVelCompensation(float yaw, float localVelocityRef[3]){

	// Compensation for moving direction when rotating
	float assumed_delay = 0.06; //s
	static float prevYaw = 0;
	float yawVel = constrainAngle(yaw - prevYaw)/TIME_DIFF;
	prevYaw = yaw;
	float compensation_dir = yawVel*assumed_delay;
	rotate(compensation_dir, localVelocityRef, localVelocityRef);
}

void notControl(float velocityRef[3], float xsensData[3], float w_wheels[4], float output[4]){
	// determine local velocity reference
		float localVelocityRef[3] = {velocityRef[body_x],velocityRef[body_y],velocityRef[body_w]};
		if (use_global_ref) {
			rotate(xsensData[body_w], velocityRef, localVelocityRef); // apply coordinate transform from global to local for the velocity reference
		}

		rotVelCompensation(xsensData[body_w], localVelocityRef);

		// yaw  and velocity controllers
		float angleRef;
		angleRef = localVelocityRef[body_w] / 16;
		float forceRef[3] = {localVelocityRef[body_x]*1500, localVelocityRef[body_y]*2000, 0};
		forceRef[body_w] = angleController(angleRef, xsensData[body_w])*2.0F;
		float scale = compute_limit_scale(forceRef, 100);
		forceRef[body_x] = scale*forceRef[body_x];
		forceRef[body_y] = scale*forceRef[body_y];
		forceRef[body_w] = forceRef[body_w]/2.0F;
		body2Wheels(forceRef, output);

	return;
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

bool DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float output[4]){
	static bool calibration_needed = true; //Has to be static, if reset every time this function is run the wheels hamper

	// get xsens yaw
	float xsens_yaw = MT_GetAngles()[2]/180*M_PI;

	// get and filter wheel speeds
	float w_wheels[4] = {0,0,0,0};
	wheelFilter(w_wheels);//edits the above array

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
	notControl(velocityRef, xsensData, w_wheels, output);
	}

	return calibration_needed;
}
