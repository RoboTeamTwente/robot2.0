/*
 * control.h
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */
/*
Description: Determines the wanted wheel speed, based on received data

Instructions:
1) Initialize
2) Receives data of it's state
3) Combine data, not in here yet
4) Transform data to right frame and units
5) Apply control functions
6) Scale and limit the outgoing signal

Extra functions:

GPIO Pins: None

Notes:
Still need to add the right specs
*/

#ifndef DO_DO_H_
#define DO_DO_H_
#define TIME_DIFF 0.01F // time difference due to 100Hz
#include <stdbool.h>
#include "control_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int vel_control_Init();
void getvel(float Vel[2]);
void DO_Control(float velocityRef[3], float vision_yaw, bool vision_available, float wheel_ref[4]);

float getYaw(); // For debugging

#endif /* DO_DO_H_ */

