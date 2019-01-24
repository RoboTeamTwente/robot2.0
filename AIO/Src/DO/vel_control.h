/*
 * vel_control.h
 *
 *  Created on: Jan 23, 2019
 *      Author: simen
 */

#ifndef DO_VEL_CONTROL_H_
#define DO_VEL_CONTROL_H_

#include "control_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3]);

#endif /* DO_VEL_CONTROL_H_ */
