/*
 * stateEstimation.h
 *
 *  Created on: Jan 25, 2019
 *      Author: simen
 */

#ifndef DO_STATEESTIMATION_H_
#define DO_STATEESTIMATION_H_

#include "control_util.h"
#include "../wheels/wheels.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

float velocities[3];
void estimateState(float State[3], float xsensData[3]);
void getvel(float Vel[2]);

#endif /* DO_STATEESTIMATION_H_ */
