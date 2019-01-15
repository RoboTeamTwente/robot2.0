/*
 * yawCalibration.h
 *
 *  Created on: Jan 8, 2019
 *      Author: simen
 */

#ifndef YAWCALIBRATION_H_
#define YAWCALIBRATION_H_

#include "stdbool.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS
float calibrateYaw(float xsensYaw, float visionYaw, bool visionAvailable);

#endif /* YAWCALIBRATION_H_ */
