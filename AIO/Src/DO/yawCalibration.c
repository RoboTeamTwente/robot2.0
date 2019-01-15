/*
 * yawCalibration.c
 *
 *  Created on: Jan 8, 2019
 *      Author: simen
 */

#include "yawCalibration.h"
#include "stdbool.h"
#include <math.h>

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
bool isCalibrationNeeded(float visionYaw, float xsensYaw);
bool isRotatingSlow(float xsensYaw);
float constrainAngle(float x);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS
float calibrateYaw(float xsensYaw, float visionYaw, bool visionAvailable) {
	static float yawOffset = 0;
	static int restCounter = 0;
	static float avgXsensVec[2] = {0}, avgVisionVec[2] = {0};
	int restDuration = 20; // number of time steps to do for averaging TODO: test this

	if (isCalibrationNeeded(visionYaw, xsensYaw) && isRotatingSlow(xsensYaw) && visionAvailable) {
		if (restCounter > restDuration) {
			// calculate offset
			float avgVisionYaw = atan2f(avgVisionVec[1], avgVisionVec[0]);
			float avgXsensYaw = atan2f(avgXsensVec[1], avgXsensVec[0]);
			yawOffset = constrainAngle(avgVisionYaw - avgXsensYaw);
			restCounter = 0;
		} else {
			// Taking the average of the angles makes no sense, since Pi and -Pi would not be the same angle.
			// Instead, sum the unit vectors with these angles and then take the angle of the resulting vector.
			avgXsensVec[0] += cosf(xsensYaw);
			avgXsensVec[1] += sinf(xsensYaw);
			avgVisionVec[0] += cosf(visionYaw);
			avgVisionVec[1] += sinf(visionYaw);
			restCounter++;
		}
	} else {
		restCounter = 0;
		avgXsensVec[0] = 0; avgXsensVec[1] = 0;
		avgVisionVec[0] = 0; avgVisionVec[1] = 0;
	}
	return constrainAngle(xsensYaw + yawOffset);
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
bool isCalibrationNeeded(float visionYaw, float xsensYaw) {
	// if vision yaw and xsens yaw deviate too much for several time steps, set calibration needed to true
	static int checkCounter = 0;
	if (fabs(constrainAngle(visionYaw - xsensYaw)) > 0.2) {
		checkCounter++;
	} else {
		checkCounter = 0;
	}
	if (checkCounter > 10) {
		checkCounter = 0;
		return true;
	}
	return false;
}

bool isRotatingSlow(float xsensYaw) {
	// Check if robot has been rotating sufficiently slow for several time steps
	static int rotateCounter = 0;
	static float startXsensYaw = 0;
	if (fabs(constrainAngle(startXsensYaw - xsensYaw)) < 0.01) {
		rotateCounter++;
	} else {
		rotateCounter = 0;
		startXsensYaw = xsensYaw;
	}
	if (rotateCounter > 10) {
		rotateCounter = 0;
		startXsensYaw = xsensYaw;
		return true;
	}
	return false;
}

//Scales the angle to the range Pi to -Pi in radians
//float constrainAngle(float x){
//    x = fmodf(x + M_PI, 2*M_PI);
//    if (x < 0)
//        x += 2*M_PI;
//    return x - M_PI;
//}
