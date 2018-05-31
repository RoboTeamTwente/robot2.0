/*
 * wheels.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef WHEELS_WHEELS_H_
#define WHEELS_WHEELS_H_

#include <stdint.h>

#define FOREACHWHEEL(X) for(wheels_handles X = wheels_RF; X <= wheels_LF; X++)

#define N_WHEELS 4				// number of wheels

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF
}wheels_handles;

void wheels_Init();
void wheels_DeInit();

void calcMotorSpeeds (float magnitude, float direction, int rotSign, float wRadPerSec, float power[4]);
/*	Set a power for each wheel
 * 	param:
 * 		power[4]: an array of four percentages in sequence from RF, RB, LB, LF
 */
void wheels_SetOutput(float power[4]);
/*	returns the encoder count of one whee;
 * 	ret: encoder count
 * 	param:
 * 		wheel: which wheel to get the count from
 */
int16_t wheels_GetEncoder(wheels_handles wheel);
/*	returns the speed of the wheels (resets the encoder of this wheel)
 * 	ret: speed in rotations per second
 * 	param:
 * 		wheel: which wheel to get the speed from
 */
float wheels_GetSpeed(wheels_handles wheel);
/*	will be used to brake before switching directions
 *
 */
void wheels_Callback();

#endif /* WHEELS_WHEELS_H_ */
