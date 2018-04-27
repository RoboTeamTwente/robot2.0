/*
 * wheels.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef WHEELS_WHEELS_H_
#define WHEELS_WHEELS_H_

#include <stdint.h>

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF
}wheels_handles;

void wheels_Init();
void calcVelocityRef (float magnitude, float direction, int rotSign, float wRadPerSec, float power[4]);
void wheels_SetOutput(float power[4]);
int16_t wheels_GetEncoder(wheels_handles wheel);
float wheels_GetSpeed(wheels_handles wheel);
void wheels_Callback();

#endif /* WHEELS_WHEELS_H_ */
