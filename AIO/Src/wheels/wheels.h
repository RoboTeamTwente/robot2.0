/*
 * wheels.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef WHEELS_WHEELS_H_
#define WHEELS_WHEELS_H_

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF
}wheels_handles;

void wheels_Init();
void wheels_SetOutput(wheels_handles wheel, float power);
int wheels_GetEncoder(wheels_handles wheel);
float wheels_GetSpeed(wheels_handles wheel);

#endif /* WHEELS_WHEELS_H_ */
