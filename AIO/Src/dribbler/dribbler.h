/*
 * dribbler.h
 *
 *  Created on: May 23, 2018
 *      Author: Leon
 */

#ifndef DRIBBLER_DRIBBLER_H_
#define DRIBBLER_DRIBBLER_H_

/*	initialize the dribbler
 *
 */
void dribbler_Init();

/*	deinitialize the dribbler
 *
 */
void dribbler_Deinit();

/*	set the dribbler speed
 * 	args:
 * 		speed [0...7]
 */
void dribbler_SetSpeed(int speed);

#endif /* DRIBBLER_DRIBBLER_H_ */
