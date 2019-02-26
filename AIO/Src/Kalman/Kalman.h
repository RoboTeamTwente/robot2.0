/*
 * Kalman.h
 *
 *  Created on: Feb 19, 2019
 *      Author: kjhertenberg
 */

#ifndef KALMAN_KALMAN_H_
#define KALMAN_KALMAN_H_

void Kalman_init();
void Kalman(float vel[2], float controlInput[4]);
void getState(float state[4]);
void getKGain(float gain[4][2]);
void getP(float P[16]);
#endif /* KALMAN_KALMAN_H_*/
