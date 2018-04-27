/*
 * DO.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef DO_DO_H_
#define DO_DO_H_

typedef enum {
	DO_succes,
	DO_error
}DO_States;

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

DO_States DO_Init();

DO_States DO_Control(float velocityRef[3], float xsensData[3]);

#endif /* DO_DO_H_ */
