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

DO_States DO_Init();

DO_States DO_Update();

#endif /* DO_DO_H_ */
