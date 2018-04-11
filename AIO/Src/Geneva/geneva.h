/*
 * geneva.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_


typedef enum{
	geneva_idle,
	geneva_setup,		// at startup it will try to find the edge sensor
	geneva_too_close,	// The Geneva drive started too close to the sensor and needs to move away
	geneva_returning,	// when moving back to the initial/zero position
	geneva_running		// when being operational
}geneva_states;

typedef enum{
	geneva_leftleft,
	geneva_left,
	geneva_middle,
	geneva_right,
	geneva_rightright,
	geneva_none			// While rotating
}geneva_positions;

void geneva_Init();
void geneva_Update();
void geneva_Control();
void geneva_SensorCallback();
void geneva_SetState(geneva_states state);
geneva_positions geneva_SetPosition(geneva_positions position);
geneva_positions geneva_GetPosition();
#endif /* GENEVA_GENEVA_H_ */
