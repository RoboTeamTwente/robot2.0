/*
 * pid.h
 *
 *  Created on: Nov 16, 2017
 *      Author: Leon
 */

#ifndef PID_H_
#define PID_H_

#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct{
	float P;
	float I;
	float D;
}PID;// holds the current PID values
typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PID_Terms;// holds the control factors

typedef struct{
	PID_Terms K_terms;
	PID pid;
	float ref;
	float timestep;
	TIM_HandleTypeDef* actuator;
	uint32_t actuator_channel;
	TIM_HandleTypeDef* CallbackTimer;
	float CLK_FREQUENCY;
	int16_t current_pwm;
	uint16_t dir[2];					// pin number of channel A and B respectively
	GPIO_TypeDef * dir_Port[2];			// GPIO Port of channel A and B respectively
	uint16_t max_pwm;
}PID_controller_HandleTypeDef;

// directly set the current output, if the pid control loop is running, this will not have much effect
void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc);
// return all pid controller parameters
PID_controller_HandleTypeDef pid_GetControllerValue();
/* return current P, I and D values
 *
 */
PID pid_GetCurrentPIDValues(PID_controller_HandleTypeDef* pc);
// Returns the current output to the actuator
int16_t pid_GetCurrentOutput(PID_controller_HandleTypeDef* pc);
// Set the reference value
void pid_SetReference(float ref, PID_controller_HandleTypeDef* pc);
// calculate the current speed according to the encoder values
void pid_Init(PID_controller_HandleTypeDef* PID_controller);
void pid_Deinit(PID_controller_HandleTypeDef* PID_controller);
// controls the output, to be called on a regular schedule
void pid_Control(float current_speed, PID_controller_HandleTypeDef* pc);
#endif /* PID_H_ */
