/*
 * encoder.h
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

#include <stdbool.h>
#include "tim.h"
#include "gpio.h"

/* holds values for one encoder
 * holds both constants and output
 */
typedef struct{
	uint16_t CHANNEL[2];					// pin number of channel A and B respectively
	GPIO_TypeDef * CHANNEL_Port[2];			// GPIO Port of channel A and B respectively
	TIM_HandleTypeDef* MeasurementTimer;	// This is the handle of the timer that will be used to measure the time between interrupts
	int32_t cnt[2];								//counter for the edges of channel A and B respectively, counts up or down depending on direction
	bool high[2];							// are channel A and B respectively currently High
	int8_t direction;						// current direction, 1 is forward voltage, -1 is reverse voltage 0 is not yet known
	uint32_t period;						// last calculated period between the sample times
	uint32_t last_tim_sample;				// latest sample time of the rising edge
	uint32_t prev_tim_sample;				// previous sample time of the rising edge
	float speed;							// speed calculated by encoder_CalculateSpeed()
	float COUNTS_PER_ROTATION;				// Encoder constant
	float CLK_FREQUENCY;					// frequency of the MeasurementTimer clock
	float GEAR_RATIO;						// Gear Ratio of the motor
}encoder_HandleTypeDef;

// Initialize the encoder pointed to by enc
void encoder_Init(encoder_HandleTypeDef* enc);
/* handle input from the encoder lines, should be called within the external interrupt call
 * and needs to be called for all encoders which could be affected
 */
void encoder_Input(uint16_t channel, encoder_HandleTypeDef* enc);
/* Calculate the speed of encoder enc, this function also stores the speed in teh object
 *
 */
float encoder_CalculateSpeed(encoder_HandleTypeDef* enc);
/* Returns the last calculated speed from this encoder
 *
 */
float encoder_GetLatestSpeed(encoder_HandleTypeDef* enc);

#endif /* ENCODER_H_ */
