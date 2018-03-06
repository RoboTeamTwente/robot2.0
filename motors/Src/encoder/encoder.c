/*
 * encoder.c
 *
 *  Created on: Nov 20, 2017
 *      Author: Leon
 */
#include "encoder.h"

static uint32_t FindTimeDifference(uint32_t newest, uint32_t previous){
	return newest < previous ?  previous - newest : newest - previous;
}
void encoder_Init(encoder_HandleTypeDef* enc){
	if(enc == NULL)
		return;
	__HAL_TIM_ENABLE(enc->MeasurementTimer);
}
void encoder_Input(uint16_t channel, encoder_HandleTypeDef* enc){
	if(channel == enc->CHANNEL[0]){
		enc->high[0] = HAL_GPIO_ReadPin(enc->CHANNEL_Port[0], enc->CHANNEL[0]);
		if(enc->high[0]){
			enc->prev_tim_sample = enc->last_tim_sample;
			enc->last_tim_sample = __HAL_TIM_GET_COUNTER(enc->MeasurementTimer);
			enc->period = FindTimeDifference(enc->last_tim_sample, enc->prev_tim_sample);
		}
		enc->direction = enc->high[1] != enc->high[0] ? 1 : -1;
		enc->cnt[0] += enc->direction;
	}else if(channel == enc->CHANNEL[1]){
		enc->high[1] = HAL_GPIO_ReadPin(enc->CHANNEL_Port[1], enc->CHANNEL[1]);

		enc->direction = enc->high[0] != enc->high[1] ? -1 : 1;
		enc->cnt[1] += enc->direction;
	}
}

float encoder_CalculateSpeed(encoder_HandleTypeDef* enc){
	float rot_speed = 0;
	float rot_period = (enc->period > FindTimeDifference(__HAL_TIM_GET_COUNTER(enc->MeasurementTimer), enc->last_tim_sample) ? enc->period : FindTimeDifference(__HAL_TIM_GET_COUNTER(enc->MeasurementTimer), enc->last_tim_sample))* (enc->COUNTS_PER_ROTATION);
	rot_speed = 1.0F/rot_period;
	rot_speed = rot_speed * ((enc->CLK_FREQUENCY/(enc->MeasurementTimer->Init.Prescaler+1))/enc->GEAR_RATIO) * enc->direction;

	return enc->speed = rot_speed;
}

float encoder_GetLatestSpeed(encoder_HandleTypeDef* enc){
	return enc->speed;
}
