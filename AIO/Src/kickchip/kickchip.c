#include "gpio.h"
#include <stdbool.h>
#include "tim.h"
#include "kickchip.h"
#include "../PuttyInterface/PuttyInterface.h"

bool chargeBlock = 0;


void kick_Kick(int percentage)
{
	HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable charging
	chargeBlock = 1;													// Block charging
	HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_SET); 			// Kick on
	__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);							// Clear timer
	__HAL_TIM_SET_AUTORELOAD(&htim13, percentage << 2);					// Set kick time
	HAL_TIM_Base_Start_IT(&htim13);   									// Start timer for kick off

}

void kick_Chip(int percentage)
{
	HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable charging
	chargeBlock = 1;													// Block charging
	HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_SET); 			// Chip on
	__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);							// Clear timer
	__HAL_TIM_SET_AUTORELOAD(&htim13, percentage << 2);					// Set kick time
	HAL_TIM_Base_Start_IT(&htim13);   									// Start timer for kick off

}

void kick_Callback()
{
	HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
	HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
	chargeBlock = 0;
	HAL_TIM_Base_Stop(&htim13);
	__HAL_TIM_SET_COUNTER(&htim13, 0);
	uprintf("Holabola\n\r"/*, chargeBlock*/);
}


void kick_ChargeUpdate()
{
	if(chargeBlock == 0)			// If charging is allowed
	{
		if(HAL_GetTick()%1000>900) // If between 0.9 an 1.0 of each second
		{
			if(HAL_GPIO_ReadPin(Charge_done_GPIO_Port, Charge_done_Pin))			// If the charger has reached 300V in the past second
			{
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);	// Turn charging off
			}
		}else{
			HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Turn charging on
		}
	}

}

void kick_printblock()
{
	uprintf("Block = [%d]\n\r", chargeBlock);
}
