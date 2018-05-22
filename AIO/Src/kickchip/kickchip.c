#include "gpio.h"
#include <stdbool.h>
#include "tim.h"
#include "kickchip.h"
#include "../PuttyInterface/PuttyInterface.h"

#define Timestep 20
kick_states kick_state = kick_Idle;
bool TicToc;
int Callback_time;


// Init variables and call kick_Callback(), which will keep calling itself.
void kick_Init(){
	kick_state = kick_Charging;
	HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
	HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
	HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// kick_Charging on
	TicToc = 0;
	Callback_time = 0;
	kick_Callback();
}


void kick_DeInit(){
	HAL_TIM_Base_Stop(&htim13);
	kick_state = kick_Idle;
	HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
	HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
	HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);	// kick_Charging on
}
/*
 *  Initiates the kick_Kicking of the robot at a percentage of the max power.
 *  Will only do this if in state kick_Ready.
 *  Sets a timer for kick_Callback() to stop the kick. The power of the kick depends on the time interval between this function and the callback.
 */
void kick_Kick(int percentage)
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(kick_state == kick_Ready)
	{
		kick_state = kick_Kicking;												// Block kick_Charging
		HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable kick_Charging
		uprintf("kick_Kicking\n\r");
		HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_SET); 			// Kick on

		HAL_TIM_Base_Stop(&htim13);											// Stop timer
		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim13, 0);									// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, percentage);						// Set kick time
		HAL_TIM_Base_Start_IT(&htim13);   									// Start timer for kick off
	}
}

/*
 * Initiates the chipping of the robot at a percentage of the max power.
 *  Will only do this if in state kick_Ready.
 *  Sets a timer for kick_Callback() to stop the chip.
 */
void kick_Chip(int percentage)
{
	percentage = percentage * 2;	// chipping benefits from longer duration
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(kick_state == kick_Ready)
	{
		HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable kick_Charging
		kick_state = kick_Kicking;												// Block kick_Charging
		uprintf("kick_Kicking\n\r");
		HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_SET); 			// Chip on

		HAL_TIM_Base_Stop(&htim13);											// Stop timer
		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim13, 0);									// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, percentage << 1);						// Set kick time
		HAL_TIM_Base_Start_IT(&htim13);   									// Start timer for kick off
	}
}


/*
 * Handles all the callbacks for kick_Kicking, chipping and kick_Charging
 * This function keeps setting a new timer to call itself.
 */
void kick_Callback()
{
		HAL_TIM_Base_Stop(&htim13);											// Stop the timer
		__HAL_TIM_SET_COUNTER(&htim13, 0);

		if(kick_state == kick_Kicking)
		{
			HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
			HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
			kick_state = kick_Charging;
			HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Turn kick_Charging on
			Callback_time = 100*Timestep;										// Set timer to 100ms
		}
		else if(kick_state == kick_Charging)
		{
			if(!HAL_GPIO_ReadPin(Charge_done_GPIO_Port, Charge_done_Pin))		// If kick_Charging is done
			{
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);// Turn kick_Charging off
				kick_state = kick_Ready;												// Go to kick_Ready state
			}
			Callback_time = 100*Timestep;										// Set timer to 100ms
		}
		else if(kick_state == kick_Ready)
		{
			if(TicToc == 0){
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);	// Turn kick_Charging off
				Callback_time = 100*Timestep;										// Set timer to 100ms
			}else{
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Turn kick_Charging on
				Callback_time = 900*Timestep;										// Set timer to 900ms
			}
			TicToc = !TicToc;
		}

		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);							// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, Callback_time);					// Set callback time to defined value
		HAL_TIM_Base_Start_IT(&htim13);

}


void kick_Stateprint()
{
	uprintf("Block = [%d]\n\r", kick_state);
}
