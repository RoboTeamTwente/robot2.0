#include "gpio.h"
#include <stdbool.h>
#include "tim.h"
#include "kickchip.h"
#include "../PuttyInterface/PuttyInterface.h"

#define Timestep 20
kick_states kick_state;
bool TicToc;
int Callback_time;


// Init variables and call kick_Callback(), which will keep calling itself.
void kick_Init(){
	kick_state = Charging;
	HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
	HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
	HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Charging on
	TicToc = 0;
	Callback_time = 0;
	kick_Callback();
}


/*
 *  Initiates the kicking of the robot at a percentage of the max power.
 *  Will only do this if in state Ready.
 *  Sets a timer for kick_Callback() to stop the kick. The power of the kick depends on the time interval between this function and the callback.
 */
void kick_Kick(int percentage)
{
	if(kick_state == Ready)
	{
		HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable charging
		kick_state = Kicking;												// Block charging
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
 *  Will only do this if in state Ready.
 *  Sets a timer for kick_Callback() to stop the chip.
 */
void kick_Chip(int percentage)
{
	if(kick_state == Ready)
	{
		HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET); 	// Disable charging
		kick_state = Kicking;												// Block charging
		HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_SET); 			// Chip on

		HAL_TIM_Base_Stop(&htim13);											// Stop timer
		__HAL_TIM_CLEAR_IT(&htim13,TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim13, 0);									// Clear timer
		__HAL_TIM_SET_AUTORELOAD(&htim13, percentage);						// Set kick time
		HAL_TIM_Base_Start_IT(&htim13);   									// Start timer for kick off
	}
}


/*
 * Handles all the callbacks for kicking, chipping and charging
 * This function keeps setting a new timer to call itself.
 */
void kick_Callback()
{
		HAL_TIM_Base_Stop(&htim13);											// Stop the timer
		__HAL_TIM_SET_COUNTER(&htim13, 0);

		if(kick_state == Kicking)
		{
			HAL_GPIO_WritePin(Kick_GPIO_Port, Kick_Pin, GPIO_PIN_RESET);		// Kick off
			HAL_GPIO_WritePin(Chip_GPIO_Port, Chip_Pin, GPIO_PIN_RESET);		// Chip off
			kick_state = Charging;
			HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Turn charging on
			Callback_time = 100*Timestep;										// Set timer to 100ms
		}
		else if(kick_state == Charging)
		{
			if(!HAL_GPIO_ReadPin(Charge_done_GPIO_Port, Charge_done_Pin))		// If charging is done
			{
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);// Turn charging off
				kick_state = Ready;												// Go to ready state
				uprintf("Hoi");
			}
			Callback_time = 100*Timestep;										// Set timer to 100ms
		}
		else if(kick_state == Ready)
		{
			if(TicToc == 0){
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_RESET);	// Turn charging off
				Callback_time = 100*Timestep;										// Set timer to 100ms
			}else{
				HAL_GPIO_WritePin(Charge_GPIO_Port, Charge_Pin, GPIO_PIN_SET);		// Turn charging on
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
