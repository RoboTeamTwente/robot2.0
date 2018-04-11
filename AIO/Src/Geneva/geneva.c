/*
 * geneva.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include "geneva.h"
#include "pid/pid.h"

#define GENEVA_CAL_SENS_CNT 390
#define GENEVA_POSITION_DIF_CNT 240
#define GENEVA_MAX_ALLOWED_OFFSET 0.2*240

geneva_states geneva_state = geneva_idle;

uint geneva_cnt;


int geneva_Encodervalue();

PID_controller_HandleTypeDef Geneva_pid = {
		.pid = {0,0,0},
		.K_terms = {5.0F, 1.0F, 0.3F},
		.ref = 0.0F,
		.timestep = 0.0F,
		.actuator = &htim10,
		.actuator_channel = TIM_CHANNEL_1,
		.CallbackTimer = &htim6,
		.CLK_FREQUENCY = 48000000.0F,
		.current_pwm = 0,
		.dir[0] = Geneva_dir_B_Pin,
		.dir[1] = Geneva_dir_A_Pin,
		.dir_Port[0] = Geneva_dir_B_GPIO_Port,
		.dir_Port[1] = Geneva_dir_A_GPIO_Port,
};

void geneva_Init(){
	geneva_state = geneva_setup;
	pid_Init(&Geneva_pid);
	HAL_TIM_Base_Start(&htim2);
	geneva_cnt  = HAL_GetTick();
}

void geneva_Update(){
	  switch(geneva_state){
	  case geneva_idle:
		  break;
	  case geneva_setup:// While in setup, slowly move towards the sensor
		  if(HAL_GPIO_ReadPin(Geneva_cal_sens_GPIO_Port, Geneva_cal_sens_Pin)){
			  if((HAL_GetTick() - geneva_cnt) < 100){
				  geneva_state = geneva_too_close;
			  }else{
				  HAL_GPIO_EXTI_Callback(Geneva_cal_sens_Pin);
			  }
		  }else{
			  Geneva_pid.ref = (HAL_GetTick() - geneva_cnt)*1;
		  }
		  break;
	  case geneva_too_close:
		  if(HAL_GPIO_ReadPin(Geneva_cal_sens_GPIO_Port, Geneva_cal_sens_Pin)){
			  Geneva_pid.ref = -200;
		  }else{
			  geneva_state = geneva_setup;
		  }
		  break;
	  case geneva_returning:// while returning move to the middle position
		  if(50 > (geneva_Encodervalue())){
			  geneva_state = geneva_running;
		  }else{
			  Geneva_pid.ref = 0;
		  }
		  break;
	  case geneva_running:
		  break;
	  }
}

void geneva_Control(){
	if(geneva_idle != geneva_state){
		pid_Control(geneva_Encodervalue(),&Geneva_pid);
	}
}

int geneva_Encodervalue(){
	return (int32_t)__HAL_TIM_GetCounter(&htim2);
}


void geneva_SensorCallback(){
	switch(geneva_state){
	case geneva_idle:
		break;
	case geneva_setup:
		__HAL_TIM_SET_COUNTER(&htim2, GENEVA_CAL_SENS_CNT);
		Geneva_pid.ref = 0;
		geneva_state = geneva_returning;
		break;
	case geneva_too_close:
		break;
	case geneva_returning:
		break;
	case geneva_running:
		break;
	}
}

void geneva_SetState(geneva_states state){
	geneva_state = state;
}


geneva_positions geneva_SetPosition(geneva_positions position){
	switch(position){
	case geneva_rightright:
		Geneva_pid.ref = 2 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_right:
		Geneva_pid.ref = 1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_middle:
		Geneva_pid.ref = 0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_left:
		Geneva_pid.ref = -1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_leftleft:
		Geneva_pid.ref = -2 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_none:
		break;
	}
	return geneva_GetPosition();
}

geneva_positions geneva_GetPosition(){
	int enc = geneva_Encodervalue();
	if((enc % GENEVA_POSITION_DIF_CNT) > GENEVA_MAX_ALLOWED_OFFSET){
		return geneva_none;
	}
	return 2 + (geneva_Encodervalue()/GENEVA_POSITION_DIF_CNT);
}
