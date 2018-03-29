#include "pid.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

#define SWITCH_OFF_TRESHOLD 200
#define MAX_DUTY_CYCLE_INVERSE_FRACTION 4

static int32_t ClipInt(int32_t input, int32_t min, int32_t max){
	return (input > max) ? max : (input < min) ? min : input;
}

static int32_t ClipFloat(float input, float min, float max){
	return (input > max) ? max : (input < min) ? min : input;
}
static float AverageErr( float * errors, int n_floats){
	float total = 0;
	for(int i = 0; i < n_floats; i++){
		total += *errors++;
	}
	return (total / n_floats);
}
void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc){
	if(pwm < -SWITCH_OFF_TRESHOLD){
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 1);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 0);
	}else if(pwm > SWITCH_OFF_TRESHOLD){
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 0);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 1);
	}else{
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 0);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 0);
	}
	pwm = abs(pwm);
	pc->current_pwm = ClipInt(pwm, 0, pc->actuator->Init.Period/MAX_DUTY_CYCLE_INVERSE_FRACTION);// Power limited by having maximum duty cycle
	__HAL_TIM_SET_COMPARE(pc->actuator, pc->actuator_channel, pc->current_pwm);
}
int16_t pid_GetCurrentOutput(PID_controller_HandleTypeDef* pc){
	return pc->current_pwm;
}
void pid_SetReference(float ref, PID_controller_HandleTypeDef* pc){
	pc->ref = ref;
}

PID pid_GetCurrentPIDValues(PID_controller_HandleTypeDef* pc){
	return pc->pid;
}
void pid_Init(PID_controller_HandleTypeDef* PID_controller){
	static uint8_t n_motors = 0;
	n_motors++;
	if(PID_controller == NULL)
		return;
	PID_controller->max_pwm = PID_controller->actuator->Init.Period;
	HAL_TIM_PWM_Start(PID_controller->actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(PID_controller->CallbackTimer);
	PID_controller->timestep = (((float)n_motors*(float)PID_controller->CallbackTimer->Init.Period))/(PID_controller->CLK_FREQUENCY/((float)(PID_controller->CallbackTimer->Init.Prescaler + 1)));
	__HAL_TIM_SET_AUTORELOAD(PID_controller->CallbackTimer, __HAL_TIM_GET_AUTORELOAD(PID_controller->CallbackTimer)/ n_motors);
}

#define FILTER_SIZE 1 // DO NOT USE (BESIDES 1)
void pid_Control(float sensor_output, PID_controller_HandleTypeDef* pc){
	static float prev_e[FILTER_SIZE] = {0};
	static uint e_cnt;
	float err = pc->ref - sensor_output;
	pc->pid.P = pc->K_terms.Kp*err;
	pc->pid.I += pc->K_terms.Ki*err*pc->timestep;

	pc->pid.D = (pc->K_terms.Kd*(err-AverageErr(prev_e, FILTER_SIZE)))/pc->timestep;
	prev_e[e_cnt++] = err;
	e_cnt %= FILTER_SIZE;

	pc->pid.P = ClipFloat(pc->pid.P, -2 * pc->max_pwm, 2 * pc->max_pwm);
	pc->pid.I = ClipFloat(pc->pid.I, -2 * pc->max_pwm, 2 * pc->max_pwm);
	pc->current_pwm = (int16_t)(pc->pid.P + pc->pid.I + pc->pid.D);
	pid_SetOutput(pc->current_pwm, pc);
}
