
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include "PuttyInterface/PuttyInterface.h"
#include "address/address.h"
#include "Geneva/geneva.h"
#include "DO/DO.h"
#include "Ballsensor/ballsensor.h"
//#include "myNRF24.h"
#include "wheels/wheels.h"
#include "kickchip/kickchip.h"
#include "MTi/MTiControl.h"
#include "wireless/wireless.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STOP_AFTER 250 //ms
PuttyInterfaceTypeDef puttystruct;
bool battery_empty = false;
bool user_control = false;
bool print_encoder = false;
bool print_euler = false;
bool wheels_testing = false;
float wheels_testing_power = 30;
bool keyboard_control = false;
bool started_icc = false;

float velocityRef[3] = {0};
float wheelsPWM[4] = {0,0,0,0};

uint kick_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleCommand(char* input);
/*
 * uint is the value to display
 * n_leds chooses bitwise which leds to show the uint on, 0 means no edit
 */
void Uint2Leds(uint8_t uint, uint8_t n_leds);

void dribbler_SetSpeed(uint8_t percentage);
void dribbler_Init();

bool Wireless_newData();
void Wireless_Init();
void Wireless_newPacketHandler();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);
//  geneva_Init();
  DO_Init();
  dribbler_Init();
  ballsensorInit();
  wheels_Init();
  MT_Init();

  kick_Init();

  uint printtime = 0;


  Wireless_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		preparedAckData.roboID = localRobotID;
		float wheelsPWM2[4] = {wheelsPWM[0],wheelsPWM[1],wheelsPWM[2],wheelsPWM[3]};
		wheels_SetOutput(wheelsPWM2);
		HAL_GPIO_TogglePin(Switch_GPIO_Port,Switch_Pin);

		if(Wireless_newData()) {
			Wireless_newPacketHandler();
		}

		//wheels
		float wheels[4];
		int rotSign = 1;
		if(receivedRoboData.driving_reference){
			rotSign = -1;
		}
		calcMotorSpeeds ((float)receivedRoboData.rho/ 1000.0F, (float)receivedRoboData.theta * (2*M_PI/512), rotSign, (float)(receivedRoboData.velocity_angular/180.0)*M_PI, wheels);
		//uprintf("[%f, %f, %f, %f]\n\r", wheels[wheels_RF], wheels[wheels_RB],  wheels[wheels_LB], wheels[wheels_LF]);
		wheels_SetOutput(wheels);

		//dribbler
		dribbler_SetSpeed(receivedRoboData.velocity_dribbler);

		if(kickchip_command) { //received instruction to kick or chip
		  if ((HAL_GetTick() - kick_timer) > 0) {
				kick_timer = HAL_GetTick() + 1000U;
				if(receivedRoboData.do_chip) {
					kick_Chip((receivedRoboData.kick_chip_power*100)/255);
				}
				else {
					kick_Kick((receivedRoboData.kick_chip_power*100)/255);
				}
				kickchip_command = false;
			}
		}

	  if((HAL_GetTick() - LastPackageTime > STOP_AFTER)/* && !user_control*/){;
	  	  float wheel_powers[4] = {0, 0, 0, 0};
		  wheels_SetOutput(wheel_powers);
	  }
	  if(HAL_GPIO_ReadPin(empty_battery_GPIO_Port, empty_battery_Pin)){
		  uprintf("Battery empty!\n\r");
		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
		  wheels_DeInit();
		  kick_DeInit();
		  preparedAckData.batteryState = 0;
// 		  BATTERY IS ALMOST EMPTY!!!!!
//		  battery_empty = true;
//		  dribbler_SetSpeed(0);
	  }
	  else {
		  preparedAckData.batteryState = 1;
	  }

	  preparedAckData.ballSensor = ballsensorMeasurementLoop(receivedRoboData.do_kick, receivedRoboData.do_chip, receivedRoboData.kick_chip_power);

	  geneva_Update();
	  MT_Update();
	  if((HAL_GetTick() - printtime > 1000)){
		  printtime = HAL_GetTick();

		  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);

		  if(*MT_GetStatusWord() & 0b00011000){
			  uprintf("in NRU; ")
		  }else if(started_icc == false){
			  started_icc = true;
			  MT_UseIcc();
		  }
		  uprintf("MT status suc/err = [%u/%u]\n\r", MT_GetSuccErr()[0], MT_GetSuccErr()[1]);
		  //uprintf("status word [%08lx]\n\r", (unsigned long)*MT_GetStatusWord());
		  //uprintf("charge = %d\n\r", HAL_GPIO_ReadPin(Charge_GPIO_Port, Charge_Pin));
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  PuttyInterface_Update(&puttystruct);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
#define TEST_WHEELS_COMMAND "test wheels"
#define SET_FILTER_COMMAND "mt filter"
void HandleCommand(char* input){
	if(strcmp(input, "mt start") == 0){
		uprintf("Starting device MTi\n\r");
		if(MT_succes == MT_Init()){
			uprintf("MTi started.\n\r");
		}else{
			uprintf("No communication with MTi!\n\r");
		}
	}else if(!strcmp(input, "mt stop")){
		uprintf("resetting the MTi.\n\r");
		MT_DeInit();
	}else if(strcmp(input, "mt config") == 0){
		MT_GoToConfig();
	}else if(!strcmp(input, "mt measure")){
		MT_GoToMeasure();
	}else if(!strcmp(input, "mt options")){
		MT_ReqOptions();
	}else if(!strcmp(input, "mt setoptions")){
		MT_SetOptions();
	}else if(!strcmp(input, "mt icc")){
		MT_UseIcc();
	}else if(!strcmp(input, "mt norotation")){
		MT_NoRotation(10);
	}else if(!memcmp(input, SET_FILTER_COMMAND , strlen(SET_FILTER_COMMAND))){
		MT_SetFilterProfile(strtol(input + 1 + strlen(SET_FILTER_COMMAND), NULL, 10));
	}else if(strcmp(input, "mt factoryreset") == 0){
		uprintf("Resetting the configuration.\n\r");
		MT_FactoryReset();
	}else if(strcmp(input, "mt reqfilter") == 0){
		uprintf("requesting current filter profile.\n\r");
		MT_ReqFilterProfile();
	}else if(memcmp(input, "mt setconfig", strlen("mt setconfig")) == 0){
		MT_BuildConfig(XDI_PacketCounter, 100, false);
		MT_BuildConfig(XDI_FreeAcceleration, 100, false);
		MT_BuildConfig(XDI_EulerAngles, 100, true);
	}else if(strcmp(input, "reqconfig") == 0){
		uprintf("requesting output configuration mode\n\r");
		MT_RequestConfig();
	}else if(!strcmp(input, "address")){
		uprintf("address = [%d]\n\r", localRobotID);
	}else if(!strcmp(input, "example2")){
		uprintf("stop!\n\r");
	}else if(!strcmp(input, "geneva")){
		uprintf("position = [%u]\n\r", geneva_GetPosition());
	}else if(!strcmp(input, "geneva stop")){
		geneva_SetState(geneva_idle);
	}else if(!strcmp(input, "euler")){
		print_euler = ! print_euler;
		uprintf("print_euler = %d\n\r", print_euler);
	}else if(!memcmp(input, "geneva" , strlen("geneva"))){
		geneva_SetPosition(2 + strtol(input + 1 + strlen("geneva"), NULL, 10));
	}else if(!memcmp(input, "control" , strlen("control"))){
		geneva_SetPosition(2 + strtol(input + 1 + strlen("control"), NULL, 10));
	}else if(!memcmp(input, "kick" , strlen("kick"))){
		kick_Kick(strtol(input + 1 + strlen("kick"), NULL, 10));
	}else if(!memcmp(input, "chip" , strlen("chip"))){
		kick_Chip(strtol(input + 1 + strlen("chip"), NULL, 10));
	}else if(!memcmp(input, "block" , strlen("block"))){
		kick_Stateprint();
	}else if(!memcmp(input, TEST_WHEELS_COMMAND, strlen(TEST_WHEELS_COMMAND))){
		wheels_testing_power = atoff(input + strlen(TEST_WHEELS_COMMAND));
		wheels_testing = (wheels_testing_power <= -10 || wheels_testing_power >= 10);
		if((wheels_testing)){
			uprintf("wheels test on, pwm [%f]\n\r", wheels_testing_power);
		}
	}else if(!memcmp(input, "dribble", strlen("dribble"))){
		uint8_t speed = strtol(input + strlen("dribble"), NULL, 10);
		dribbler_SetSpeed(speed);
		uprintf("speed is set to[%lu]\n\r", __HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1));
	}

	else if(!strcmp(input, "keyboard control")){
		uprintf("going to keyboard control\r\npress escape to stop\n\r");
		keyboard_control = true;
		wheels_testing = true;
	}
}

void Uint2Leds(uint8_t uint, uint8_t n_leds){
	if(!n_leds) n_leds = 0xff;
	if(n_leds & 0b00000001) HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin, uint & 0b00000001);
	if(n_leds & 0b00000010) HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, uint & 0b00000010);
	if(n_leds & 0b00000100) HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin, uint & 0b00000100);
	if(n_leds & 0b00001000) HAL_GPIO_WritePin(LD4_GPIO_Port,LD4_Pin, uint & 0b00001000);
	if(n_leds & 0b00010000) HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin, uint & 0b00010000);
	if(n_leds & 0b00100000) HAL_GPIO_WritePin(LD6_GPIO_Port,LD6_Pin, uint & 0b00100000);
}

void dribbler_SetSpeed(uint8_t speed){
	if(speed > 7){
		speed = 7;
	}
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (7 - speed) * (MAX_PWM / 7));
}

void dribbler_Init(){
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	dribbler_SetSpeed(0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart3.Instance){//input from the PC
		puttystruct.huart_Rx_len = 1;
		puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	}else if(huart->Instance == huartMT.Instance){// Input from the Xsens
		MT_UART_RxCpltCallback();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim->Instance == htim6.Instance){
		geneva_Control();
	}else if(htim->Instance == htim7.Instance){

		// get xsens data
		float * accptr;
		accptr = MT_GetAcceleration();
		float xsensYaw = MT_GetAngles()[2]/180*M_PI;

		// check for calibration possibilities (if angle has not changed much for a while)
		int bufferSize = 5;
		int timesteps = 100;
		static float offset = 0;
		static float xsensAngles[5] = {0};
		static float angle0 = 0;
		static int counter = 0;
		if (fabs(angle0 - xsensAngles[0]) < 0.01) {
			if (counter>timesteps) {
				float avg_xsens = 0;
				float avg_vision = 0;
				for (int i = 0; i<bufferSize; i++) {
					avg_xsens += xsensAngles[i];
					avg_vision += 0; //TODO
				}
				offset = avg_vision - avg_xsens;
				//uprintf("[%f]\n\r", offset);
				counter = 0;
			} else if (counter > timesteps - bufferSize) {
				xsensAngles[timesteps-counter] = xsensYaw;
			}
		} else {
			angle0 = xsensYaw;
			counter = 0;
		}
		counter++;

		//
		float xsensData[3];
		xsensData[body_x] = -accptr[0];
		xsensData[body_y] = -accptr[1];
		xsensData[body_w] = xsensYaw + offset;
		//uprintf("[x: %.6f \t y: %.6f \t angular: %.6f]\n",xsensData[body_x], xsensData[body_y], xsensData[body_w]);

		fillXSensData(xsensData);

		bool DO_enabled = false;
		bool useAngleControl = false;
		bool refIsAngle = false;
		DO_Control(velocityRef, xsensData, DO_enabled, useAngleControl, refIsAngle, wheelsPWM);
//		wheelsPWM[0] = -10;
//		wheelsPWM[2] = -10;
		//if(wheels_testing)	uprintf("wheels speeds are[%f %f %f %f]\n\r", wheels_GetSpeed(wheels_LF), wheels_GetSpeed(wheels_RF), wheels_GetSpeed(wheels_RB), wheels_GetSpeed(wheels_LB));
		//if(wheels_testing)	uprintf("wheels encoders are[%d %d %d %d]\n\r", wheels_GetEncoder(wheels_RF), wheels_GetEncoder(wheels_RB), wheels_GetEncoder(wheels_LB), wheels_GetEncoder(wheels_LF));
//		float * euler;
//		euler = MT_GetAngles();
//		if(Xsens_state == Xsens_Measure && print_euler)	uprintf("euler angles[%f, %f, %f]\n\r", euler[0], euler[1], euler[2]);
	}else if(htim->Instance == htim13.Instance){
		kick_Callback();
	}else if(htim->Instance == htim14.Instance){
		wheels_Callback();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == SPI1_IRQ_Pin){
		//Wireless_newPacketHandler();
	}else if(GPIO_Pin == Geneva_cal_sens_Pin){
		// calibration  of the geneva drive finished
		uprintf("geneva sensor callback\n\r");
		geneva_SensorCallback();
	}
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
