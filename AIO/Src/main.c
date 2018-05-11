
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
#include "wireless/myNRF24.h"
#include "wireless/roboNRF24.h"
#include "wireless/debugNRF24.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STOP_AFTER 250 //ms
PuttyInterfaceTypeDef puttystruct;
uint8_t freqChannel = 78;
bool battery_empty = false;
bool user_control = false;
bool print_encoder = false;

bool wheels_testing = false;
float wheels_testing_power = 30;
bool keyboard_control = false;

float velocity[3] = {0};

uint8_t isNrfInitialized = 0;
uint8_t localRobotID = 0xff; //"uninitialized"
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

void Wireless_Init();

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
  //ballsensorInit();
  wheels_Init();
  MT_Init();

  kick_Init();
  uint LastPackageTime = 0;
  uint printtime = 0;
  uint kick_timer = 0;

  Wireless_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  preparedAckData.roboID = localRobotID;
	  HAL_GPIO_TogglePin(Switch_GPIO_Port,Switch_Pin);

		  }

			  //geneva
			  }
				  }
					  kick_Kick((dataStruct.kickForce*100)/255);
				  }else{
					  kick_Chip((dataStruct.kickForce*100)/255);
				  if(dataStruct.chipper){
				  kick_timer = HAL_GetTick() + 1000U;
			  if (dataStruct.kickForce && ((HAL_GetTick() - kick_timer) > 0)){

			  //kicker
			  dribbler_SetSpeed(dataStruct.driblerSpeed);
			  //dribbler
			  wheels_SetOutput(wheels);
			  //uprintf("[%f, %f, %f, %f]\n\r", wheels[wheels_RF], wheels[wheels_RB],  wheels[wheels_LB], wheels[wheels_LF]);
			  calcMotorSpeed ((float)dataStruct.robotVelocity/ 1000.0F, (float)dataStruct.movingDirection * (2*M_PI/512), rotSign, (float)(dataStruct.angularVelocity/180.0)*M_PI, wheels);
			  //uprintf("magn[%u]; angle[%u]\n\r", dataStruct.robotVelocity, dataStruct.angularVelocity);
			  }
				  rotSign = -1;
			  if(dataStruct.rotationDirection){
			  int rotSign = 1;
			  float wheels[4];
			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		  roboCallback(&hspi2, &dataStruct);
		  if(dataStruct.robotID == address){
		  LastPackageTime = HAL_GetTick();
	 if(irqRead(&hspi2)){
	 //ballsensorMeasurementLoop();
	  }else if((HAL_GetTick() - LastPackageTime > STOP_AFTER)/* && !user_control*/){;
	  	  float wheel_powers[4] = {0, 0, 0, 0};
		  wheels_SetOutput(wheel_powers);
	  }
	  if(HAL_GPIO_ReadPin(empty_battery_GPIO_Port, empty_battery_Pin)){
		  uprintf("Battery empty!\n\r");
		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
		  wheels_DeInit();
		  kick_DeInit();
// 		  BATTERY IS ALMOST EMPTY!!!!!
//		  battery_empty = true;
//		  dribbler_SetSpeed(0);
	  }
	  if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port, bs_EXTI_Pin)){
		  // handle the message
	  }
	  geneva_Update();	  MT_Update();
	  if((HAL_GetTick() - printtime > 1000)){
		  printtime = HAL_GetTick();
		  uprintf("encoder values[%i %i %i %i]\n\r", wheels_GetEncoder(wheels_RF), wheels_GetEncoder(wheels_RB), wheels_GetEncoder(wheels_LB), wheels_GetEncoder(wheels_LF))
		  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
		  //printNRFregisters();
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
	}else if(strcmp(input, "mt factoryreset") == 0){
		uprintf("Resetting the configuration.\n\r");
		MT_FactoryReset();
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
		float * accptr;
		accptr = MT_GetAcceleration();
		velocity[0] += *accptr++ / 100;
		velocity[1] += *accptr++ / 100;
		velocity[2] += *accptr   / 100;
		if(wheels_testing)	uprintf("wheels speeds are[%f %f %f %f]\n\r", wheels_GetSpeed(wheels_LF), wheels_GetSpeed(wheels_RF), wheels_GetSpeed(wheels_RB), wheels_GetSpeed(wheels_LB));
		//if(wheels_testing)	uprintf("wheels encoders are[%d %d %d %d]\n\r", wheels_GetEncoder(wheels_RF), wheels_GetEncoder(wheels_RB), wheels_GetEncoder(wheels_LB), wheels_GetEncoder(wheels_LF));
		DO_Control();
	}else if(htim->Instance == htim13.Instance){
		kick_Callback();
	}else if(htim->Instance == htim14.Instance){
		wheels_Callback();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == SPI1_IRQ_Pin){
		if(isNrfInitialized) {
				uprintf("\n\nInterrupt fired.\n");


				int8_t error_code = roboCallback(localRobotID);
				if(error_code) {
					uprintf("RoboCallback failed with error: %i\n", error_code);
				}

				//flushRX();
				clearInterrupts(); //should not be needed
			}
	}else if(GPIO_Pin == Geneva_cal_sens_Pin){
		// calibration  of the geneva drive finished
		geneva_SensorCallback();
	}
}

void Wireless_Init() {
	localRobotID = ReadAddress();
	uprintf("Robot ID: %i\n", localRobotID);

	while(initRobo(&hspi2, RADIO_CHANNEL, localRobotID) != 0) {
		uprintf("Error while initializing nRF wireless module. Check connections.\n");
	}
	isNrfInitialized = 1;
	uprintf("nRF wireless module successfully initialized.\n");

	nrfPrintStatus();
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
