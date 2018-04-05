
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
#include "dma.h"
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
#include "geneva/geneva.h"
#include "DO/DO.h"
#include "myNRF24.h"
#include "wheels/wheels.h"
#include "MTi/MTiControl.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define STOP_AFTER 250 //ms

PuttyInterfaceTypeDef puttystruct;
int8_t address = -1;
uint8_t freqChannel = 78;
bool battery_empty = false;
bool user_control = false;
bool print_encoder = false;

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

void MTiPrintOutputConfig(struct XbusMessage const* message);
void MTiErrorHandler(struct XbusMessage const* message);
void printMessageData(struct XbusMessage const* message);

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
  MX_DMA_Init();
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
  /* USER CODE BEGIN 2 */
  address = ReadAddress();
  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);
  geneva_Init();
  DO_Init();
  dribbler_Init();
  wheels_Init();
  MT_Init();

  nssHigh(&hspi2);
  initRobo(&hspi2, freqChannel, address);
  dataPacket dataStruct;
  uint LastPackageTime = 0;
  uint printtime = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(irqRead(&hspi2)){
		  LastPackageTime = HAL_GetTick();
		  roboCallback(&hspi2, &dataStruct);
		  if(dataStruct.robotID == address){
			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			  float wheels[4];
			  int rotSign = 1;
			  if(dataStruct.rotationDirection){
				  rotSign = -1;
			  }
			  calcMotorSpeed ((float)dataStruct.robotVelocity/ 1000.0F, (float)dataStruct.movingDirection * (2*M_PI/512), rotSign, (float)(dataStruct.angularVelocity/180.0)*M_PI, wheels);
			  uprintf("[%f, %f, %f, %f]\n\r", wheels[wheels_RF], wheels[wheels_RB],  wheels[wheels_LB], wheels[wheels_LF]);
			  wheels_SetOutput(wheels);
			  //dribbler
			  dribbler_SetSpeed((dataStruct.driblerSpeed*100)/7);

			  //kicker
			  if (dataStruct.kickForce != 0){

			  }
		  }

	  }else if((HAL_GetTick() - LastPackageTime > STOP_AFTER)/* && !user_control*/){;
	  	  float wheel_powers[4] = {0, 0, 0, 0};
		  wheels_SetOutput(wheel_powers);
	  }
	  if(!HAL_GPIO_ReadPin(empty_battery_GPIO_Port, empty_battery_Pin)){
		  // BATTERY IS ALMOST EMPTY!!!!!
		  battery_empty = true;
		  dribbler_SetSpeed(0);
	  }
	  if(HAL_GPIO_ReadPin(bs_EXTI_GPIO_Port, bs_EXTI_Pin)){
		  // handle the message
	  }
	  geneva_Update();
	  //if(huartMT.hdmarx->StreamIndex - huartMT.hdmarx->StreamBaseAddress)  uprintf("StreamIndex = [%lu]\n\r", huartMT.hdmarx->StreamIndex/* - huartMT.hdmarx->StreamBaseAddress*/);
	  MT_Update();
	  if((HAL_GetTick() - printtime > 500)){
		  printtime = HAL_GetTick();
		  if(print_encoder) uprintf("encoder values[%i %i %i %i]\n\r", wheels_GetEncoder(wheels_RF), wheels_GetEncoder(wheels_RB), wheels_GetEncoder(wheels_LB), wheels_GetEncoder(wheels_LF));
		  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
void HandleCommand(char* input){
	if(strcmp(input, "start") == 0){
		TextOut("Starting device MTi\n\r");
		if(MT_succes == MT_StartOperation(true)){
			TextOut("Communication with MTi started, in config state.\n\r");
		}else{
			TextOut("No communication with MTi!\n\r");
		}
	}else if(strcmp(input, "start2") == 0){
		TextOut("Starting device MTi\n\r");
		if(MT_succes == MT_StartOperation(false)){
			TextOut("Communication with MTi started, going to measure state in .5 seconds.\n\r");
		}else{
			TextOut("No communication with MTi!\n\r");
		}
	}else if(strcmp(input, "config") == 0){
		struct XbusMessage mess = { .mid = XMID_GoToConfig,
									.data = NULL,
									.length = 0};
		uint8_t cnt = 0;
		do{
			MT_SendXbusMessage(mess);
			cnt++;
		}while(MT_WaitForAck(XMID_GoToConfigAck) != MT_succes && cnt < 20 );
		uprintf("cnt = %d\n\r",  cnt);
		if(cnt < 20){
			TextOut("In config state.\n\r");
		}else{
			TextOut("No GoToConfigAck received.\n\r");
		}
	}else if(!strcmp(input, "measure")){
		struct XbusMessage mess = { .mid = XMID_GoToMeasurement,
									.data = NULL,
									.length = 0};
		uint8_t cnt = 0;
		do {
			MT_SendXbusMessage(mess);
			cnt++;
		}while(MT_WaitForAck(XMID_GoToMeasurementAck) != MT_succes && cnt < 20 );
		uprintf("cnt = %d\n\r",  cnt);
		if(cnt < 20){
			TextOut("In measurement state.\n\r");
		}else{
			TextOut("No GoToMeasurementAck received.\n\r");
		}

	}else if(strcmp(input, "reqdata") == 0){
		TextOut("Sending request data message.\n\r");
		struct XbusMessage mess = { .mid = XMID_ReqData,
									.length = 0,
									.data = NULL};
		MT_SendXbusMessage(mess);
		MT_ReadNewMessage(0);
	}else if(strcmp(input, "factoryreset") == 0){
		TextOut("Resetting the configuration.\n\r");
		struct XbusMessage mess = { .mid = XMID_RestoreFactoryDef,
									.length = 0,
									.data = NULL};
		MT_SendXbusMessage(mess);
	}else if(strcmp(input, "readMT") == 0){
		TextOut("Reading MT messages\n\r");
		MT_ReadNewMessage(0);
	}else if(memcmp(input, "setconfig", strlen("setconfig")) == 0){
		uint8_t n_configs = 3;
		uint16_t frequency = 100;
		TextOut("Setting the preset configuration.\n\r");
		struct OutputConfiguration config[n_configs];
		config[0].dtype = XDI_PacketCounter;
		config[0].freq =  frequency;
		config[1].dtype = XDI_FreeAcceleration;
		config[1].freq =  frequency;
		config[2].dtype = XDI_EulerAngles;
		config[2].freq =  frequency;
		struct XbusMessage mess;
		mess.mid = XMID_SetOutputConfiguration;
		mess.length = n_configs;
		mess.data = &config;
		uint16_t* mdptr = mess.data;
		uprintf( "[%x %x] [%x %x] [%x %x]\n\r", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);
		MT_SendXbusMessage(mess);
		MT_ReadNewMessage(0);
	}else if(strcmp(input, "reqconfig") == 0){
		TextOut("requesting output configuration mode\n\r");
		struct XbusMessage mess = { .mid = XMID_ReqOutputConfiguration,
									.data = NULL,
									.length = 0};
		MT_SendXbusMessage(mess);
		MT_ReadNewMessage(0);
	}else if(!strcmp(input, "receive")){
		TextOut("receiving a message in interrupt mode\n\r");
		MT_ReadNewMessage(0);
	}else if(!strcmp(input, "reset")){
		uprintf("resetting the MTi.\n\r");
		MT_CancelOperation();
	}

	else if(!strcmp(input, "address")){
		uprintf("address = [%d]\n\r", ReadAddress());
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
	}else if(!strcmp(input, "encoder")){
		print_encoder = !print_encoder;
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

void dribbler_SetSpeed(uint8_t percentage){
	if(percentage > 100){
		percentage = 100;
	}
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (100 - percentage) * MAX_PWM);
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart3.Instance){

	}else if(huart->Instance == huartMT.Instance){
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if(htim->Instance == htim6.Instance){
		geneva_Control();
	}else if(htim->Instance == htim7.Instance){
		DO_Control();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == SPI1_IRQ_Pin){
		//wireless message received
	}else if(GPIO_Pin == Geneva_cal_sens_Pin){
		// calibration  of the geneva drive finished
		geneva_SensorCallback();
	}
}

void MT_HandleMessage(struct XbusMessage* RX_message){
	if(RX_message->mid == XMID_Error){
		MTiErrorHandler(RX_message);
	}else if(RX_message->mid == XMID_MTData2){
		printMessageData(RX_message);
	}else if(RX_message->mid == XMID_ReqOutputConfigurationAck){
		MTiPrintOutputConfig(RX_message);
	}
}
void MTiPrintOutputConfig(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("MTiPrintOutputConfig:\n\r");

	uint8_t* rawptr = message->data;
	uint16_t fptr[message->length];
	uprintf( "len = [%u]", message->length);

	for(uint16_t * i = fptr; i < fptr + (message->length)/2; i++){
		rawptr = XbusUtility_readU16(i, rawptr);
		uprintf( "[%04x]", *i);

	}
	uprintf( "\n\r");


	uint16_t freq;
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Temperature, message))){
		uprintf("XDI_Temperature:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_UtcTime, message))){
		uprintf("XDI_UtcTime:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_PacketCounter, message))){
		uprintf("XDI_PacketCounter:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeFine, message))){
		uprintf("XDI_SampleTimeFine:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeCoarse, message))){
		uprintf("XDI_SampleTimeCoarse:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Quaternion, message))){
		uprintf("XDI_Quaternion:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RotationMatrix, message))){
		uprintf("XDI_RotationMatrix:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_EulerAngles, message))){
		uprintf("XDI_EulerAngles:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaV, message))){
		uprintf("XDI_DeltaV:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Acceleration, message))){
		uprintf("XDI_Acceleration:%x\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_FreeAcceleration, message))){
			uprintf("XDI_FreeAcceleration:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_AccelerationHR, message))){
		uprintf("XDI_AccelerationHR:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurn, message))){
		uprintf("XDI_RateOfTurn:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaQ, message))){
		uprintf("XDI_DeltaQ:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurnHR, message))){
		uprintf("XDI_RateOfTurnHR:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_MagneticField, message))){
		uprintf("XDI_MagneticField:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusByte, message))){
		uprintf("XDI_StatusByte:%u\n\r", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusWord, message))){
		uprintf("XDI_StatusWord:%u\n\r", freq);

	}
}
void MTiErrorHandler(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("ERROR: %02x\n\r", *(uint8_t *)(message->data));

}

void printMessageData(struct XbusMessage const* message){
	int bytes = message->length;
	if (!message)
		return;
	uprintf("MTData2:");

	uint16_t counter;
	if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message)){
		uprintf( " Packet counter: %5d", counter);
		bytes -= 2 + 2;
	}
	uint32_t SampleTimeFine;
	if (XbusMessage_getDataItem(&SampleTimeFine, XDI_SampleTimeFine, message)){
		uprintf( " SampleTimeFine: %lu", SampleTimeFine);
		bytes -= 2 + 4;
	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message)){
		uprintf( " Orientation: (%.3f, %.3f, %.3f, %.3f)", ori[0], ori[1],
				ori[2], ori[3]);
		bytes -= 4 * 4 + 2;

	}
	float angles[3];
	if (XbusMessage_getDataItem(angles, XDI_EulerAngles, message)){
		uprintf( " EulerAngles: (%.3f, %.3f, %.3f)", angles[0], angles[1], angles[2]);
		bytes -= 3 * 4 + 2;

	}
	float delta_v[3];
	if (XbusMessage_getDataItem(delta_v, XDI_DeltaV, message)){
		uprintf( " deltaV: (%.3f, %.3f, %.3f)", delta_v[0], delta_v[1], delta_v[2]);
		bytes -= 3 * 4 + 2;
	}
	float acc[3];
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message)){
		uprintf( " Acceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);
		bytes -= 2 * 4 + 2;
	}
	if (XbusMessage_getDataItem(acc, XDI_FreeAcceleration, message)){
		uprintf( " FreeAcceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);
		bytes -= 3 * 4 + 2;
	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message)){
		uprintf( " Rate Of Turn: (%.3f, %.3f, %.3f)", gyr[0], gyr[1], gyr[2]);
		bytes -= 3 * 4 + 2;
	}
	float delta_q[4];
	if (XbusMessage_getDataItem(delta_q, XDI_Quaternion, message)){
		uprintf( " deltaQ: (%.3f, %.3f, %.3f, %.3f)", delta_q[0], delta_q[1],
				delta_q[2], delta_q[3]);
		bytes -= 4 * 4 + 2;
	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message)){
		uprintf( " Magnetic Field: (%.3f, %.3f, %.3f)", mag[0], mag[1], mag[2]);
		bytes -= 3 * 4 + 2;
	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message)){
		uprintf( " Status:%lX", status);
		bytes -= 4 + 2;
	}
	uprintf(" [%i] bytes unread\n\r", bytes);
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
