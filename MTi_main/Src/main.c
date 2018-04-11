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
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "PuttyInterface/PuttyInterface.h"
#include "MTiControl.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef puttystruct;

#define DEBUG 					0
#define MAX_RAW_MESSAGE_SIZE 	2055

/* Buffer used for reception */

uint8_t stop_after_message_complete = 1;
uint8_t message_handled_flag = 0;
float speed_x, speed_y;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MTiPrintOutputConfig(struct XbusMessage const* message);
void MTiErrorHandler(struct XbusMessage const* message);
void printMessageData(struct XbusMessage const* message);
void PrintUsartStatus(HAL_StatusTypeDef status);
void HandleCommand(char * input);
void SendPossibleCommands();
void HandleMessage();
float calculateSpeed(float acc_x, float acc_y, float acc_z, int freq);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	MTi_Init();
	speed_x = 0;
	speed_y = 0;

	puttystruct.handle = HandleCommand;
	PuttyInterface_Init(&puttystruct);
	SendPossibleCommands();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		if(cplt_mess_stored_flag){
			cplt_mess_stored_flag = 0;
			if(DEBUG) TextOut("cplt_mess_stored_flag\n\r");
			HandleMessage();
			if(!stop_after_message_complete){
				message_handled_flag = 0;
				ReadNewMessage(0);
			}
		}

		CheckWhatNeedsToBeDone();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
// Callback is called when the HAL_Uart application is finished transmitting its bytes
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart1.Instance){

	}else if(huart->Instance == huart2.Instance){
		MT_TxCallback();
	}
}
// Callback is called when the HAL_Uart received its wanted amount of bytes
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart1.Instance){
		puttystruct.huart2_Rx_len = 1;
		puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	}else if(huart->Instance == huart2.Instance){
		MT_RxCallback();
	}
}

void HandleCommand(char * input){
	if(strcmp(input, "start") == 0){
		TextOut("Starting device MTi\n\r");
		HAL_GPIO_WritePin(MTi_nRSt_GPIO_Port, MTi_nRSt_Pin, 1);
		if(WaitForAck(XMID_WakeUp)){
			SendWakeUpAck();
			TextOut("Communication with MTi started, in config state.\n\r");
		}else{
			TextOut("No communication with MTi!\n\r");
		}
	}
	if(strcmp(input, "start2") == 0){
		TextOut("Starting device MTi\n\r");
		HAL_GPIO_WritePin(MTi_nRSt_GPIO_Port, MTi_nRSt_Pin, 1);
		if(WaitForAck(XMID_WakeUp)){
			TextOut("Communication with MTi started, going to measure state in .5 seconds.\n\r");
		}else{
			TextOut("No communication with MTi!\n\r");
		}
	}else if(strcmp(input, "config") == 0){
		struct XbusMessage mess = { .mid = XMID_GoToConfig,
									.data = NULL,
									.length = 0};

		uint8_t cnt = 0;
		while(!WaitForAck(XMID_GoToConfigAck) && cnt < 20 ){
			SendXbusMessage(mess);
			cnt++;
		}
		if(cnt < 20){
			TextOut("In config state.\n\r");
		}else{
			TextOut("No GoToConfigAck received.\n\r");
		}
	}else if(strcmp(input, "measure") == 0){
		struct XbusMessage mess = { .mid = XMID_GoToMeasurement,
									.data = NULL,
									.length = 0};
		SendXbusMessage(mess);
		if(WaitForAck(XMID_GoToMeasurementAck)){
			TextOut("In measurement state.\n\r");
		}else{
			TextOut("No GoToMeasurementAck received.\n\r");
		}

	}else if(strcmp(input, "reqdata") == 0){
		TextOut("Sending request data message.\n\r");
		struct XbusMessage mess = { .mid = XMID_ReqData,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
		ReadNewMessage(0);
	}else if(strcmp(input, "help") == 0){
		SendPossibleCommands();
	}else if(strcmp(input, "factoryreset\n") == 0){
		TextOut("Resetting the configuration.\n\r");
		struct XbusMessage mess = { .mid = XMID_RestoreFactoryDef,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
	}else if(strcmp(input, "selftest") == 0){
		TextOut("Running self test of MTi.\n\r");
		struct XbusMessage mess = { .mid = XMID_RunSelftest,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
	}else if(strcmp(input, "setoutputperiod") == 0){
		TextOut("setting outputconfig.\n\r");
		uint16_t data = 0x012C;// 300 Hz
		struct XbusMessage mess = { .mid = XMID_SetPeriod,
									.data = &data,
									.length = 0};
		SendXbusMessage(mess);
	}else if(memcmp(input, "enterdata", strlen("enterdata")) == 0){
		TextOut("Creating a message manually.\n\r");
		struct XbusMessage mess;
		mess.mid = XMID_SetOutputConfiguration;

		uint8_t data[64];
		uint8_t * dptr = data;
		int holder[2];
		sscanf(input, "enterdata%04x%04x", &holder[0], &holder[1] );
		dptr = XbusUtility_writeU16(dptr, holder[0]);
		dptr = XbusUtility_writeU16(dptr, holder[1]);
		mess.data = data;
		if(DEBUG){
			uprintf( "\n[%02x %02x %02x %02x]\n\n", data[0], data[1], data[2], data[3]);

		}
		SendXbusMessage(mess);
	}else if(memcmp(input, "setconfig", strlen("setconfig")) == 0){
		uint8_t n_configs = 2;
		uint16_t frequency = 60;
		TextOut("Setting the preset configuration.\n\r");
		struct OutputConfiguration config[n_configs];
		config[0].dtype = XDI_PacketCounter;
		config[0].freq =  frequency;
		config[1].dtype = XDI_Acceleration;
		config[1].freq =  frequency;
		struct XbusMessage mess;
		mess.mid = XMID_SetOutputConfiguration;
		mess.length = n_configs;
		mess.data = &config;
		uint16_t* mdptr = mess.data;
		if(DEBUG){
			uprintf( "[%x %x] [%x %x] [%x %x]\n", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);

		}
		SendXbusMessage(mess);
		ReadNewMessage(0);
	}else if(strcmp(input, "receive") == 0){
		TextOut("receiving a message in interrupt mode\n\r");
		ReadNewMessage(0);

	}else if(strcmp(input, "reqconfig\n") == 0){
		TextOut("requesting output configuration mode\n\r");
		struct XbusMessage mess = { .mid = XMID_ReqOutputConfiguration,
									.data = NULL,
									.length = 0};
		SendXbusMessage(mess);
		ReadNewMessage(0);
	}else if(strcmp(input, "reset") == 0){
		MtiReset();
	}else if(strcmp(input, "abort") == 0){
		CancelMtiOperation();
	}else if(strcmp(input, "readcontinue") == 0){
		TextOut("Reading continuously till readstop command\n\r");
		stop_after_message_complete = 0;
		ReadNewMessage(0);
	}else if(strcmp(input, "readstop") == 0){
		stop_after_message_complete = 1;
	}else if(strcmp(input, "temp\n") == 0){
		TextOut("temporary message holder.");
		uint8_t data[] = {0x40, 0x20,0x01, 0x80, 0x80, 0x20,0x01, 0x80};
		struct XbusMessage mess = {
				.mid = XMID_SetOutputConfiguration,
				.data = data,
				.length = sizeof(data)};
		SendXbusMessage(mess);
	}
}
// inform the user about possible commands (probably not up to date)
void SendPossibleCommands(){
	TextOut("--------------------------------------------------------------------------------\n\r");
	TextOut("These are possible input commands:\n\r");
	TextOut("\n\r");
	TextOut("'help'   			: Get this information again\n\r");
	TextOut("'start'  			: Get the device out of reset mode and into configuration mode.\n\r");
	TextOut("'start2' 			: Get the device out of reset mode and into measure mode.\n\r");
	TextOut("'config' 			: Go to configuration state.\n\r");
	TextOut("'measure'			: Go to Measurement state.\n\r");
	TextOut("'receive'			: start receiving.\n\r");
	TextOut("'readcontinue'		: Gather data continuously.\n\r");
	TextOut("'readstop'			: Stop gathering data continuously.\n\r");
	TextOut("'reqconfig'		: Request the current config.\n\r");
	TextOut("'enterdata' 		: T. B. A.\n\r");
	TextOut("'setoutputconfig'	: Set the output config to the standard.\n\r");
	TextOut("'setoutputperiod'	: [deprecated]Set Output Period to 300 Hz.\n\r");
	TextOut("'factoryreset'  	: Reset the factory settings.\n\r");
	TextOut("'selftest'  		: Test how the MTi thinks it's doing.\n\r");
	TextOut("'reset'  			: Restart the device.\n\r");
	TextOut("'abort'  			: Abort all communication with the MTi device and restart it.\n\r");
	TextOut("--------------------------------------------------------------------------------\n\r");
}

void HandleMessage(){
	if(ReceivedMessageStorage->mid == XMID_Error){
		MTiErrorHandler(ReceivedMessageStorage);
	}else if(ReceivedMessageStorage->mid == XMID_MTData2){
		printMessageData(ReceivedMessageStorage);
	}else if(ReceivedMessageStorage->mid == XMID_ReqOutputConfigurationAck){
		MTiPrintOutputConfig(ReceivedMessageStorage);
	}
	message_handled_flag = 1;
	DeallocateMem();
}
void MTiPrintOutputConfig(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("MTiPrintOutputConfig:\n");

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
		uprintf("XDI_Temperature:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_UtcTime, message))){
		uprintf("XDI_UtcTime:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_PacketCounter, message))){
		uprintf("XDI_PacketCounter:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeFine, message))){
		uprintf("XDI_SampleTimeFine:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_SampleTimeCoarse, message))){
		uprintf("XDI_SampleTimeCoarse:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Quaternion, message))){
		uprintf("XDI_Quaternion:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RotationMatrix, message))){
		uprintf("XDI_RotationMatrix:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_EulerAngles, message))){
		uprintf("XDI_EulerAngles:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaV, message))){
		uprintf("XDI_DeltaV:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_Acceleration, message))){
		uprintf("XDI_Acceleration:%x\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_FreeAcceleration, message))){
			uprintf("XDI_FreeAcceleration:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_AccelerationHR, message))){
		uprintf("XDI_AccelerationHR:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurn, message))){
		uprintf("XDI_RateOfTurn:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_DeltaQ, message))){
		uprintf("XDI_DeltaQ:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_RateOfTurnHR, message))){
		uprintf("XDI_RateOfTurnHR:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_MagneticField, message))){
		uprintf("XDI_MagneticField:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusByte, message))){
		uprintf("XDI_StatusByte:%u\n", freq);

	}
	if(0 != (freq = XbusMessage_getOutputFreq(XDI_StatusWord, message))){
		uprintf("XDI_StatusWord:%u\n", freq);

	}
}
void MTiErrorHandler(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("ERROR: %02x\n", *(uint8_t *)(message->data));

}

void printMessageData(struct XbusMessage const* message){
	if (!message)
		return;
	uprintf("MTData2:");

	uint16_t counter;
	if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message))
	{
		uprintf( " Packet counter: %5d", counter);

	}
	uint32_t SampleTimeFine;
	if (XbusMessage_getDataItem(&SampleTimeFine, XDI_SampleTimeFine, message))
	{
		uprintf( " SampleTimeFine: %lu", SampleTimeFine);

	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
	{
		uprintf( " Orientation: (%.3f, %.3f, %.3f, %.3f)", ori[0], ori[1],
				ori[2], ori[3]);

	}
	float angles[3];
	if (XbusMessage_getDataItem(angles, XDI_EulerAngles, message))
	{
		uprintf( " EulerAngles: (%.3f, %.3f, %.3f)", angles[0], angles[1], angles[2]);

	}
	float delta_v[3];
	if (XbusMessage_getDataItem(delta_v, XDI_DeltaV, message))
	{
		uprintf( " deltaV: (%.3f, %.3f, %.3f)", delta_v[0], delta_v[1], delta_v[2]);

	}
	float acc[3];
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
	{
		uprintf( " Acceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);

		uprintf( "current speed = [%f].\n", calculateSpeed(acc[0], acc[1],acc[2], 100));

	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
	{
		uprintf( " Rate Of Turn: (%.3f, %.3f, %.3f)", gyr[0], gyr[1], gyr[2]);

	}
	float delta_q[4];
	if (XbusMessage_getDataItem(delta_q, XDI_Quaternion, message))
	{
		uprintf( " deltaQ: (%.3f, %.3f, %.3f, %.3f)", delta_q[0], delta_q[1],
				delta_q[2], delta_q[3]);

	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
	{
		uprintf( " Magnetic Field: (%.3f, %.3f, %.3f)", mag[0], mag[1], mag[2]);

	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message))
	{
		uprintf( " Status:%lX", status);

	}
	TextOut(" \n\r");
}

void PrintUsartStatus(HAL_StatusTypeDef status){
	if(DEBUG){
		switch(status){
			case HAL_OK:// no problem
				//TextOut("HAL_OK\n\r");
				break;
			case HAL_BUSY:
				TextOut("HAL_BUSY\n\r");
				break;
			case HAL_ERROR:
				TextOut("HAL_ERROR\n\r");
				break;
			case HAL_TIMEOUT:
				TextOut("HAL_TIMEOUT\n\r");
				break;
			default:
				TextOut("Unknown HAL_StatusTypeDef\n\r");
				break;
		}
	}
}

float calculateSpeed(float acc_x, float acc_y, float acc_z, int freq)
{
    float dt = 1/freq;
    speed_x += acc_x + dt;
    speed_y += acc_y + dt;
    float speed_total = sqrt(speed_x*speed_x + speed_y*speed_y);
    return speed_total;
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
