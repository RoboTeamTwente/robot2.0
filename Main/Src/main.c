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
#include <stdlib.h>
#include <math.h>
#include "PuttyInterface/PuttyInterface.h"
#include "myNRF24.h"
#include "motorscomm/motorscomm.h"
#include "ID/ReadId.h"
<<<<<<< master
#include "myNRF24.h"
=======
#include "MTiControl/MTiControl.h"
#include <math.h>
>>>>>>> did some changes to the ioc
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef puttystruct;
motorscomm_HandleTypeDef motorscommstruct = {
		/*.huart = &huart3*/
};

uint8_t address = 1;
uint8_t freqChannel = 78;

enum motorscomm_states{
	motorscomm_Initialize,
	motorscomm_Transmitting,
	motorscomm_Receiving,
	motorscomm_Failed
}motorscomm_state = motorscomm_Initialize;
int TX_err_count = 0;
int RX_err_count = 0;
int RX_count = 0;
int TX_count = 0;

#define DEBUG 					1
#define MAX_RAW_MESSAGE_SIZE 	2055

/* Buffer used for reception */

uint8_t stop_after_message_complete = 1;
uint8_t message_handled_flag = 0;
float speed_x, speed_y;
HAL_StatusTypeDef error_code;

uint64_t while_cnt = 0;
uint64_t while_cnt_cnt = 0;
uint64_t while_avg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleCommand(char* input);
void MTiPrintOutputConfig(struct XbusMessage const* message);
void MTiErrorHandler(struct XbusMessage const* message);
void printMessageData(struct XbusMessage const* message);
void PrintUsartStatus(HAL_StatusTypeDef status);
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
  address = ReadAddress();

  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);

  motorscommstruct.TX_message.id[0] = motorscomm_LF;
  motorscommstruct.TX_message.id[1] = motorscomm_LB;
  motorscommstruct.TX_message.id[2] = motorscomm_RB;
  motorscommstruct.TX_message.id[3] = motorscomm_RF;
  motorscommstruct.TX_message.wheel_speed[0] = 0.1F;
  motorscommstruct.TX_message.wheel_speed[1] = 0.5F;
  motorscommstruct.TX_message.wheel_speed[2] = 1.0F;
  motorscommstruct.TX_message.wheel_speed[3] = 2.0F;
  MTi_Init();
  speed_x = 0;
  speed_y = 0;
  HAL_Delay(1000);
  initRobo(&hspi1, freqChannel, address);
  dataPacket dataStruct;
  // enable UART transmission timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  nssHigh(&hspi1);

  initRobo(&hspi1, freqChannel, address);
  dataPacket dataStruct;
  float cos_a0 = cos(_a0);
  float sin_a0 = sin(_a0);
  float cos_a1 = cos(_a1);
  float sin_a1 = sin(_a1);
  float cos_a2 = cos(_a2);
  float sin_a2 = sin(_a2);
  float cos_a3 = cos(_a3);
  float sin_a3 = sin(_a3);
  float wheelScalar = 1/_r;
  float wRadPerSec;
  float angularComponent;

  uint LastPackageTime = 0;

  uint led_timer = 0;
  while (1)
  {
//	  if(irqRead(&hspi1)){
//	  		  //roboCallback(&hspi1, &dataStruct);
//	  		  //if(dataStruct.robotID == address){
//	  			  //HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
//	  			  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	  			  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
//	  			  //kicker
//	  		  //}
//	  		  HAL_GPIO_TogglePin(LD0_GPIO_Port, LD0_Pin);
//	  	  }
	  while_cnt++;// counts how many times the while loop is passed
	  switch(motorscomm_state){
	  case motorscomm_Initialize:
		  uprintf("motorscomm_Initialize\n\r");
		  motorscomm_state = motorscomm_Receiving;
		  HAL_TIM_Base_Start_IT(&htim6);
		  break;
	  case motorscomm_Transmitting:
	  case motorscomm_Receiving:
		  if(motorscommstruct.UART2_Rx_flag){
			  motorscommstruct.UART2_Rx_flag = false;
			  motorscomm_DecodeBuf(&motorscommstruct);
		  }
		  break;
	  case motorscomm_Failed:
		  //HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 1);
		  break;
	  }

	  if(cplt_mess_stored_flag){
			cplt_mess_stored_flag = 0;
			if(DEBUG) uprintf("cplt_mess_stored_flag\n\r");
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
	  if(HAL_GetTick() > led_timer + 500 ){
		  led_timer = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
//		  uprintf("suc/err:TX[%d/%d];RX[%d/%d]\n\r", TX_count, TX_err_count, RX_count, RX_err_count);
//		  uprintf("Tx = [%f, %f, %f, %f]\n\r", motorscommstruct.TX_message.wheel_speed[0], motorscommstruct.TX_message.wheel_speed[1], motorscommstruct.TX_message.wheel_speed[2], motorscommstruct.TX_message.wheel_speed[3]);
//		  uint8_t* ptr = motorscommstruct.UART2RX_buf;
//		  uprintf("RX in hex[%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X]\n\r", *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++));
//		  ptr = motorscommstruct.UART2TX_buf;
//		  uprintf("TX in hex[%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X]\n\r", *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++), *(ptr++));
//		  uprintf("avg n_whileloops = [%lu]\n\r", while_avg);
	  }
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
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
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


void HandleCommand(char* input){
	if(!strcmp(input, "start")){
		uprintf("started;>)\n\r");
	}else if(!strcmp(input, "stop")){
		uprintf("stopped\n\r");
	}else if(!strcmp(input, "address")){
		uprintf("address = [%d]\n\r", ReadAddress());
	}else if(!memcmp(input, "control", 7)){
		float duty = atof(input+ 8);
		for(uint i = 0; i < 4; i++){
			motorscommstruct.TX_message.wheel_speed[i] = duty;
		}
		uprintf("speed set to = [%f]\n\r", motorscommstruct.TX_message.wheel_speed[0]);
	}else if(strcmp(input, "start2") == 0){
		uprintf("Starting device MTi\n\r");
		//HAL_GPIO_WritePin(GPIOB, MT_RST_Pin, 1);
		if(WaitForAck(XMID_WakeUp)){
			uprintf("Communication with MTi started, going to measure state in .5 seconds.\n\r");
		}else{
			uprintf("No communication with MTi!\n\r");
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
			uprintf("In config state.\n\r");
		}else{
			uprintf("No GoToConfigAck received.\n\r");
		}
	}else if(strcmp(input, "measure") == 0){
		struct XbusMessage mess = { .mid = XMID_GoToMeasurement,
									.data = NULL,
									.length = 0};
		SendXbusMessage(mess);
		if(WaitForAck(XMID_GoToMeasurementAck)){
			uprintf("In measurement state.\n\r");
		}else{
			uprintf("No GoToMeasurementAck received.\n\r");
		}

	}else if(strcmp(input, "reqdata") == 0){
		uprintf("Sending request data message.\n\r");
		struct XbusMessage mess = { .mid = XMID_ReqData,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
		ReadNewMessage(0);
	}else if(strcmp(input, "factoryreset") == 0){
		uprintf("Resetting the configuration.\n\r");
		struct XbusMessage mess = { .mid = XMID_RestoreFactoryDef,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
	}else if(strcmp(input, "selftest") == 0){
		uprintf("Running self test of MTi.\n\r");
		struct XbusMessage mess = { .mid = XMID_RunSelftest,
									.length = 0,
									.data = NULL};
		SendXbusMessage(mess);
	}else if(strcmp(input, "setoutputperiod") == 0){
		uprintf("setting outputconfig.\n\r");
		uint16_t data = 0x012C;// 300 Hz
		struct XbusMessage mess = { .mid = XMID_SetPeriod,
									.data = &data,
									.length = 0};
		SendXbusMessage(mess);
	}else if(memcmp(input, "enterdata", strlen("enterdata")) == 0){
		uprintf("Creating a message manually.\n\r");
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
			uprintf(smallStrBuffer, "\n[%02x %02x %02x %02x]\n\n", data[0], data[1], data[2], data[3]);

		}
		SendXbusMessage(mess);
	}else if(memcmp(input, "setconfig", strlen("setconfig")) == 0){
		uint8_t n_configs = 2;
		uint16_t frequency = 60;
		uprintf("Setting the preset configuration.\n\r");
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
			uprintf("[%x %x] [%x %x] [%x %x]\n", *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++, *mdptr++);

		}
		SendXbusMessage(mess);
		ReadNewMessage(0);
	}else if(strcmp(input, "receive") == 0){
		uprintf("receiving a message in interrupt mode\n\r");
		ReadNewMessage(0);

	}else if(strcmp(input, "reqconfig") == 0){
		uprintf("requesting output configuration mode\n\r");
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
		uprintf("Reading continuously till readstop command\n\r");
		stop_after_message_complete = 0;
		ReadNewMessage(0);
	}else if(strcmp(input, "readstop") == 0){
		stop_after_message_complete = 1;
	}else if(strcmp(input, "temp") == 0){
		uprintf("temporary message holder.");
		uint8_t data[] = {0x40, 0x20,0x01, 0x80, 0x80, 0x20,0x01, 0x80};
		struct XbusMessage mess = {
				.mid = XMID_SetOutputConfiguration,
				.data = data,
				.length = sizeof(data)};
		SendXbusMessage(mess);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart1.Instance){
		puttystruct.huart2_Rx_len = 1;
		puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	/*}else if(huart->Instance == huart3.Instance){
		if(motorscomm_state == motorscomm_Receiving){
			motorscommstruct.UART2_Rx_flag = true;
			RX_count++;
		}else{
			uprintf("motorscomm_Failed from HAL_UART_RxCpltCallback\n\r");
			motorscomm_state = motorscomm_Failed;
		}*/
	}else if(huart->Instance == huart2.Instance){
		MT_HAL_UART_RxCpltCallback();
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(/*huart->Instance == huart3.Instance){
		if(motorscomm_Transmitting == motorscomm_state){
			error_code = motorscomm_HAL_UART_Receive(&motorscommstruct);
			if(error_code == HAL_OK){
			  // start read successful
			  motorscomm_state = motorscomm_Receiving;
			}else{
			  // transmission failed, trying again next while loop cycle
			  uprintf("receive failed with error code:[%d]\n\r", error_code);
			  motorscomm_state = motorscomm_Failed;
			}
			TX_count++;
		}else{
			uprintf("motorscomm_Failed from HAL_UART_TxCpltCallback\n\r");
			motorscomm_state = motorscomm_Failed;
		}
	}else if(*/huart->Instance == huart2.Instance){
		MT_HAL_UART_TxCpltCallback();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6){
		while_cnt_cnt++;
		while_avg = ((while_avg * while_cnt_cnt - while_avg) + while_cnt)/while_cnt_cnt;
		while_cnt = 0;
//		if (motorscomm_Receiving != motorscomm_state){
//			TX_err_count++;
//			uprintf("motorscomm_Failed from HAL_TIM_PeriodElapsedCallback; state = [%d]\n\r", motorscomm_state);
//			motorscomm_state = motorscomm_Failed;
//			return;
//		}
//		error_code = motorscomm_UART_StartTransmit(&motorscommstruct, 0);
//		if(error_code == HAL_OK){
//		  // transmission successful
//		  motorscomm_state = motorscomm_Transmitting;
//		}else{
//		  // transmission failed, trying again next while loop cycle
//		  uprintf("Transmit failed error code:[%d]\n\r", error_code);
//		  motorscomm_state = motorscomm_Failed;
//		  RX_err_count++;
//		}
	}
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
	uprintf("len = [%u]", message->length);

	for(uint16_t * i = fptr; i < fptr + (message->length)/2; i++){
		rawptr = XbusUtility_readU16(i, rawptr);
		uprintf("[%04x]", *i);

	}
	uprintf("\n\r");


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
		uprintf(" Packet counter: %5d", counter);

	}
	uint32_t SampleTimeFine;
	if (XbusMessage_getDataItem(&SampleTimeFine, XDI_SampleTimeFine, message))
	{
		uprintf(" SampleTimeFine: %lu", SampleTimeFine);

	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
	{
		uprintf(" Orientation: (%.3f, %.3f, %.3f, %.3f)", ori[0], ori[1],
				ori[2], ori[3]);

	}
	float angles[3];
	if (XbusMessage_getDataItem(angles, XDI_EulerAngles, message))
	{
		uprintf(" EulerAngles: (%.3f, %.3f, %.3f)", angles[0], angles[1], angles[2]);

	}
	float delta_v[3];
	if (XbusMessage_getDataItem(delta_v, XDI_DeltaV, message))
	{
		uprintf(" deltaV: (%.3f, %.3f, %.3f)", delta_v[0], delta_v[1], delta_v[2]);

	}
	float acc[3];
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
	{
		uprintf(" Acceleration: (%.3f, %.3f, %.3f)", acc[0], acc[1], acc[2]);

		uprintf("current speed = [%f].\n", calculateSpeed(acc[0], acc[1],acc[2], 100));

	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
	{
		uprintf(" Rate Of Turn: (%.3f %.3f %.3f)", gyr[0], gyr[1], gyr[2]);

	}
	float delta_q[4];
	if (XbusMessage_getDataItem(delta_q, XDI_Quaternion, message))
	{
		uprintf(" deltaQ: (%.3f, %.3f, %.3f, %.3f)", delta_q[0], delta_q[1],
				delta_q[2], delta_q[3]);

	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
	{
		uprintf(" Magnetic Field: (%.3f, %.3f, %.3f)", mag[0], mag[1], mag[2]);

	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message))
	{
		uprintf(" Status:%lX", status);

	}
	uprintf(" \n\r");
}

void PrintUsartStatus(HAL_StatusTypeDef status){
	if(DEBUG){
		switch(status){
			case HAL_OK:// no problem
				//uprintf("HAL_OK\n\r");
				break;
			case HAL_BUSY:
				uprintf("HAL_BUSY\n\r");
				break;
			case HAL_ERROR:
				uprintf("HAL_ERROR\n\r");
				break;
			case HAL_TIMEOUT:
				uprintf("HAL_TIMEOUT\n\r");
				break;
			default:
				uprintf("Unknown HAL_StatusTypeDef\n\r");
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
