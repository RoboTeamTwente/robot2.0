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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "PuttyInterface/PuttyInterface.h"
#include "motorscomm/motorscomm.h"
#include "ID/ReadId.h"
#include "myNRF24.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef puttystruct;
motorscomm_HandleTypeDef motorscommstruct = {
		.huart = &huart3
};

uint8_t address = 1;
uint8_t freqChannel = 78;

#define MAX_TIME_AFTER_LAST_MESSAGE 100

const float _a0 = 60 * 3.1415/180.0; //240
const float _a1 = 120 * 3.1415/180.0; //300
const float _a2 = 240  * 3.1415/180.0; //60
const float _a3 = 300 * 3.1415/180.0; //120

#define _R   0.09f
#define _r   0.0275f


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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
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
  HAL_Delay(1000);
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
	  while_cnt++;
	  if(irqRead(&hspi1)){
		  LastPackageTime = HAL_GetTick();
		  roboCallback(&hspi1, &dataStruct);
		  if(dataStruct.robotID == address){
			  HAL_GPIO_TogglePin(LD0_GPIO_Port,LD0_Pin);
			  float magnitude = dataStruct.robotVelocity / 1000; //from mm/s to m/s;
			  float direction = dataStruct.movingDirection * (2*M_PI/512);
			  float cosDir = cos(direction);
			  float sinDir = sin(direction);
			  float xComponent = cosDir*magnitude;
			  float yComponent = sinDir*magnitude;
			  int rotSign;

			  if(dataStruct.rotationDirection != 0){
				  rotSign = -1;
			  }else{
				  rotSign = 1;
			  }

			  wRadPerSec = (dataStruct.angularVelocity/180.0)*PI;
			  angularComponent = rotSign*_R*wRadPerSec;

			  motorscommstruct.TX_message.wheel_speed[0] = ((-cos_a0*yComponent * 1.4 + sin_a0*xComponent + angularComponent)*wheelScalar)/(30*8);
			  motorscommstruct.TX_message.wheel_speed[1] = ((-cos_a1*yComponent * 1.4 + sin_a1*xComponent + angularComponent)*wheelScalar)/(30*8);
			  motorscommstruct.TX_message.wheel_speed[2] = ((-cos_a2*yComponent * 1.4 + sin_a2*xComponent + angularComponent)*wheelScalar)/(30*8);
			  motorscommstruct.TX_message.wheel_speed[3] = ((-cos_a3*yComponent * 1.4 + sin_a3*xComponent + angularComponent)*wheelScalar)/(30*8);
		  }

	  }else if (HAL_GetTick()-LastPackageTime > MAX_TIME_AFTER_LAST_MESSAGE){
		  motorscommstruct.TX_message.wheel_speed[0] = 0;
		  motorscommstruct.TX_message.wheel_speed[1] = 0;
		  motorscommstruct.TX_message.wheel_speed[2] = 0;
		  motorscommstruct.TX_message.wheel_speed[3] = 0;
	  }

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
		  HAL_GPIO_WritePin(LD0_GPIO_Port, LD0_Pin, 1);
		  break;
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  PuttyInterface_Update(&puttystruct);
	  if(HAL_GetTick() > led_timer + 500 ){
		  led_timer = HAL_GetTick();
		  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
		  uprintf("suc/err:TX[%d/%d];RX[%d/%d]\n\r", TX_count, TX_err_count, RX_count, RX_err_count);
		  uprintf("Tx = [%f, %f, %f, %f]\n\r", motorscommstruct.TX_message.wheel_speed[0], motorscommstruct.TX_message.wheel_speed[1], motorscommstruct.TX_message.wheel_speed[2], motorscommstruct.TX_message.wheel_speed[3]);
		  uint8_t* ptr = motorscommstruct.UART2RX_buf;
		  uprintf("RX in hex[%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X]\n\r", *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++);
		  ptr = motorscommstruct.UART2TX_buf;
		  uprintf("TX in hex[%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X]\n\r", *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++, *ptr++);
		  uprintf("avg n_whileloops = [%ld]\n\r", while_avg);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
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
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart1.Instance){
		puttystruct.huart2_Rx_len = 1;
		puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	}else if(huart->Instance == huart3.Instance){
		if(motorscomm_state == motorscomm_Receiving){
			motorscommstruct.UART2_Rx_flag = true;
			RX_count++;
		}else{
			uprintf("motorscomm_Failed from HAL_UART_RxCpltCallback\n\r");
			motorscomm_state = motorscomm_Failed;
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart3.Instance){
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
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6){
		while_cnt_cnt++;
		while_avg = ((while_avg * while_cnt_cnt - while_avg) + while_cnt)/while_cnt_cnt;
		while_cnt = 0;
		if (motorscomm_Receiving != motorscomm_state){
			TX_err_count++;
			uprintf("motorscomm_Failed from HAL_TIM_PeriodElapsedCallback; state = [%d]\n\r", motorscomm_state);
			motorscomm_state = motorscomm_Failed;
			return;
		}
		error_code = motorscomm_UART_StartTransmit(&motorscommstruct, 0);
		if(error_code == HAL_OK){
		  // transmission successful
		  motorscomm_state = motorscomm_Transmitting;
		}else{
		  // transmission failed, trying again next while loop cycle
		  uprintf("Transmit failed error code:[%d]\n\r", error_code);
		  motorscomm_state = motorscomm_Failed;
		  RX_err_count++;
		}
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
