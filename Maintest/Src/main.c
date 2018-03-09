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
#include "PuttyInterface/PuttyInterface.h"
#include "motorscomm/motorscomm.h"
#include "ID/ReadId.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef puttystruct;
motorscomm_HandleTypeDef motorscommstruct = {
		.huart = &huart3
};

enum motorscomm_states{
	motorscomm_Initialize,
	motorscomm_StartTransmission,
	motorscomm_Transmitting,
	motorscomm_TransmitDone,
	motorscomm_Idle,
	motorscomm_ResponseReceived,
	motorscomm_Failed
}motorscomm_state = motorscomm_Initialize;
int TX_err_count = 0;
int RX_err_count = 0;
int RX_count = 0;
int TX_count = 0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  puttystruct.handle = HandleCommand;
  PuttyInterface_Init(&puttystruct);

  motorscommstruct.TX_message.id[0] = motorscomm_LF;
  motorscommstruct.TX_message.id[1] = motorscomm_LB;
  motorscommstruct.TX_message.id[2] = motorscomm_RB;
  motorscommstruct.TX_message.id[3] = motorscomm_RF;
  //HAL_Delay(100);
  // enable UART transmission timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint led_timer = 0;
  HAL_StatusTypeDef error_code;
  while (1)
  {
	  switch(motorscomm_state){
	  case motorscomm_Initialize:
		  uprintf("motorscomm_Initialize\n\r");
		  motorscomm_state = motorscomm_Idle;
		  HAL_TIM_Base_Start_IT(&htim6);
		  break;
	  case motorscomm_StartTransmission:
		  error_code = motorscomm_UART_StartTransmit(&motorscommstruct, 0);
		  if(error_code == HAL_OK){
			  // transmission successful
			  motorscomm_state = motorscomm_Transmitting;
			  //uprintf("motorscomm_state = motorscomm_Transmitting...\n\r");
		  }
		  else{
			  // transmission failed, trying again next while loop cycle
			  uprintf("Transmit failed error code:[%d]\n\r", error_code);
			  motorscomm_state = motorscomm_Failed;
			  RX_err_count++;
		  }
		  break;
	  case motorscomm_Transmitting:
		  break;
	  case motorscomm_TransmitDone:
		  error_code = motorscomm_HAL_UART_Receive(&motorscommstruct);
		  if(error_code == HAL_OK){
			  // start read successful
			  motorscomm_state = motorscomm_Idle;
		  }else{
			  // transmission failed, trying again next while loop cycle
			  uprintf("receive failed with error code:[%d]\n\r", error_code);
			  motorscomm_state = motorscomm_Failed;
		  }
		  break;
	  case motorscomm_Idle:
		  break;
	  case motorscomm_ResponseReceived:
		  motorscomm_DecodeBuf(&motorscommstruct);
		  motorscomm_state = motorscomm_Idle;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
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


void HandleCommand(char* input){
	if(!strcmp(input, "start")){
		uprintf("started;>)\n\r");
	}else if(!strcmp(input, "stop")){
		uprintf("stopped\n\r");
	}else if(!strcmp(input, "address")){
		uprintf("address = [%d]\n\r", ReadAddress());
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart1.Instance){
		puttystruct.huart2_Rx_len = 1;
		puttystruct.small_buf[0] = *(huart->pRxBuffPtr-1);
	}else if(huart->Instance == huart3.Instance){
		if(motorscomm_state == motorscomm_Idle){
			motorscomm_state = motorscomm_ResponseReceived;
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
			motorscomm_state = motorscomm_TransmitDone;
			TX_count++;
		}else{
			uprintf("motorscomm_Failed from HAL_UART_TxCpltCallback\n\r");
			motorscomm_state = motorscomm_Failed;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6){
		if (motorscomm_Idle != motorscomm_state){
			TX_err_count++;
			uprintf("motorscomm_Failed from HAL_TIM_PeriodElapsedCallback; state = [%d]\n\r", motorscomm_state);
			motorscomm_state = motorscomm_Failed;
			return;
		}
		motorscomm_state = motorscomm_StartTransmission;
		//uprintf("motorscomm_state = motorscomm_StartTransmission\n\r");
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
