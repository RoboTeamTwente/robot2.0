/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Geneva_cal_sens_Pin GPIO_PIN_13
#define Geneva_cal_sens_GPIO_Port GPIOC
#define Geneva_dir_A_Pin GPIO_PIN_14
#define Geneva_dir_A_GPIO_Port GPIOC
#define Geneva_dir_B_Pin GPIO_PIN_15
#define Geneva_dir_B_GPIO_Port GPIOC
#define SPI1_IRQ_Pin GPIO_PIN_2
#define SPI1_IRQ_GPIO_Port GPIOA
#define Charge_done_Pin GPIO_PIN_4
#define Charge_done_GPIO_Port GPIOA
#define kick_Pin GPIO_PIN_0
#define kick_GPIO_Port GPIOB
#define chip_Pin GPIO_PIN_1
#define chip_GPIO_Port GPIOB
#define charge_Pin GPIO_PIN_2
#define charge_GPIO_Port GPIOB
#define MCU_TX_Pin GPIO_PIN_10
#define MCU_TX_GPIO_Port GPIOB
#define MCU_RX_Pin GPIO_PIN_11
#define MCU_RX_GPIO_Port GPIOB
#define ID0_Pin GPIO_PIN_12
#define ID0_GPIO_Port GPIOB
#define ID1_Pin GPIO_PIN_15
#define ID1_GPIO_Port GPIOB
#define Tx_PC_Pin GPIO_PIN_9
#define Tx_PC_GPIO_Port GPIOA
#define Rx_PC_Pin GPIO_PIN_10
#define Rx_PC_GPIO_Port GPIOA
#define ID2_Pin GPIO_PIN_11
#define ID2_GPIO_Port GPIOA
#define ID3_Pin GPIO_PIN_12
#define ID3_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOA
#define LD0_Pin GPIO_PIN_14
#define LD0_GPIO_Port GPIOA
#define bs_interrupt_Pin GPIO_PIN_15
#define bs_interrupt_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_8
#define SPI1_NSS_GPIO_Port GPIOB
#define Geneva_PWM_Pin GPIO_PIN_9
#define Geneva_PWM_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
