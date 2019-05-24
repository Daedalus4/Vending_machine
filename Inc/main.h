/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern UART_HandleTypeDef huart3;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _24V_EN_Pin GPIO_PIN_13
#define _24V_EN_GPIO_Port GPIOC
#define MOTO1_SENSOR_Pin GPIO_PIN_14
#define MOTO1_SENSOR_GPIO_Port GPIOC
#define MOTO1_Pin GPIO_PIN_15
#define MOTO1_GPIO_Port GPIOC
#define MOTO2_SENSOR_Pin GPIO_PIN_0
#define MOTO2_SENSOR_GPIO_Port GPIOD
#define MOTO2_Pin GPIO_PIN_1
#define MOTO2_GPIO_Port GPIOD
#define MOTO3_SENSOR_Pin GPIO_PIN_0
#define MOTO3_SENSOR_GPIO_Port GPIOC
#define MOTO3_Pin GPIO_PIN_1
#define MOTO3_GPIO_Port GPIOC
#define E_LOCK4_Pin GPIO_PIN_2
#define E_LOCK4_GPIO_Port GPIOC
#define E_LOCK3_Pin GPIO_PIN_3
#define E_LOCK3_GPIO_Port GPIOC
#define V_BAT_Pin GPIO_PIN_0
#define V_BAT_GPIO_Port GPIOA
#define E_LOCK2_Pin GPIO_PIN_1
#define E_LOCK2_GPIO_Port GPIOA
#define E_LOCK1_Pin GPIO_PIN_4
#define E_LOCK1_GPIO_Port GPIOA
#define DOWM_DE_1_IN_Pin GPIO_PIN_5
#define DOWM_DE_1_IN_GPIO_Port GPIOA
#define DOWM_DE_2_IN_Pin GPIO_PIN_6
#define DOWM_DE_2_IN_GPIO_Port GPIOA
#define LAST_DE_1_IN_Pin GPIO_PIN_7
#define LAST_DE_1_IN_GPIO_Port GPIOA
#define LAST_DE_2_IN_Pin GPIO_PIN_4
#define LAST_DE_2_IN_GPIO_Port GPIOC
#define LAST_DE_3_IN_Pin GPIO_PIN_5
#define LAST_DE_3_IN_GPIO_Port GPIOC
#define DORE_DE_IN_Pin GPIO_PIN_0
#define DORE_DE_IN_GPIO_Port GPIOB
#define CTRL_AOLA_Pin GPIO_PIN_1
#define CTRL_AOLA_GPIO_Port GPIOB
#define CTRL_LIGHT_Pin GPIO_PIN_2
#define CTRL_LIGHT_GPIO_Port GPIOB
#define CTRL_AO_DOOR_DE_Pin GPIO_PIN_12
#define CTRL_AO_DOOR_DE_GPIO_Port GPIOB
#define STEP_MOTO1_A_Pin GPIO_PIN_13
#define STEP_MOTO1_A_GPIO_Port GPIOB
#define STEP_MOTO1_B_Pin GPIO_PIN_14
#define STEP_MOTO1_B_GPIO_Port GPIOB
#define STEP_MOTO1_C_Pin GPIO_PIN_15
#define STEP_MOTO1_C_GPIO_Port GPIOB
#define STEP_MOTO1_D_Pin GPIO_PIN_6
#define STEP_MOTO1_D_GPIO_Port GPIOC
#define SEC_DOOR_DE_Pin GPIO_PIN_7
#define SEC_DOOR_DE_GPIO_Port GPIOC
#define SEC_LOCK_DE_Pin GPIO_PIN_8
#define SEC_LOCK_DE_GPIO_Port GPIOC
#define SPK_SHUTDOWN_Pin GPIO_PIN_11
#define SPK_SHUTDOWN_GPIO_Port GPIOA
#define _2G_WAKE_UP_Pin GPIO_PIN_12
#define _2G_WAKE_UP_GPIO_Port GPIOA
#define _2G_PWR_ON_Pin GPIO_PIN_15
#define _2G_PWR_ON_GPIO_Port GPIOA
#define _2G_LPG_Pin GPIO_PIN_3
#define _2G_LPG_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOB
#define RGB_R_Pin GPIO_PIN_5
#define RGB_R_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_6
#define RGB_B_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_7
#define RGB_G_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define KEY1_Pin GPIO_PIN_10
//#define KEY1_GPIO_Port GPIOB
//#define KEY2_Pin GPIO_PIN_8
//#define KEY2_GPIO_Port GPIOA
#define MOTO1_A(x) HAL_GPIO_WritePin(STEP_MOTO1_A_GPIO_Port, STEP_MOTO1_A_Pin, x)
#define MOTO1_B(x) HAL_GPIO_WritePin(STEP_MOTO1_B_GPIO_Port, STEP_MOTO1_B_Pin, x)
#define MOTO1_C(x) HAL_GPIO_WritePin(STEP_MOTO1_C_GPIO_Port, STEP_MOTO1_C_Pin, x)
#define MOTO1_D(x) HAL_GPIO_WritePin(STEP_MOTO1_D_GPIO_Port, STEP_MOTO1_D_Pin, x)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
