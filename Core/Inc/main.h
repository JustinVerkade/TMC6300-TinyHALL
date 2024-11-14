/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VS_ADC_Pin GPIO_PIN_0
#define VS_ADC_GPIO_Port GPIOA
#define COMMAND_TX_Pin GPIO_PIN_2
#define COMMAND_TX_GPIO_Port GPIOA
#define COMMAND_RX_Pin GPIO_PIN_3
#define COMMAND_RX_GPIO_Port GPIOA
#define BLDC_ISENSE_Pin GPIO_PIN_4
#define BLDC_ISENSE_GPIO_Port GPIOA
#define BLDC_DIAG_Pin GPIO_PIN_5
#define BLDC_DIAG_GPIO_Port GPIOA
#define BLDC_NSTDBY_Pin GPIO_PIN_6
#define BLDC_NSTDBY_GPIO_Port GPIOA
#define BLDC_WL_Pin GPIO_PIN_7
#define BLDC_WL_GPIO_Port GPIOA
#define BLDC_VL_Pin GPIO_PIN_0
#define BLDC_VL_GPIO_Port GPIOB
#define BLDC_UL_Pin GPIO_PIN_1
#define BLDC_UL_GPIO_Port GPIOB
#define BLDC_WH_Pin GPIO_PIN_8
#define BLDC_WH_GPIO_Port GPIOA
#define BLDC_VH_Pin GPIO_PIN_9
#define BLDC_VH_GPIO_Port GPIOA
#define BLDC_UH_Pin GPIO_PIN_10
#define BLDC_UH_GPIO_Port GPIOA
#define HAL_C_Pin GPIO_PIN_11
#define HAL_C_GPIO_Port GPIOA
#define HAL_C_EXTI_IRQn EXTI15_10_IRQn
#define HAL_B_Pin GPIO_PIN_12
#define HAL_B_GPIO_Port GPIOA
#define HAL_B_EXTI_IRQn EXTI15_10_IRQn
#define HAL_A_Pin GPIO_PIN_15
#define HAL_A_GPIO_Port GPIOA
#define HAL_A_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_7
#define LED0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
