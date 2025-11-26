/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_1
#define CS_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_2
#define CE_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_3
#define IRQ_GPIO_Port GPIOA
#define TIM3_0_Pin GPIO_PIN_0
#define TIM3_0_GPIO_Port GPIOB
#define TIM3_1_Pin GPIO_PIN_1
#define TIM3_1_GPIO_Port GPIOB
#define TIM1_0_Pin GPIO_PIN_13
#define TIM1_0_GPIO_Port GPIOE
#define TIM1_1_Pin GPIO_PIN_14
#define TIM1_1_GPIO_Port GPIOE
#define TIM2_0_Pin GPIO_PIN_10
#define TIM2_0_GPIO_Port GPIOB
#define TIM2_1_Pin GPIO_PIN_11
#define TIM2_1_GPIO_Port GPIOB
#define TIM4_0_Pin GPIO_PIN_14
#define TIM4_0_GPIO_Port GPIOD
#define TIM4_1_Pin GPIO_PIN_15
#define TIM4_1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
