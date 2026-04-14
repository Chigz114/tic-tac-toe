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
#include "stm32f4xx_hal.h"

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
#define key_in3_Pin GPIO_PIN_7
#define key_in3_GPIO_Port GPIOF
#define key_in4_Pin GPIO_PIN_8
#define key_in4_GPIO_Port GPIOF
#define key_in2_Pin GPIO_PIN_11
#define key_in2_GPIO_Port GPIOF
#define key_out1_Pin GPIO_PIN_9
#define key_out1_GPIO_Port GPIOE
#define key_out2_Pin GPIO_PIN_11
#define key_out2_GPIO_Port GPIOE
#define key_out3_Pin GPIO_PIN_13
#define key_out3_GPIO_Port GPIOE
#define key_out4_Pin GPIO_PIN_15
#define key_out4_GPIO_Port GPIOE
#define key_in1_Pin GPIO_PIN_9
#define key_in1_GPIO_Port GPIOD
#define magnet_Pin GPIO_PIN_11
#define magnet_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
