/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GRAY_AD0_Pin GPIO_PIN_0
#define GRAY_AD0_GPIO_Port GPIOC
#define GRAY_AD1_Pin GPIO_PIN_1
#define GRAY_AD1_GPIO_Port GPIOC
#define GRAY_AD2_Pin GPIO_PIN_2
#define GRAY_AD2_GPIO_Port GPIOC
#define GRAY_OUT_Pin GPIO_PIN_3
#define GRAY_OUT_GPIO_Port GPIOC
#define CAR_PWM_LEFT_Pin GPIO_PIN_6
#define CAR_PWM_LEFT_GPIO_Port GPIOA
#define CAR_PWM_RIGHT_Pin GPIO_PIN_7
#define CAR_PWM_RIGHT_GPIO_Port GPIOA
#define ENC_LEFT_A_Pin GPIO_PIN_8
#define ENC_LEFT_A_GPIO_Port GPIOA
#define ENC_LEFT_B_Pin GPIO_PIN_9
#define ENC_LEFT_B_GPIO_Port GPIOA
#define CAR_LEFT_AIN1_Pin GPIO_PIN_0
#define CAR_LEFT_AIN1_GPIO_Port GPIOB
#define CAR_LEFT_AIN2_Pin GPIO_PIN_1
#define CAR_LEFT_AIN2_GPIO_Port GPIOB
#define CAR_RIGHT_BIN1_Pin GPIO_PIN_2
#define CAR_RIGHT_BIN1_GPIO_Port GPIOB
#define ENC_RIGHT_A_Pin GPIO_PIN_6
#define ENC_RIGHT_A_GPIO_Port GPIOC
#define ENC_RIGHT_B_Pin GPIO_PIN_7
#define ENC_RIGHT_B_GPIO_Port GPIOC
#define CAR_RIGHT_BIN2_Pin GPIO_PIN_10
#define CAR_RIGHT_BIN2_GPIO_Port GPIOB
#define CAR_LED_Pin GPIO_PIN_5
#define CAR_LED_GPIO_Port GPIOB
#define CAR_BEEP_Pin GPIO_PIN_6
#define CAR_BEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
