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
#include "stm32g4xx_hal.h"

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
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOE
#define LED_SELF_Pin GPIO_PIN_13
#define LED_SELF_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOB
#define CIN1_Pin GPIO_PIN_10
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_11
#define CIN2_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_6
#define STBY_GPIO_Port GPIOC
#define DIP3_Pin GPIO_PIN_7
#define DIP3_GPIO_Port GPIOC
#define DIN2_Pin GPIO_PIN_12
#define DIN2_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_2
#define DIN1_GPIO_Port GPIOD
#define DIP4_Pin GPIO_PIN_3
#define DIP4_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_4
#define DIP1_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_5
#define DIP2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_9
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
