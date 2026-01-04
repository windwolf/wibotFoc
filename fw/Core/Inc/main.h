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
#define O_LIGHT_Pin GPIO_PIN_13
#define O_LIGHT_GPIO_Port GPIOC
#define ADC2_IN4_VIN_Pin GPIO_PIN_7
#define ADC2_IN4_VIN_GPIO_Port GPIOA
#define ADC2_IN5_VBAT_Pin GPIO_PIN_4
#define ADC2_IN5_VBAT_GPIO_Port GPIOC
#define INT_BATT_Pin GPIO_PIN_6
#define INT_BATT_GPIO_Port GPIOC
#define O_PWREN_Pin GPIO_PIN_11
#define O_PWREN_GPIO_Port GPIOA
#define I_KEY_ONOFF_Pin GPIO_PIN_12
#define I_KEY_ONOFF_GPIO_Port GPIOA
#define LED_LOCK_Pin GPIO_PIN_15
#define LED_LOCK_GPIO_Port GPIOA
#define I_KEY_DEC_Pin GPIO_PIN_3
#define I_KEY_DEC_GPIO_Port GPIOB
#define I_KEY_INC_Pin GPIO_PIN_4
#define I_KEY_INC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
