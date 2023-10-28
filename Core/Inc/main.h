/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define RF_KEY_Pin GPIO_PIN_13
#define RF_KEY_GPIO_Port GPIOC
#define DB6_Pin GPIO_PIN_14
#define DB6_GPIO_Port GPIOC
#define DB5_Pin GPIO_PIN_15
#define DB5_GPIO_Port GPIOC
#define DB4_Pin GPIO_PIN_0
#define DB4_GPIO_Port GPIOA
#define DB7_Pin GPIO_PIN_1
#define DB7_GPIO_Port GPIOA
#define E1_Pin GPIO_PIN_2
#define E1_GPIO_Port GPIOA
#define R_W_Pin GPIO_PIN_3
#define R_W_GPIO_Port GPIOA
#define E2_Pin GPIO_PIN_4
#define E2_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_5
#define RS_GPIO_Port GPIOA
#define ADC_DRDY_Pin GPIO_PIN_10
#define ADC_DRDY_GPIO_Port GPIOB
#define ADC_RST_Pin GPIO_PIN_11
#define ADC_RST_GPIO_Port GPIOB
#define ADC_CS_Pin GPIO_PIN_12
#define ADC_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
