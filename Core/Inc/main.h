/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CUR_SENS_Pin GPIO_PIN_1
#define CUR_SENS_GPIO_Port GPIOA
#define MIC_SENS_Pin GPIO_PIN_3
#define MIC_SENS_GPIO_Port GPIOA
#define MIC_TH_Pin GPIO_PIN_4
#define MIC_TH_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_12
#define LED_B_GPIO_Port GPIOE
#define ESP_EN_Pin GPIO_PIN_13
#define ESP_EN_GPIO_Port GPIOE
#define ESP_GPIO1_Pin GPIO_PIN_14
#define ESP_GPIO1_GPIO_Port GPIOE
#define ESP_GPIO0_Pin GPIO_PIN_15
#define ESP_GPIO0_GPIO_Port GPIOE
#define MD_DIAG_Pin GPIO_PIN_10
#define MD_DIAG_GPIO_Port GPIOD
#define MD_INDEX_Pin GPIO_PIN_11
#define MD_INDEX_GPIO_Port GPIOD
#define MD_DIR_Pin GPIO_PIN_15
#define MD_DIR_GPIO_Port GPIOD
#define MD_STEP_Pin GPIO_PIN_2
#define MD_STEP_GPIO_Port GPIOG
#define MD_MS2_Pin GPIO_PIN_4
#define MD_MS2_GPIO_Port GPIOG
#define MD_MS1_Pin GPIO_PIN_5
#define MD_MS1_GPIO_Port GPIOG
#define MD_NEN_Pin GPIO_PIN_6
#define MD_NEN_GPIO_Port GPIOG
#define I2C2_AD0_Pin GPIO_PIN_4
#define I2C2_AD0_GPIO_Port GPIOB
#define I2C2_INT_Pin GPIO_PIN_5
#define I2C2_INT_GPIO_Port GPIOB
#define I2C2_INT_EXTI_IRQn EXTI9_5_IRQn
#define I2C1_AD0_Pin GPIO_PIN_0
#define I2C1_AD0_GPIO_Port GPIOE
#define I2C1_INT_Pin GPIO_PIN_1
#define I2C1_INT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
