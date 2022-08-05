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
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_3
#define SW1_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOA
#define IMU_PWR_DIS_Pin GPIO_PIN_4
#define IMU_PWR_DIS_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOC
#define DXL_PWR_EN_Pin GPIO_PIN_5
#define DXL_PWR_EN_GPIO_Port GPIOC
#define DXL_6_TX_EN_Pin GPIO_PIN_15
#define DXL_6_TX_EN_GPIO_Port GPIOB
#define DXL_5_TX_EN_Pin GPIO_PIN_9
#define DXL_5_TX_EN_GPIO_Port GPIOC
#define DXL_3_TX_EN_Pin GPIO_PIN_15
#define DXL_3_TX_EN_GPIO_Port GPIOA
#define FT_EN_Pin GPIO_PIN_9
#define FT_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
