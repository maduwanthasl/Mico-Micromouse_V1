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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LF_RECEIVER_Pin GPIO_PIN_0
#define LF_RECEIVER_GPIO_Port GPIOA
#define LD_RECEIVER_Pin GPIO_PIN_1
#define LD_RECEIVER_GPIO_Port GPIOA
#define LMF_Pin GPIO_PIN_2
#define LMF_GPIO_Port GPIOA
#define LMB_Pin GPIO_PIN_3
#define LMB_GPIO_Port GPIOA
#define RD_RECEIVER_Pin GPIO_PIN_4
#define RD_RECEIVER_GPIO_Port GPIOA
#define RF_RECEIVER_Pin GPIO_PIN_5
#define RF_RECEIVER_GPIO_Port GPIOA
#define RMF_Pin GPIO_PIN_6
#define RMF_GPIO_Port GPIOA
#define RMB_Pin GPIO_PIN_7
#define RMB_GPIO_Port GPIOA
#define ENA_RM_Pin GPIO_PIN_0
#define ENA_RM_GPIO_Port GPIOB
#define LF_EMMITER_Pin GPIO_PIN_12
#define LF_EMMITER_GPIO_Port GPIOB
#define D_EMMITER_Pin GPIO_PIN_13
#define D_EMMITER_GPIO_Port GPIOB
#define RF_EMMITER_Pin GPIO_PIN_14
#define RF_EMMITER_GPIO_Port GPIOB
#define LED_PWR_Pin GPIO_PIN_15
#define LED_PWR_GPIO_Port GPIOB
#define LED_SPD_Pin GPIO_PIN_8
#define LED_SPD_GPIO_Port GPIOA
#define LED_COM_Pin GPIO_PIN_9
#define LED_COM_GPIO_Port GPIOA
#define PUSH_BTN_Pin GPIO_PIN_12
#define PUSH_BTN_GPIO_Port GPIOA
#define ENA_LM_Pin GPIO_PIN_4
#define ENA_LM_GPIO_Port GPIOB
#define ENB_LM_Pin GPIO_PIN_5
#define ENB_LM_GPIO_Port GPIOB
#define PUSH_BTN2_Pin GPIO_PIN_7
#define PUSH_BTN2_GPIO_Port GPIOB
#define RM_PWM_Pin GPIO_PIN_8
#define RM_PWM_GPIO_Port GPIOB
#define LM_PWM_Pin GPIO_PIN_9
#define LM_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
