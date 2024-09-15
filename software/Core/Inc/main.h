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
void GameModeSelectionLoop();
void GameLoop();
void GameOverLoop();
void UaUaUaUaUaaaa();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TFT_LED_Pin GPIO_PIN_4
#define TFT_LED_GPIO_Port GPIOA
#define TFT_SCK_Pin GPIO_PIN_5
#define TFT_SCK_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_6
#define TFT_CS_GPIO_Port GPIOA
#define TFT_SDI_Pin GPIO_PIN_7
#define TFT_SDI_GPIO_Port GPIOA
#define TFT_RS_Pin GPIO_PIN_0
#define TFT_RS_GPIO_Port GPIOB
#define TFT_RESET_Pin GPIO_PIN_1
#define TFT_RESET_GPIO_Port GPIOB
#define ModeDownBtn_Pin GPIO_PIN_3
#define ModeDownBtn_GPIO_Port GPIOB
#define ModeDownBtn_EXTI_IRQn EXTI3_IRQn
#define ModeUpBtn_Pin GPIO_PIN_4
#define ModeUpBtn_GPIO_Port GPIOB
#define ModeUpBtn_EXTI_IRQn EXTI4_IRQn
#define ResetBtn_Pin GPIO_PIN_5
#define ResetBtn_GPIO_Port GPIOB
#define ResetBtn_EXTI_IRQn EXTI9_5_IRQn
#define PauseBtn_Pin GPIO_PIN_6
#define PauseBtn_GPIO_Port GPIOB
#define PauseBtn_EXTI_IRQn EXTI9_5_IRQn
#define BlackBtn_Pin GPIO_PIN_7
#define BlackBtn_GPIO_Port GPIOB
#define BlackBtn_EXTI_IRQn EXTI9_5_IRQn
#define WhiteBtn_Pin GPIO_PIN_8
#define WhiteBtn_GPIO_Port GPIOB
#define WhiteBtn_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
