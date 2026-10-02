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
#define CodDA_Pin GPIO_PIN_0
#define CodDA_GPIO_Port GPIOA
#define CodDB_Pin GPIO_PIN_1
#define CodDB_GPIO_Port GPIOA
#define data_AX12_Pin GPIO_PIN_2
#define data_AX12_GPIO_Port GPIOA
#define Battery_level_Pin GPIO_PIN_4
#define Battery_level_GPIO_Port GPIOA
#define Power_enable_stm_Pin GPIO_PIN_5
#define Power_enable_stm_GPIO_Port GPIOA
#define CodDA2_Pin GPIO_PIN_6
#define CodDA2_GPIO_Port GPIOA
#define CodDB2_Pin GPIO_PIN_7
#define CodDB2_GPIO_Port GPIOA
#define Moteur_1_DIR_Pin GPIO_PIN_1
#define Moteur_1_DIR_GPIO_Port GPIOB
#define Moteur_2_DIR_Pin GPIO_PIN_2
#define Moteur_2_DIR_GPIO_Port GPIOB
#define ARU_signal_Pin GPIO_PIN_14
#define ARU_signal_GPIO_Port GPIOB
#define CodG2B_Pin GPIO_PIN_6
#define CodG2B_GPIO_Port GPIOC
#define CodGA2_Pin GPIO_PIN_7
#define CodGA2_GPIO_Port GPIOC
#define Moteur_1_PWM_Pin GPIO_PIN_8
#define Moteur_1_PWM_GPIO_Port GPIOA
#define Moteur_2_PWM_Pin GPIO_PIN_9
#define Moteur_2_PWM_GPIO_Port GPIOA
#define Moteur_3_PWM_Pin GPIO_PIN_10
#define Moteur_3_PWM_GPIO_Port GPIOA
#define Moteur_4_PWM_Pin GPIO_PIN_11
#define Moteur_4_PWM_GPIO_Port GPIOA
#define Sabertooth1_TX_Pin GPIO_PIN_10
#define Sabertooth1_TX_GPIO_Port GPIOC
#define Sabertooth1_RX_Pin GPIO_PIN_11
#define Sabertooth1_RX_GPIO_Port GPIOC
#define Sabertooth2_TX_Pin GPIO_PIN_12
#define Sabertooth2_TX_GPIO_Port GPIOC
#define Sabertooth2_RX_Pin GPIO_PIN_2
#define Sabertooth2_RX_GPIO_Port GPIOD
#define Moteur_3_DIR_Pin GPIO_PIN_3
#define Moteur_3_DIR_GPIO_Port GPIOB
#define Moteur_4_DIR_Pin GPIO_PIN_4
#define Moteur_4_DIR_GPIO_Port GPIOB
#define CodGB_Pin GPIO_PIN_6
#define CodGB_GPIO_Port GPIOB
#define CodGA_Pin GPIO_PIN_7
#define CodGA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
