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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define J108_1_Pin GPIO_PIN_2
#define J108_1_GPIO_Port GPIOE
#define A0_Pin GPIO_PIN_3
#define A0_GPIO_Port GPIOE
#define J108_2_Pin GPIO_PIN_4
#define J108_2_GPIO_Port GPIOE
#define A1_Pin GPIO_PIN_5
#define A1_GPIO_Port GPIOE
#define J108_3_Pin GPIO_PIN_6
#define J108_3_GPIO_Port GPIOE
#define A2_Pin GPIO_PIN_13
#define A2_GPIO_Port GPIOC
#define J108_4_Pin GPIO_PIN_0
#define J108_4_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_2
#define A3_GPIO_Port GPIOC
#define J108_5_Pin GPIO_PIN_3
#define J108_5_GPIO_Port GPIOC
#define J108_6_Pin GPIO_PIN_0
#define J108_6_GPIO_Port GPIOA
#define J107_8_Pin GPIO_PIN_3
#define J107_8_GPIO_Port GPIOA
#define J107_7_Pin GPIO_PIN_0
#define J107_7_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_1
#define D2_GPIO_Port GPIOB
#define J107_6_Pin GPIO_PIN_2
#define J107_6_GPIO_Port GPIOB
#define J107_5_Pin GPIO_PIN_7
#define J107_5_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOE
#define D3__Pin GPIO_PIN_9
#define D3__GPIO_Port GPIOE
#define J107_4_Pin GPIO_PIN_10
#define J107_4_GPIO_Port GPIOE
#define D5__Pin GPIO_PIN_11
#define D5__GPIO_Port GPIOE
#define J107_3_Pin GPIO_PIN_12
#define J107_3_GPIO_Port GPIOE
#define D6__Pin GPIO_PIN_13
#define D6__GPIO_Port GPIOE
#define J107_2_Pin GPIO_PIN_14
#define J107_2_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_15
#define D7_GPIO_Port GPIOE
#define USART3_TX_D0_Pin GPIO_PIN_10
#define USART3_TX_D0_GPIO_Port GPIOB
#define ETH_RXER_Pin GPIO_PIN_14
#define ETH_RXER_GPIO_Port GPIOB
#define D9__Pin GPIO_PIN_15
#define D9__GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_8
#define D8_GPIO_Port GPIOD
#define USART3_RX_D0_Pin GPIO_PIN_9
#define USART3_RX_D0_GPIO_Port GPIOD
#define D12_Pin GPIO_PIN_10
#define D12_GPIO_Port GPIOD
#define D13_Pin GPIO_PIN_11
#define D13_GPIO_Port GPIOD
#define D10__Pin GPIO_PIN_12
#define D10__GPIO_Port GPIOD
#define D11__Pin GPIO_PIN_13
#define D11__GPIO_Port GPIOD
#define J110_1_Pin GPIO_PIN_14
#define J110_1_GPIO_Port GPIOD
#define J111_2_Pin GPIO_PIN_15
#define J111_2_GPIO_Port GPIOD
#define J110_2_Pin GPIO_PIN_6
#define J110_2_GPIO_Port GPIOC
#define J111_3_Pin GPIO_PIN_7
#define J111_3_GPIO_Port GPIOC
#define J110_3_Pin GPIO_PIN_8
#define J110_3_GPIO_Port GPIOC
#define J111_4_Pin GPIO_PIN_9
#define J111_4_GPIO_Port GPIOC
#define J110_4_Pin GPIO_PIN_8
#define J110_4_GPIO_Port GPIOA
#define J111_5_Pin GPIO_PIN_15
#define J111_5_GPIO_Port GPIOA
#define J110_5_LED4_BLU_Pin GPIO_PIN_10
#define J110_5_LED4_BLU_GPIO_Port GPIOC
#define J111_6_Pin GPIO_PIN_11
#define J111_6_GPIO_Port GPIOC
#define J110_6_LED3_RED_Pin GPIO_PIN_12
#define J110_6_LED3_RED_GPIO_Port GPIOC
#define J111_7_Pin GPIO_PIN_0
#define J111_7_GPIO_Port GPIOD
#define J110_7_LED2_ORG_Pin GPIO_PIN_1
#define J110_7_LED2_ORG_GPIO_Port GPIOD
#define J111_8_Pin GPIO_PIN_2
#define J111_8_GPIO_Port GPIOD
#define J110_8_LED1_GRN_Pin GPIO_PIN_3
#define J110_8_LED1_GRN_GPIO_Port GPIOD
#define J109_1_Pin GPIO_PIN_4
#define J109_1_GPIO_Port GPIOD
#define J109_2_Pin GPIO_PIN_5
#define J109_2_GPIO_Port GPIOD
#define J109_3_Pin GPIO_PIN_6
#define J109_3_GPIO_Port GPIOD
#define J109_4_Pin GPIO_PIN_7
#define J109_4_GPIO_Port GPIOD
#define J109_5_Pin GPIO_PIN_4
#define J109_5_GPIO_Port GPIOB
#define J109_6_Pin GPIO_PIN_8
#define J109_6_GPIO_Port GPIOB
#define J109_7_Pin GPIO_PIN_0
#define J109_7_GPIO_Port GPIOE
#define J109_8_Pin GPIO_PIN_1
#define J109_8_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
