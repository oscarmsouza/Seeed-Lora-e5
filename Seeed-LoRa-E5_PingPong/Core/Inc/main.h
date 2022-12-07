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
#include "stm32wlxx_hal.h"

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
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)

#define USARTx_RX_Pin GPIO_PIN_7
#define USARTx_RX_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_5
#define L2_GPIO_Port GPIOB
#define DBG1_Pin GPIO_PIN_0
#define DBG1_GPIO_Port GPIOA
#define BUT1_Pin GPIO_PIN_13
#define BUT1_GPIO_Port GPIOB
#define BUT1_EXTI_IRQn EXTI15_10_IRQn
#define USARTx_TX_Pin GPIO_PIN_6
#define USARTx_TX_GPIO_Port GPIOB

#define RF_CTRL1_Pin GPIO_PIN_4
#define RF_CTRL1_GPIO_Port GPIOA
#define RF_CTRL2_Pin GPIO_PIN_5
#define RF_CTRL2_GPIO_Port GPIOA

#define PLAY1_Pin GPIO_PIN_9
#define PLAY1_GPIO_Port GPIOA
#define PLAY3_Pin GPIO_PIN_1
#define PLAY3_GPIO_Port GPIOC
#define BOTAO2_Pin GPIO_PIN_3
#define BOTAO2_GPIO_Port GPIOA
#define BOTAO1_Pin GPIO_PIN_2
#define BOTAO1_GPIO_Port GPIOA
#define PLAY2_Pin GPIO_PIN_10
#define PLAY2_GPIO_Port GPIOB
#define PLAY5_Pin GPIO_PIN_15
#define PLAY5_GPIO_Port GPIOA
#define PLAY6_Pin GPIO_PIN_15
#define PLAY6_GPIO_Port GPIOB
#define PLAY8_Pin GPIO_PIN_3
#define PLAY8_GPIO_Port GPIOB
#define PLAY7_Pin GPIO_PIN_4
#define PLAY7_GPIO_Port GPIOB
#define PLAY4_Pin GPIO_PIN_9
#define PLAY4_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOA


/* USER CODE BEGIN Private defines */
#define LED1_ON 		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
#define LED1_OFF 		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

#define LED2_ON 		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
#define LED2_OFF 		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);

#define DISPLAY1_ON 	HAL_GPIO_WritePin(PLAY1_GPIO_Port, PLAY1_Pin, 0);
#define DISPLAY1_OFF 	HAL_GPIO_WritePin(PLAY1_GPIO_Port, PLAY1_Pin, 1);

#define DISPLAY2_ON 	HAL_GPIO_WritePin(PLAY2_GPIO_Port, PLAY2_Pin, 0);
#define DISPLAY2_OFF 	HAL_GPIO_WritePin(PLAY2_GPIO_Port, PLAY2_Pin, 1);

#define DISPLAY3_ON 	HAL_GPIO_WritePin(PLAY3_GPIO_Port, PLAY3_Pin, 0);
#define DISPLAY3_OFF 	HAL_GPIO_WritePin(PLAY3_GPIO_Port, PLAY3_Pin, 1);

#define DISPLAY4_ON 	HAL_GPIO_WritePin(PLAY4_GPIO_Port, PLAY4_Pin, 0);
#define DISPLAY4_OFF 	HAL_GPIO_WritePin(PLAY4_GPIO_Port, PLAY4_Pin, 1);

#define DISPLAY5_ON 	HAL_GPIO_WritePin(PLAY5_GPIO_Port, PLAY5_Pin, 0);
#define DISPLAY5_OFF 	HAL_GPIO_WritePin(PLAY5_GPIO_Port, PLAY5_Pin, 1);

#define DISPLAY6_ON 	HAL_GPIO_WritePin(PLAY6_GPIO_Port, PLAY6_Pin, 0);
#define DISPLAY6_OFF 	HAL_GPIO_WritePin(PLAY6_GPIO_Port, PLAY6_Pin, 1);

#define DISPLAY7_ON 	HAL_GPIO_WritePin(PLAY7_GPIO_Port, PLAY7_Pin, 0);
#define DISPLAY7_OFF 	HAL_GPIO_WritePin(PLAY7_GPIO_Port, PLAY7_Pin, 1);

#define DISPLAY8_ON 	HAL_GPIO_WritePin(PLAY8_GPIO_Port, PLAY8_Pin, 0);
#define DISPLAY8_OFF 	HAL_GPIO_WritePin(PLAY8_GPIO_Port, PLAY8_Pin, 1);

#define BOTAO1_STATUS	  HAL_GPIO_ReadPin(BOTAO1_GPIO_Port, BOTAO1_Pin);
#define BOTAO2_STATUS	  HAL_GPIO_ReadPin(BOTAO2_GPIO_Port, BOTAO2_Pin);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
