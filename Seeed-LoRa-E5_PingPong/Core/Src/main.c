/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_subghz_phy.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ativo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t display_num(uint8_t numero);
void teste_display(void);
void teste_led(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SubGHz_Phy_Init();
  /* USER CODE BEGIN 2 */
	LED1_OFF
	LED2_OFF
	teste_led();
	uint8_t central_num = 0;
	display_num(central_num);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
    if (HAL_GPIO_ReadPin(BOTAO1_GPIO_Port, BOTAO1_Pin)) {
    	MX_SubGHz_Phy_Process();
    }
	if (HAL_GPIO_ReadPin(BOTAO2_GPIO_Port, BOTAO2_Pin)) {
		if (central_num < 10)
			central_num++;
		else
			central_num = 0;
		display_num(central_num);
		while (HAL_GPIO_ReadPin(BOTAO2_GPIO_Port, BOTAO2_Pin)) {
		}
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t display_num(uint8_t numero) {
	DISPLAY1_OFF
	DISPLAY2_OFF
	DISPLAY3_OFF
	DISPLAY4_OFF
	DISPLAY5_OFF
	DISPLAY6_OFF
	DISPLAY7_OFF
	DISPLAY8_OFF

	if (ativo)
		DISPLAY4_ON
	switch (numero) {
	case 0:
		DISPLAY1_ON
		DISPLAY2_ON
		DISPLAY3_ON
		DISPLAY5_ON
		DISPLAY7_ON
		DISPLAY8_ON
		break;
	case 1:
		DISPLAY8_ON
		DISPLAY3_ON
		break;
	case 2:
		DISPLAY7_ON
		DISPLAY8_ON
		DISPLAY6_ON
		DISPLAY1_ON
		DISPLAY2_ON
		break;
	case 3:
		DISPLAY7_ON
		DISPLAY8_ON
		DISPLAY3_ON
		DISPLAY2_ON
		DISPLAY6_ON
		break;
	case 4:
		DISPLAY5_ON
		DISPLAY6_ON
		DISPLAY8_ON
		DISPLAY3_ON
		break;
	case 5:
		DISPLAY7_ON
		DISPLAY5_ON
		DISPLAY6_ON
		DISPLAY3_ON
		DISPLAY2_ON
		break;
	case 6:
		DISPLAY7_ON
		DISPLAY5_ON
		DISPLAY6_ON
		DISPLAY1_ON
		DISPLAY2_ON
		DISPLAY3_ON
		break;
	case 7:
		DISPLAY7_ON
		DISPLAY8_ON
		DISPLAY3_ON
		break;
	case 8:
		DISPLAY1_ON
		DISPLAY2_ON
		DISPLAY3_ON
		DISPLAY5_ON
		DISPLAY6_ON
		DISPLAY7_ON
		DISPLAY8_ON

		break;
	case 9:

		DISPLAY3_ON
		DISPLAY5_ON
		DISPLAY6_ON
		DISPLAY7_ON
		DISPLAY8_ON
		DISPLAY2_ON

		break;

	default:
		break;
	}
	return 0;
}

void teste_display(void) {
	DISPLAY1_ON
	HAL_Delay(100);
	DISPLAY1_OFF
	HAL_Delay(100);
	DISPLAY2_ON
	HAL_Delay(100);
	DISPLAY2_OFF
	HAL_Delay(100);
	DISPLAY3_ON
	HAL_Delay(100);
	DISPLAY3_OFF
	HAL_Delay(100);
	DISPLAY4_ON
	HAL_Delay(100);
	DISPLAY4_OFF
	HAL_Delay(100);
	DISPLAY5_ON
	HAL_Delay(100);
	DISPLAY5_OFF
	HAL_Delay(100);
	DISPLAY6_ON
	HAL_Delay(100);
	DISPLAY6_OFF
	HAL_Delay(100);
	DISPLAY7_ON
	HAL_Delay(100);
	DISPLAY7_OFF
	HAL_Delay(100);
	DISPLAY8_ON
	HAL_Delay(100);
	DISPLAY8_OFF
	HAL_Delay(100);
	display_num(1);
	HAL_Delay(300);
	display_num(2);
	HAL_Delay(300);
	display_num(3);
	HAL_Delay(300);
	display_num(4);
	HAL_Delay(300);
	display_num(5);
	HAL_Delay(300);
	display_num(6);
	HAL_Delay(300);
	display_num(7);
	HAL_Delay(300);
	display_num(8);
	HAL_Delay(300);
	display_num(8);
	HAL_Delay(300);
	display_num(9);
	HAL_Delay(300);
	display_num(0);
	HAL_Delay(300);
	display_num(99);

}

void teste_led(void) {
	LED1_OFF
	LED2_OFF
	LED1_ON
	HAL_Delay(500);
	LED1_OFF
	HAL_Delay(100);
	LED2_ON
	HAL_Delay(500);
	LED2_OFF
	HAL_Delay(100);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
