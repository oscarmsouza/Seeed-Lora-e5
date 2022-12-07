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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "sys_app.h"
#include "subghz_phy_app.h"
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
bool comando_status;
bool vvStatus;
uint8_t numero_vv, numero_cv, vvSuspenso = 0, indexDigitos;
uint32_t actual_token, next_token;
int indexToken;
uint8_t buffRxProgram[64] = { 0 };
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t Address = 0, PageError = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint32_t data32 = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t display_num(uint8_t numero);
void teste_display(void);
void teste_led(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);

void fn_send_test(void);
bool Flash_Write_data(uint64_t dataTowrite);
uint64_t Flash_Read_Data(void);
bool erase_flash(void);
void SystemClock_Config(void);
bool codec_and_send_start_frame_rc(void);
bool codec_and_send_action_frame_cc(void);
bool codec_and_send_frame_command(void);
bool codec_and_send_action_frame_rc(void);
static uint32_t GetPage(uint32_t Address);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t randomTaskNumber = 0;

//**************GLOBAL VARIABLES*********************************
Machine_States_t e_Current_Machine_State = INITIALIZING,
		e_Previous_Machine_State = NONE;

//*********************** CONTROL FUNCTIONS ***************************
void fn_Change_Machine_State(Machine_States_t state) {
	e_Previous_Machine_State = e_Current_Machine_State;
	e_Current_Machine_State = state;
}
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
	numero_vv = 1;
	display_num(numero_vv);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
    if (HAL_GPIO_ReadPin(BOTAO1_GPIO_Port, BOTAO1_Pin)) {
    	codec_and_send_start_frame_rc();
    }
	if (HAL_GPIO_ReadPin(BOTAO2_GPIO_Port, BOTAO2_Pin)) {
		if (numero_vv < 5)
			numero_vv++;
		else
			numero_vv = 1;
		display_num(numero_vv);
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

//*****************  FUNCTIONS  ****************//

uint8_t check_receiver_frame_rc(void) {
	uint8_t ok = 2;
	uint8_t buff_program[14] = { 0 };

	char buffer_token[8] = { 0 };

//se receber payload, processar
//TODO: modo de temporizacao pois pode chegar errado
	if (Lora_RX(buff_program) == 1) {

		uint8_t numVVinfor = buff_program[0] - 48;
		uint8_t numRCinfor = buff_program[1] - 48;
		//se o numero da central e do controle estiverem corretos segue
		if (numVVinfor != numero_vv && numRCinfor != numeroCadeado) {
			//se o numero da central e o numero do controle remoto estiver incorreta retorna erro de identidade {ERRO 3}
			return 3;
		}
		__NOP();
		//passa os numeros tokens antigos para novos
		actual_token = next_token;

		//altera para char
		itoa(actual_token, buffer_token, 10);

		//verifica se o numero token e o mesmo requerido na etapa anterior, se positivo segue
		if (buffer_token[0] == buff_program[2]
				&& buffer_token[1] == buff_program[3]
				&& buffer_token[2] == buff_program[4]
				&& buffer_token[3] == buff_program[5]
				&& buffer_token[4] == buff_program[6]
				&& buffer_token[5] == buff_program[7]
				&& buffer_token[6] == buff_program[8]
				&& buffer_token[7] == buff_program[9]) {
			__NOP();
			APP_TPRINTF("Token confirmado\r\n");

			//resgata o numero do index que veio da central para informar novo token
			indexToken = (((buff_program[10] - 48) * 100)
					+ ((buff_program[11] - 48) * 10) + (buff_program[12] - 48));

			//restaga a quantidede de digitos presentes no index (importante mais a frente)
			if (!(buff_program[10] - 48) && !(buff_program[11] - 48)) {
				indexDigitos = 1;
				indexToken *= 100;
			} else if (!(buff_program[10] - 48)) {
				indexDigitos = 2;
				indexToken *= 10;
			} else
				indexDigitos = 3;

			//resgata novo token a ser enviado para central
			actual_token = tokens[indexToken];

			APP_TPRINTF("Actual Token = %u, in the index= %d\r\n", actual_token,
					indexToken);
			//tudo certo

			//agora verifica o status, se 0 ativo, se 1 desativo
			ok = (buff_program[13] - 48);
		}
		__NOP();

	} else {
		//erro retorna zero
		APP_TPRINTF("Parametros de cadeado e/ou central errados!!!\r\n");
		ok = 4;
	}

	return ok;
}

bool codec_and_send_start_frame_rc(void) {

	bool ok = 0;
//payload inicial tem 16 caracteres
	uint8_t buffer_payload[16] = { 0 };
//pega o numero randomico para enviar o index
	int bffInxStart = randomTaskNumber;
//cria o char index
	char StartIndex[3] = { 0 };
//transforma o numero randomico em char
	itoa(bffInxStart, StartIndex, 10);

//acerta o char do numero randomico por deslocamento de zeros
	if (bffInxStart <= 99 && bffInxStart > 9) {

		StartIndex[2] = StartIndex[1];
		StartIndex[1] = StartIndex[0];
		StartIndex[0] = 48;
	} else if (bffInxStart <= 9) {
		StartIndex[2] = StartIndex[0];
		StartIndex[1] = 48;
		StartIndex[0] = 48;
	}

//resgata o token atual em funcao do index
	actual_token = tokens[bffInxStart];
	APP_TPRINTF("Actual Token = %u\r\n", actual_token);
	HAL_Delay(bffInxStart);
//cria um segundo index randomico para resgatar na proxima mensagem
	indexToken = randomTaskNumber;
//cria o char do segundo index
	char CommandIndex[3] = { 0 };
//transmorma em char o segundo numero randomico
	itoa(indexToken, CommandIndex, 10);
//acerto os zeros do segundo numero randomico
	if (indexToken <= 99 && indexToken > 9) {

		CommandIndex[2] = CommandIndex[1];
		CommandIndex[1] = CommandIndex[0];
		CommandIndex[0] = 48;
	} else if (indexToken <= 9) {
		CommandIndex[2] = CommandIndex[0];
		CommandIndex[1] = 48;
		CommandIndex[0] = 48;
	} else if (indexToken == 0){
		CommandIndex[2] = 48;
		CommandIndex[1] = 48;
		CommandIndex[0] = 48;
	}
	char tokenInChar[8]={0};
	sprintf(tokenInChar,"%lu",actual_token);
//pega e armazena o proximo numero token
	next_token = tokens[indexToken];

	APP_TPRINTF("Next Token[%u] = %u\r\n ", indexToken, next_token);

//monta o novo payload
	sprintf((char*)buffer_payload, "%i%i%s%s%s", numeroCadeado, numero_vv, StartIndex,
			tokenInChar, CommandIndex);

	__NOP();
	APP_TPRINTF("Payload  to send= %s\r\n", buffer_payload);
	write_master_string(buffer_payload);
//envia o novo payload
	MX_SubGHz_Phy_Process();
	//ok = send_start_frame(buffer_payload);
	return ok;

}

uint8_t check_interruption_receiver_frame_cc(void) {

//antes de tudo verifica se a mensagem e de mudanca de suspensao, neste caso ela deve conter 16 caracteres
	if (strlen((char*) buffRxProgram) != 16) {
		//se o tamanho for diferente de 16 retorna erro de payload {ERRO 9}
		return 9;
		__NOP();
	}
//se a mensagem for realmente de mudanca de status segue
//NOP

	char buffer_token[8] = { 0 };
	char buffer_index[3] = { 0 };
	char buffer_receiverindex[3] = { 0 };

//se receber payload, processar numero da central e numero do controle remoto

	uint8_t numRCinfor = buffRxProgram[0] - 48;
	uint8_t numVVinfor = buffRxProgram[1] - 48;

//processa o index do token recebido
	buffer_index[0] = buffRxProgram[2];
	buffer_index[1] = buffRxProgram[3];
	buffer_index[2] = buffRxProgram[4];
	unsigned int TokenIndex = atoi(buffer_index);

//processa o token recebido
	buffer_token[0] = buffRxProgram[5];
	buffer_token[1] = buffRxProgram[6];
	buffer_token[2] = buffRxProgram[7];
	buffer_token[3] = buffRxProgram[8];
	buffer_token[4] = buffRxProgram[9];
	buffer_token[5] = buffRxProgram[10];
	buffer_token[6] = buffRxProgram[11];
	buffer_token[7] = buffRxProgram[12];
	long int TokenRec = atol(buffer_token);

//processa o index para envio do token
	buffer_receiverindex[0] = buffRxProgram[13];
	buffer_receiverindex[1] = buffRxProgram[14];
	buffer_receiverindex[2] = buffRxProgram[15];
	unsigned int IndexToken = atoi(buffer_receiverindex);

//se o numero da central estiver correto segue
	if (numVVinfor != numeroCentral) {
		//se o numero da central estiver incorreta retorna erro de identidade {ERRO 3}
		return 3;
		__NOP();
	}

//se a posicao do index corresponder ao token indicado segue
	if (tokens[TokenIndex] != TokenRec) {
		//se o token estiver incorreto retorna erro de token {ERRO 6}
		return 6;
		__NOP();
	}

//passa o numero do controlador
	numero_cv = numRCinfor;

//passa os numeros tokens antigos para novos
	actual_token = next_token;
//pega o proximo numero token a ser enviado
	next_token = tokens[IndexToken];

//TODO: verificar status do VV apos o comando e retornar se realmente esta SUSPENSO

	return 1;
}

bool codec_and_send_frame_command(void) {

	bool ok = 0;
//payload de comando tem 14 caracteres
	char buffer_payload[14] = { 0 };
//pega o numero randomico para enviar o index
	int bffInxStart = randomTaskNumber;

//resgata o token atual em funcao do index
	actual_token = tokens[bffInxStart];
	APP_TPRINTF("Actual Token = %u\r\n", actual_token);

//monta o novo payload
	sprintf(buffer_payload, "%i%i%lu%d1", numeroCentral, numero_cv, next_token,
			bffInxStart);

	__NOP();
	APP_TPRINTF("Payload = %s\r\n", buffer_payload);
//envia o nuvo payload
	//ok = send_central_frame(buffer_payload);
	return ok;

}

bool codec_and_send_action_frame_rc(void) {

	int64_t dataRead = Flash_Read_Data();
	uint8_t decimal = (dataRead / 10000) - 1000;

	if (dataRead % 10000000 == 0) {
		e_Current_Machine_State = VV_ACTIVATED;
		vvSuspenso = 0;
	} else {
		e_Current_Machine_State = VV_SUSPENDED;
		vvSuspenso = 1;
	}

	if (decimal == 1)
		indexToken = (dataRead / 1000) - 1001000;
	if (decimal == 2)
		indexToken = (dataRead / 100) -  1002000;
	if (decimal == 3)
		indexToken = (dataRead / 10) -   1003000;

	APP_TPRINTF("Memory = %lu  - status do bloqueio pela memoria = %u - Index na memoria = %u\r\n",
			dataRead,vvSuspenso, indexToken);

	__NOP();

	actual_token = tokens[indexToken];

	bool ok = 0;
//payload de activacao tem 14 caracteres
	char buffer_payload[14] = { 0 };
//resgata o token atual em funcao do ultimo index
	APP_TPRINTF("Actual Token = %u\r\n", actual_token);
//cria um index randomico para resgatar na proxima mensagem
	int bffInxComm = randomTaskNumber;
//cria o char do index
	char CommandIndex[3] = { 0 };
//transmorma em char o numero randomico
	itoa(bffInxComm, CommandIndex, 10);
//acerto os zeros do numero randomico
	if (bffInxComm <= 99 && bffInxComm > 9) {

		CommandIndex[2] = CommandIndex[1];
		CommandIndex[1] = CommandIndex[0];
		CommandIndex[0] = 48;
	} else if (bffInxComm <= 9) {
		CommandIndex[2] = CommandIndex[0];
		CommandIndex[1] = 48;
		CommandIndex[0] = 48;
	}

//pega e armazena o proximo numero token
	next_token = tokens[bffInxComm];

	APP_TPRINTF("Next Token = %u\r\n", next_token);

//monta o novo payload
	sprintf(buffer_payload, "%i%i%lu%s1", numeroCadeado, numero_vv,
			actual_token, CommandIndex);

	__NOP();
	APP_TPRINTF("Payload  to send= %s\r\n", buffer_payload);
//envia o nuvo payload
	ok = send_central_frame(buffer_payload);
	__NOP();
	return ok;

}

uint8_t check_finish_frame_rc(void) {
	uint8_t ok = 0;
	uint8_t buff_program[10] = { 0 };

	char buffer_token[8] = { 0 };

//se receber payload, processar
//TODO: modo de temporizacao pois pode chegar errado
	if (Lora_RX(buff_program) == 1) {

		uint8_t numVVinfor = buff_program[0] - 48;
		uint8_t numRCinfor = buff_program[1] - 48;
		//se o numero da central e do controle estiverem corretos segue
		if (numVVinfor != numero_vv && numRCinfor != numeroCadeado) {
			//se o numero da central e o numero do controle remoto estiver incorreta retorna erro de identidade {ERRO 3}
			return 3;
		}
		__NOP();
		//passa os numeros tokens antigos para novos
		actual_token = next_token;

		//altera para char
		itoa(actual_token, buffer_token, 10);

		//verifica se o numero token e o mesmo requerido na etapa anterior, se positivo segue
		if (buffer_token[0] == buff_program[2]
				&& buffer_token[1] == buff_program[3]
				&& buffer_token[2] == buff_program[4]
				&& buffer_token[3] == buff_program[5]
				&& buffer_token[4] == buff_program[6]
				&& buffer_token[5] == buff_program[7]
				&& buffer_token[6] == buff_program[8]
				&& buffer_token[7] == buff_program[9]) {
			__NOP();
			APP_TPRINTF("Token confirmado\r\n");
			APP_TPRINTF("Operador Liberado\r\n");
			ok = 1;
			//TODO: limpar tokens e index intrelacados (melhor opcao seria resetar o controle remoto)
		}
		__NOP();

	} else {
		//erro retorna zero
		APP_TPRINTF("Parametros de cadeado e/ou central errados!!!\r\n");
		ok = 4;
	}

	return ok;
}

int32_t LED_control(int value) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, value);
	return 0;
}

bool Flash_Write_data(uint64_t dataTowrite) {
//uint64_t flashBff = 1360000001;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Erase the user Flash area
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_USER_START_ADDR);

	/* Get the number of pages to erase from 1st page */
	NbOfPages = 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = FirstPage;
	EraseInitStruct.NbPages = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		__NOP();
	}
	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	__NOP();
	Address = FLASH_USER_START_ADDR;
//while (Address < FLASH_USER_END_ADDR) {
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, dataTowrite)
			== HAL_OK) {
		//Address = Address + 8; /* increment to next double word*/
		__NOP();
	} else {
		/* Error occurred while writing data in Flash memory.
		 User can add here some code to deal with this error */
		APP_TPRINTF("\r\n");
		while (1) {
			/* Turn on LED3 */
			APP_TPRINTF("*");
		}
	}
//}
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	__NOP();
	return 1;
}

/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t GetPage(uint32_t Addr) {
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}

bool erase_flash(void) {
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Erase the user Flash area
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st page to erase */
	FirstPage = GetPage(FLASH_USER_START_ADDR);

	/* Get the number of pages to erase from 1st page */
	NbOfPages = 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = FirstPage;
	EraseInitStruct.NbPages = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		__NOP();
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
	return 1;

//FIM DE APAGAR MEMORIA

}

uint64_t Flash_Read_Data(void) {

	uint64_t data64 = 0;
	/* Check if the programmed data is OK
	 MemoryProgramStatus = 0: data programmed correctly
	 MemoryProgramStatus != 0: number of words not programmed correctly ******/
	Address = FLASH_USER_START_ADDR;
	data64 = *(__IO uint64_t*) Address;
	__NOP();
	return data64;
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
