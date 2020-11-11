/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "fatfs.h"
#include "bts7xx.h"
#include "PDM_CAN_Messages.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_CARDS 8
#define NUM_CHANNELS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_TypeDef * card_ports[4] = {PDMC_CS7_GPIO_Port, PDMC_CS6_GPIO_Port, PDMC_CS1_GPIO_Port, PDMC_CS5_GPIO_Port};
uint16_t card_pins[4] = {PDMC_CS7_Pin, PDMC_CS6_Pin, PDMC_CS1_Pin, PDMC_CS5_Pin};

//SPI_HandleTypeDef hspi2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t current_state[NUM_CARDS];
void set_channel_states(uint32_t powerChannels);

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PDMC_CS5_GPIO_Port, PDMC_CS5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS6_GPIO_Port, PDMC_CS6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS1_GPIO_Port, PDMC_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS7_GPIO_Port, PDMC_CS7_Pin, GPIO_PIN_SET);

	uint8_t buffer[1] = { 0 };
	uint8_t receiveBuff[1] = { 0 };
	buffer[0] = (uint8_t) 0x00;
	receiveBuff[0] = (uint8_t) 0x00;

	uint16_t Tx16SPIBuffer[1] = { 0 };
	uint16_t RxSPIBuffer[1] = { 0 };
	Tx16SPIBuffer[0] = 0xFFFF;
	RxSPIBuffer[0] = 0xFFFF;
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) Tx16SPIBuffer,
			(uint8_t*) RxSPIBuffer, 1, 1000);

//  Set DCR register, with SWR set to 0 - bit needs to be set for writing a register
	buffer[0] = (uint8_t) 0x00;
	receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND,
			(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
	receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND,
			(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
	receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND,
			(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
	receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND,
			(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);

//  BTS7XX_OCR_OBJ ocrObj;

// Configure channels

	// start can
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	  {
	    Error_Handler();
	  }

	// setup messages
	PDM_Heartbeat_t heartbeat_msg;
	CAN_TxHeaderTypeDef header = { 0 };
	header.IDE = CAN_ID_EXT;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	HAL_StatusTypeDef result;
	uint32_t txMailbox = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// read current state

		set_channel_states( 0b0101010101010101);
		heartbeat_msg = Compose_PDM_Heartbeat(current_state);
		header.ExtId = heartbeat_msg.id;
		header.DLC = sizeof(heartbeat_msg.data);
		result = HAL_CAN_AddTxMessage(&hcan, &header, heartbeat_msg.data, &txMailbox);


/*

		buffer[0] = BTS7XX_OUT_CH1;

		// 1
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
						(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);

		// 2
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
						(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);

		// 3
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
						(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);

		// 4
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
*/
		HAL_Delay(500);
		set_channel_states( 0);

		heartbeat_msg = Compose_PDM_Heartbeat(current_state);
		header.ExtId = heartbeat_msg.id;
		header.DLC = sizeof(heartbeat_msg.data);
		result = HAL_CAN_AddTxMessage(&hcan, &header, heartbeat_msg.data, &txMailbox);
		/*

		buffer[0] = BTS7XX_OUT_CH1 | BTS7XX_OUT_CH2;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		buffer[0] = BTS7XX_OUT_CH1 | BTS7XX_OUT_CH2 | BTS7XX_OUT_CH3;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		buffer[0] = BTS7XX_OUT_CH1 | BTS7XX_OUT_CH2 | BTS7XX_OUT_CH3
				| BTS7XX_OUT_CH4;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		buffer[0] = BTS7XX_OUT_CH1 | BTS7XX_OUT_CH2 | BTS7XX_OUT_CH3;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		buffer[0] = BTS7XX_OUT_CH1 | BTS7XX_OUT_CH2;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		buffer[0] = BTS7XX_OUT_CH1;
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS5_GPIO_Port, PDMC_CS5_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS6_GPIO_Port, PDMC_CS6_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS1_GPIO_Port, PDMC_CS1_Pin);
		receiveBuff[0] = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND,
				(uint8_t*) buffer, PDMC_CS7_GPIO_Port, PDMC_CS7_Pin);
		HAL_Delay(500);

		*/

		HAL_Delay(500);
//
//	ocrObj.OCR_OCT0 = 1;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void set_channel_states(uint32_t powerChannels) {
	// pull out each of the 8 channels
	current_state[0] = (powerChannels >> 4 * 0) & 0xF;
	current_state[1] = (powerChannels >> 4 * 1) & 0xF;
	current_state[2] = (powerChannels >> 4 * 2) & 0xF;
	current_state[3] = (powerChannels >> 4 * 3) & 0xF;
	current_state[4] = (powerChannels >> 4 * 4) & 0xF;
	current_state[5] = (powerChannels >> 4 * 5) & 0xF;
	current_state[6] = (powerChannels >> 4 * 6) & 0xF;
	current_state[7] = (powerChannels >> 4 * 7) & 0xF;

	// set states physically
	for (int i = 0; i < 4; i++) {
		BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND, &current_state[i], card_ports[i], card_pins[i]);
	}
}

void read_channel_states(uint8_t channels[8]) {
	for (int i = 0; i < 4; i++) {
		channels[i] = BTS7XX_ReadRegister(&hspi2, BTS7XX_READ_OUT_COMMAND,
				card_ports[i], card_pins[i]);
	}

	for (int i = 4; i < 8; i++) {
		channels[i] = 0;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
