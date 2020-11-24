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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "fatfs.h"
#include "bts7xx.h"
#include "stm32f3xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_CARDS_USED 8
#define NUM_CARDS 8
#define NUM_CHANNELS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_TypeDef *card_ports[NUM_CARDS_USED] = { PDMC_CS7_GPIO_Port,
PDMC_CS6_GPIO_Port, PDMC_CS1_GPIO_Port, PDMC_CS5_GPIO_Port, PDMC_CS2_GPIO_Port,
PDMC_CS4_GPIO_Port, PDMC_CS3_GPIO_Port, PDMC_CS8_GPIO_Port };
uint16_t card_pins[NUM_CARDS_USED] = { PDMC_CS7_Pin,
PDMC_CS6_Pin, PDMC_CS1_Pin, PDMC_CS5_Pin, PDMC_CS2_Pin, PDMC_CS4_Pin,
PDMC_CS3_Pin, PDMC_CS8_Pin };

//SPI_HandleTypeDef hspi2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t current_state;
volatile uint32_t user_channels;
volatile uint32_t pwm_channels;
volatile uint8_t user_dutycycles[32];
volatile uint8_t current_dutycycles[32];

uint32_t read_channel_states();
void CAN_set_channel_states(uint32_t new_state);
void CAN_set_pwm_duty_cycle(uint8_t channel, uint8_t duty_cycle);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PDMC_CS5_GPIO_Port, PDMC_CS5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS6_GPIO_Port, PDMC_CS6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS1_GPIO_Port, PDMC_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS7_GPIO_Port, PDMC_CS7_Pin, GPIO_PIN_SET);

	uint8_t receiveBuff;

	uint16_t Tx16SPIBuffer[1] = { 0 };
	uint16_t RxSPIBuffer[1] = { 0 };
	Tx16SPIBuffer[0] = 0xFFFF;
	RxSPIBuffer[0] = 0xFFFF;
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) Tx16SPIBuffer, (uint8_t*) RxSPIBuffer, 1, 1000);

	//  Set DCR register, with SWR set to 0 - bit needs to be set for writing a register
	uint8_t DCR = 0x00;
	for (int i = 0; i < NUM_CARDS_USED; i++) {
		receiveBuff = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND, &DCR, card_ports[i], card_pins[i]);
	}

	//  BTS7XX_OCR_OBJ ocrObj;

	user_channels = 0;
	pwm_channels = 0;
	current_state = 0;

	memset(user_dutycycles, 0, 32 * sizeof(uint8_t));
	memset(current_dutycycles, 0, 32 * sizeof(uint8_t));
	// Configure channels - everything start off??
	set_channel_states(0);

	HAL_Delay(1000);

	// Configure channels to enable CC
	CAN_set_channel_states(PDM_POWER_CC_MASK);

	//Test by setting all states high
//	set_channel_states(UINT32_MAX);

// start can
	Configure_CAN(&hcan);

	uint32_t init_startup_id = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x00, 0x0);
	uint32_t setchannelID = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x02, 0x00);
	uint32_t requestDutyCycleID = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x3, 0x0);
	uint32_t setDutyCycleID = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x4, 0x0);

	// setup messages
	PDM_Heartbeat_t heartbeat_msg;
	PDM_TransmitDutyCycle_t transmit_cycle_msg;
	CAN_TxHeaderTypeDef header = { 0 };
	header.IDE = CAN_ID_EXT;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	HAL_StatusTypeDef result;
	uint32_t txMailbox = 0;

	// startup sequence

	// wait for initiate startup
	uint8_t wait_flag = 0;
	while (!wait_flag) {
		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
			// pull msg out
			CAN_MSG_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &(msg.header), msg.data);

			// PDM_SetChannelStates
			if (msg.header.ExtId == init_startup_id) {
				wait_flag = 1;
				break;
			}

		}

		HAL_Delay(1);
	}

	// send startup ok
	current_state = read_channel_states();
	PDM_StartupOk_t startupOk_msg = Compose_PDM_StartupOk(current_state);
	header.ExtId = startupOk_msg.id;
	header.DLC = sizeof(startupOk_msg.data);
	result = HAL_CAN_AddTxMessage(&hcan, &header, startupOk_msg.data, &txMailbox);

	// start heartbeat timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		Error_Handler();
	}

	// start PWM timer
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		Error_Handler();
	}

	// start duty_cycle ramper timer
	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// read current state
		current_state = read_channel_states();

		// send heartbeat if enough time has passed
		if (send_heartbeat == 1) {
			heartbeat_msg = Compose_PDM_Heartbeat(current_state);
			header.ExtId = heartbeat_msg.id;
			header.DLC = sizeof(heartbeat_msg.data);
			result = HAL_CAN_AddTxMessage(&hcan, &header, heartbeat_msg.data, &txMailbox);
			send_heartbeat = 0;
		}

		// check message queue
		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
			// pull msg out
			CAN_MSG_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &(msg.header), msg.data);

			// PDM_SetChannelStates
			if (msg.header.ExtId == setchannelID) {
				uint32_t set_channels = 0;
				Parse_PDM_SetChannelStates(msg.data, &set_channels);

				CAN_set_channel_states(set_channels);

			} else if (msg.header.ExtId == requestDutyCycleID) {
				uint8_t channel = 0;
				Parse_PDM_RequestDutyCycle(msg.data, &channel);

				transmit_cycle_msg = Compose_PDM_TransmitDutyCycle(channel, user_dutycycles[channel]);
				header.ExtId = transmit_cycle_msg.id;
				header.DLC = sizeof(transmit_cycle_msg.data);
				result = HAL_CAN_AddTxMessage(&hcan, &header, transmit_cycle_msg.data, &txMailbox);

			} else if (msg.header.ExtId == setDutyCycleID) {
				uint8_t channel = 0;
				uint8_t duty_cycle = 0;
				Parse_PDM_SetDutyCycle(msg.data, &channel, &duty_cycle);

				CAN_set_pwm_duty_cycle(channel, duty_cycle);
			}

		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
			| RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void CAN_set_channel_states(uint32_t new_state) {
	uint32_t prim = __get_PRIMASK();

	// disable interrupts
	__disable_irq();

	// remove PWM channels so we don't overwrite any current PWM stuff
	// these will be handled in PWM timers
	user_channels = new_state & (~PWM_CHANNELS);
	pwm_channels = (new_state & PWM_CHANNELS);

	new_state = ((current_state & PWM_CHANNELS) | user_channels);
	set_channel_states(new_state);

	// enable interrupts if previously enabled
	if (!prim) {
		__enable_irq();
	}

}

void CAN_set_pwm_duty_cycle(uint8_t channel, uint8_t duty_cycle) {
	uint32_t prim = __get_PRIMASK();

	// disable interrupts
	__disable_irq();

	// remove possible invalid indicies
	if (channel > 31) {
		channel = 0;
	}

	if (duty_cycle > MAX_DUTY_CYCLE) {
		duty_cycle = MAX_DUTY_CYCLE;
	}

	user_dutycycles[channel] = duty_cycle;

	// enable interrupts if previously enabled
	if (!prim) {
		__enable_irq();
	}
}

void set_channel_states(uint32_t powerChannels) {
	// pull out each of the 8 channels

	current_state = powerChannels;

	// set states physically
	for (int i = 0; i < NUM_CARDS_USED; i++) {
		uint8_t value = (current_state >> (i * 4)) & 0xF;
		BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND, &value, card_ports[i], card_pins[i]);
	}
}

uint32_t read_channel_states() {
	uint32_t channel_values = 0;
	for (int i = 0; i < NUM_CARDS_USED; i++) {
		uint8_t result = BTS7XX_ReadRegister(&hspi2, BTS7XX_READ_OUT_COMMAND, card_ports[i], card_pins[i]);
		channel_values |= (result & 0xF) << (i * 4);
	}
	return channel_values;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
