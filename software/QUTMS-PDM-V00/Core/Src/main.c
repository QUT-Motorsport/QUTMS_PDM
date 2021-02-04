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
#include "bmi160_Bosch.h"

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

#define TIMEOUT_MINUTE 60000
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

/* USER CODE BEGIN 1 */
// Writing functions for function pointers

int8_t bmi_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len);
int8_t bmi_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer,
		uint16_t len);
int8_t bmi_delay_i2c(uint32_t period);

/* USER CODE END 1 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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

	// start can
	Configure_CAN(&hcan);

	HAL_GPIO_WritePin(PDMC_CS5_GPIO_Port, PDMC_CS5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS6_GPIO_Port, PDMC_CS6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS1_GPIO_Port, PDMC_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PDMC_CS7_GPIO_Port, PDMC_CS7_Pin, GPIO_PIN_SET);

	uint8_t receiveBuff;

	uint16_t Tx16SPIBuffer[1] = { 0 };
	uint16_t RxSPIBuffer[1] = { 0 };
	Tx16SPIBuffer[0] = 0xFFFF;
	RxSPIBuffer[0] = 0xFFFF;
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) Tx16SPIBuffer,
			(uint8_t*) RxSPIBuffer, 1, 1000);

	//  Set DCR register, with SWR set to 0 - bit needs to be set for writing a register
	uint8_t DCR = 0x00;
	for (int i = 0; i < NUM_CARDS_USED; i++) {
		receiveBuff = BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_DCR_COMMAND,
				&DCR, card_ports[i], card_pins[i]);
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

	uint32_t init_startup_id = Compose_CANId(CAN_PRIORITY_NORMAL,
			CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x00, 0x0);
	uint32_t setchannelID = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM,
			0x0, CAN_TYPE_RECEIVE, 0x02, 0x00);
	uint32_t requestDutyCycleID = Compose_CANId(CAN_PRIORITY_NORMAL,
			CAN_SRC_ID_PDM, 0x0, CAN_TYPE_RECEIVE, 0x3, 0x0);
	uint32_t setDutyCycleID = Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_PDM,
			0x0, CAN_TYPE_RECEIVE, 0x4, 0x0);

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
	/*
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
	 */

	// send startup ok
	current_state = read_channel_states();
	PDM_StartupOk_t startupOk_msg = Compose_PDM_StartupOk(current_state);
	header.ExtId = startupOk_msg.id;
	header.DLC = sizeof(startupOk_msg.data);
	result = HAL_CAN_AddTxMessage(&hcan, &header, startupOk_msg.data,
			&txMailbox);

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

	struct bmi160_dev sensor;

		sensor.id = BMI160_I2C_ADDR;
		sensor.interface = BMI160_I2C_INTF;
		sensor.read = bmi_read_i2c;
		sensor.write = bmi_write_i2c;
		sensor.delay_ms = bmi_delay_i2c;

		int8_t rslt = BMI160_OK;
		rslt = bmi160_init(&sensor);

		rslt = BMI160_OK;

		/* Select the Output data rate, range of accelerometer sensor */
		sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
		sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

		/* Select the power mode of accelerometer sensor */
		sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

		/* Select the Output data rate, range of Gyroscope sensor */
		sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
		sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
		sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

		/* Select the power mode of Gyroscope sensor */
		sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

		/* Set the sensor configuration */
		rslt = bmi160_set_sens_conf(&sensor);

	/* USER CODE END 2 */


	/* After the above function call, accel and gyro parameters in the device structure
	 are set with default values, found in the datasheet of the sensor */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int i = 0;

	char msg[100];

	while (1) {

		int8_t a_rslt = BMI160_OK;
		int8_t g_rslt = BMI160_OK;
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;

		// Read Accel data
		a_rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);

		int16_t acc_x = (accel.x)*9.8/(MAX_15bit / ACCEL_RANGE); // sensitivity = (2^15)/(range*9.8) = 16384/9.8
		int16_t acc_y = (accel.y)*9.8/(MAX_15bit / ACCEL_RANGE);
		int16_t acc_z = (accel.z)*9.8/(MAX_15bit / ACCEL_RANGE);

		sprintf(msg, "Acc - r: %d, a: (%d,\t%d,\t%d) m/s^2.\r\n", a_rslt, acc_x,acc_y,acc_z);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char*) msg),HAL_MAX_DELAY);
		i++;

		HAL_Delay(100);

		// Read Gyro data

		g_rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &sensor);

		int16_t gyr_x = (gyro.x)*GYR_RANGE/MAX_15bit; // sensitivity = (2^15)/range
		int16_t gyr_y = (gyro.y)*GYR_RANGE/MAX_15bit;
		int16_t gyr_z = (gyro.z)*GYR_RANGE/MAX_15bit;

		sprintf(msg, "Gyr - r: %d, a: (%d,\t%d,\t%d) °/s.\r\n", g_rslt, gyr_x,gyr_y,gyr_z);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);


		// read current state
		current_state = read_channel_states();

		// send heartbeat if enough time has passed
		if (send_heartbeat == 1) {
			heartbeat_msg = Compose_PDM_Heartbeat(current_state);
			header.ExtId = heartbeat_msg.id;
			header.DLC = sizeof(heartbeat_msg.data);
			result = HAL_CAN_AddTxMessage(&hcan, &header, heartbeat_msg.data,
					&txMailbox);
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

				transmit_cycle_msg = Compose_PDM_TransmitDutyCycle(channel,
						user_dutycycles[channel]);
				header.ExtId = transmit_cycle_msg.id;
				header.DLC = sizeof(transmit_cycle_msg.data);
				result = HAL_CAN_AddTxMessage(&hcan, &header,
						transmit_cycle_msg.data, &txMailbox);

			} else if (msg.header.ExtId == setDutyCycleID) {
				uint8_t channel = 0;
				uint8_t duty_cycle = 0;
				Parse_PDM_SetDutyCycle(msg.data, &channel, &duty_cycle);

				CAN_set_pwm_duty_cycle(channel, duty_cycle);
			}

		}

		// Construct CAN message for accelerometer values





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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_ADC12;
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

int8_t bmi_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
	uint8_t address = dev_addr << 1 | 0;
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&hi2c1, address,
			&reg_addr, 1,
			TIMEOUT_MINUTE);

	if (result != HAL_OK) {
		// unable to send read request correctly
		return result;
	} else {
		return HAL_I2C_Master_Receive(&hi2c1, address, data, len,
				TIMEOUT_MINUTE);
	}
}

int8_t bmi_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buffer,
		uint16_t len) {

	uint8_t address = dev_addr << 1 | 0;

	// no crc as chip doesn't support it

	// transmit register as first byte, then all of buffer according to length

	uint8_t dataLength = 1 + len;
	uint8_t *data = (uint8_t*) calloc(dataLength, sizeof(uint8_t));
	data[0] = reg_addr;

	memcpy(data + 1, buffer, len);

	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&hi2c1, address, data,
			dataLength, TIMEOUT_MINUTE);

	// clean up dynamic memory
	free(data);

	return result;
}

int8_t bmi_delay_i2c(uint32_t period) {
	// delay ms function
	HAL_Delay(period);
}

/* USER CODE END 4 */


/* USER CODE BEGIN 5 */

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
		BTS7XX_WriteRegister(&hspi2, BTS7XX_WRITE_OUT_COMMAND, &value,
				card_ports[i], card_pins[i]);
	}
}

uint32_t read_channel_states() {
	uint32_t channel_values = 0;
	for (int i = 0; i < NUM_CARDS_USED; i++) {
		uint8_t result = BTS7XX_ReadRegister(&hspi2, BTS7XX_READ_OUT_COMMAND,
				card_ports[i], card_pins[i]);
		channel_values |= (result & 0xF) << (i * 4);
	}
	return channel_values;
}

/* USER CODE END 5 */

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
