/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PDM_CAN_Messages.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint32_t current_state;
extern volatile uint32_t user_channels;
extern volatile uint32_t pwm_channels;

extern volatile uint8_t user_dutycycles[32];
extern volatile uint8_t current_dutycycles[32];


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_channel_states(uint32_t powerChannels);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OPEN_LOAD8_Pin GPIO_PIN_3
#define OPEN_LOAD8_GPIO_Port GPIOA
#define OPEN_LOAD3_Pin GPIO_PIN_4
#define OPEN_LOAD3_GPIO_Port GPIOF
#define OPEN_LOAD4_Pin GPIO_PIN_4
#define OPEN_LOAD4_GPIO_Port GPIOA
#define OPEN_LOAD2_Pin GPIO_PIN_5
#define OPEN_LOAD2_GPIO_Port GPIOA
#define OPEN_LOAD5_Pin GPIO_PIN_6
#define OPEN_LOAD5_GPIO_Port GPIOA
#define OPEN_LOAD1_Pin GPIO_PIN_7
#define OPEN_LOAD1_GPIO_Port GPIOA
#define OPEN_LOAD6_Pin GPIO_PIN_4
#define OPEN_LOAD6_GPIO_Port GPIOC
#define OPEN_LOAD7_Pin GPIO_PIN_5
#define OPEN_LOAD7_GPIO_Port GPIOC
#define PDMC_CS2_Pin GPIO_PIN_0
#define PDMC_CS2_GPIO_Port GPIOB
#define PDMC_CS5_Pin GPIO_PIN_1
#define PDMC_CS5_GPIO_Port GPIOB
#define TEMP_SENSE_Pin GPIO_PIN_2
#define TEMP_SENSE_GPIO_Port GPIOB
#define PDMC_CS1_Pin GPIO_PIN_7
#define PDMC_CS1_GPIO_Port GPIOE
#define S2_Pin GPIO_PIN_8
#define S2_GPIO_Port GPIOE
#define PDMC_CS4_Pin GPIO_PIN_9
#define PDMC_CS4_GPIO_Port GPIOE
#define S1_Pin GPIO_PIN_11
#define S1_GPIO_Port GPIOE
#define PDMC_CS3_Pin GPIO_PIN_12
#define PDMC_CS3_GPIO_Port GPIOE
#define LHI_Pin GPIO_PIN_13
#define LHI_GPIO_Port GPIOE
#define S0_Pin GPIO_PIN_14
#define S0_GPIO_Port GPIOE
#define PDMC_CS8_Pin GPIO_PIN_15
#define PDMC_CS8_GPIO_Port GPIOE
#define PDMC_CS7_Pin GPIO_PIN_10
#define PDMC_CS7_GPIO_Port GPIOB
#define PDMC_CS6_Pin GPIO_PIN_11
#define PDMC_CS6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define PWM_CHANNELS (PDMFLAG_ACU_FAN | PDMFLAG_LEFT_FAN | PDMFLAG_RIGHT_FAN)
#define MAX_DUTY_CYCLE 100
#define MAX_15bit 32768
#define ACCEL_RANGE 2
#define GYR_RANGE 2000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
