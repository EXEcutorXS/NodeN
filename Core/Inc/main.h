/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define NODE

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <otisProtocol.h>
#include "common.h"
#include "uartNode.h"
#include "sx127x.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define VREF_CAL_VALUE  (*(uint16_t*) 0x1FF80078)

#define FLASH_EEPROM_BASE 0x8080000
#define MAX_RETRIES 7

#define USE_NTC
//NTC SETTINGS
#define BETA 3950.0F
#define R_BALANCE 4700.0F
#define R_THERMISTOR_DEFAULT 4700.0F
#define HOME_TEMP 298.15F

#define WATCHDOG_INTERVAL 200
typedef struct
{
uint32_t rtcAlarm:1;
uint32_t opened:1;
uint32_t closed:1;
uint32_t uartRx:1;
uint32_t saveSettings:1;
uint32_t readConfig:1;
uint32_t voltage:1;
uint32_t alarmDelay:1;
uint32_t beacon:1;
uint32_t reqCrc:1;
uint32_t statusRequested:1;
}flag_t;
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
void deinitPorts();
void deinitAlarmInput ();
void deinitPowerInput ();
void sleep(uint32_t delay);
void initUart(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, SX127X_t* myRadioHandler);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOA
#define D1_EXTI_IRQn EXTI0_1_IRQn
#define TempPower_Pin GPIO_PIN_1
#define TempPower_GPIO_Port GPIOA
#define TempSensor_Pin GPIO_PIN_2
#define TempSensor_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_3
#define RESET_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define BLUE_Pin GPIO_PIN_12
#define BLUE_GPIO_Port GPIOB
#define ORANGE_Pin GPIO_PIN_13
#define ORANGE_GPIO_Port GPIOB
#define extPower_Pin GPIO_PIN_8
#define extPower_GPIO_Port GPIOA
#define extPower_EXTI_IRQn EXTI4_15_IRQn
#define USER1_Pin GPIO_PIN_6
#define USER1_GPIO_Port GPIOB
#define USER2_Pin GPIO_PIN_7
#define USER2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
