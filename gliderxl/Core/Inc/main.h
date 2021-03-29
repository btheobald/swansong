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
#include "stm32g4xx_hal.h"

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
void myprintf(const char *fmt, ...);
void UART_GPS_Callback();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_RESET_Pin GPIO_PIN_13
#define GPS_RESET_GPIO_Port GPIOC
#define GPS_FORCEON_Pin GPIO_PIN_14
#define GPS_FORCEON_GPIO_Port GPIOC
#define GPS_EXT_INT_Pin GPIO_PIN_15
#define GPS_EXT_INT_GPIO_Port GPIOC
#define GPS_FIX_Pin GPIO_PIN_0
#define GPS_FIX_GPIO_Port GPIOA
#define IMU_SPI_LS_EN_Pin GPIO_PIN_1
#define IMU_SPI_LS_EN_GPIO_Port GPIOA
#define BUZZER_DAC_Pin GPIO_PIN_4
#define BUZZER_DAC_GPIO_Port GPIOA
#define NAV_STAT_0_Pin GPIO_PIN_4
#define NAV_STAT_0_GPIO_Port GPIOC
#define NAV_STAT_1_Pin GPIO_PIN_5
#define NAV_STAT_1_GPIO_Port GPIOC
#define I2C_EXT_INT_Pin GPIO_PIN_2
#define I2C_EXT_INT_GPIO_Port GPIOB
#define HPS_A_TIM2_CH4_Pin GPIO_PIN_11
#define HPS_A_TIM2_CH4_GPIO_Port GPIOB
#define HPS_Current_Sense_A_Pin GPIO_PIN_12
#define HPS_Current_Sense_A_GPIO_Port GPIOB
#define Battery_Voltage_Sense_Pin GPIO_PIN_13
#define Battery_Voltage_Sense_GPIO_Port GPIOB
#define Battery_Current_Sense_Pin GPIO_PIN_14
#define Battery_Current_Sense_GPIO_Port GPIOB
#define Battery_Temperature_Sense_Pin GPIO_PIN_15
#define Battery_Temperature_Sense_GPIO_Port GPIOB
#define SENSOR_I2C2_SDA_Pin GPIO_PIN_8
#define SENSOR_I2C2_SDA_GPIO_Port GPIOA
#define SENSOR_I2C2_SCL_Pin GPIO_PIN_9
#define SENSOR_I2C2_SCL_GPIO_Port GPIOA
#define RADIO_SET_Pin GPIO_PIN_10
#define RADIO_SET_GPIO_Port GPIOA
#define SD_NCS_Pin GPIO_PIN_15
#define SD_NCS_GPIO_Port GPIOA
#define IMU_NCS_Pin GPIO_PIN_11
#define IMU_NCS_GPIO_Port GPIOC
#define RADIO_UART5_TX_Pin GPIO_PIN_12
#define RADIO_UART5_TX_GPIO_Port GPIOC
#define RADIO_UART5_RX_Pin GPIO_PIN_2
#define RADIO_UART5_RX_GPIO_Port GPIOD
#define GPS_USART1_TX_Pin GPIO_PIN_6
#define GPS_USART1_TX_GPIO_Port GPIOB
#define GPS_USART1_RX_Pin GPIO_PIN_7
#define GPS_USART1_RX_GPIO_Port GPIOB
#define USER_BTN_Pin GPIO_PIN_8
#define USER_BTN_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_9
#define IMU_INT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi3
#define SD_CS_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_15
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
