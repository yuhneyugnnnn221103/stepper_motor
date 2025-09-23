/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define IN13_Pin GPIO_PIN_2
#define IN13_GPIO_Port GPIOE
#define IN12_Pin GPIO_PIN_3
#define IN12_GPIO_Port GPIOE
#define MOTOR_1_ENA_Pin GPIO_PIN_4
#define MOTOR_1_ENA_GPIO_Port GPIOE
#define DIR1_Pin GPIO_PIN_5
#define DIR1_GPIO_Port GPIOE
#define PUL1_Pin GPIO_PIN_6
#define PUL1_GPIO_Port GPIOE
#define MOTOR_PED_1_Pin GPIO_PIN_13
#define MOTOR_PED_1_GPIO_Port GPIOC
#define MOTOR_ALARM_1_Pin GPIO_PIN_14
#define MOTOR_ALARM_1_GPIO_Port GPIOC
#define MOTOR_PED_2_Pin GPIO_PIN_2
#define MOTOR_PED_2_GPIO_Port GPIOC
#define MOTOR_ALARM_2_Pin GPIO_PIN_3
#define MOTOR_ALARM_2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_0
#define DIR2_GPIO_Port GPIOA
#define PUL2_Pin GPIO_PIN_1
#define PUL2_GPIO_Port GPIOA
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOA
#define W5500_CK_Pin GPIO_PIN_5
#define W5500_CK_GPIO_Port GPIOA
#define W5500_MISO_Pin GPIO_PIN_6
#define W5500_MISO_GPIO_Port GPIOA
#define W5500_MOSI_Pin GPIO_PIN_7
#define W5500_MOSI_GPIO_Port GPIOA
#define W5500_INT_Pin GPIO_PIN_4
#define W5500_INT_GPIO_Port GPIOC
#define W5500_RST_Pin GPIO_PIN_5
#define W5500_RST_GPIO_Port GPIOC
#define OUTPUT_1_Pin GPIO_PIN_0
#define OUTPUT_1_GPIO_Port GPIOB
#define OUTPUT_2_Pin GPIO_PIN_1
#define OUTPUT_2_GPIO_Port GPIOB
#define OUTPUT_3_Pin GPIO_PIN_2
#define OUTPUT_3_GPIO_Port GPIOB
#define OUTPUT_4_Pin GPIO_PIN_7
#define OUTPUT_4_GPIO_Port GPIOE
#define OUTPUT_5_Pin GPIO_PIN_8
#define OUTPUT_5_GPIO_Port GPIOE
#define OUTPUT_8_Pin GPIO_PIN_9
#define OUTPUT_8_GPIO_Port GPIOE
#define OUTPUT_9_Pin GPIO_PIN_10
#define OUTPUT_9_GPIO_Port GPIOE
#define OUTPUT_10_Pin GPIO_PIN_11
#define OUTPUT_10_GPIO_Port GPIOE
#define OUTPUT_7_Pin GPIO_PIN_12
#define OUTPUT_7_GPIO_Port GPIOE
#define OUTPUT_6_Pin GPIO_PIN_13
#define OUTPUT_6_GPIO_Port GPIOE
#define CS_SPI_Pin GPIO_PIN_12
#define CS_SPI_GPIO_Port GPIOB
#define SCK_SPI_Pin GPIO_PIN_13
#define SCK_SPI_GPIO_Port GPIOB
#define MISO_SPI_Pin GPIO_PIN_14
#define MISO_SPI_GPIO_Port GPIOB
#define MOSI_SPI_Pin GPIO_PIN_15
#define MOSI_SPI_GPIO_Port GPIOB
#define INPUT_2_1_Pin GPIO_PIN_13
#define INPUT_2_1_GPIO_Port GPIOD
#define INPUT_2_2_Pin GPIO_PIN_14
#define INPUT_2_2_GPIO_Port GPIOD
#define INPUT_2_3_Pin GPIO_PIN_15
#define INPUT_2_3_GPIO_Port GPIOD
#define INPUT_2_4_Pin GPIO_PIN_6
#define INPUT_2_4_GPIO_Port GPIOC
#define INPUT_2_5_Pin GPIO_PIN_7
#define INPUT_2_5_GPIO_Port GPIOC
#define INPUT_2_6_Pin GPIO_PIN_8
#define INPUT_2_6_GPIO_Port GPIOC
#define INPUT_1_1_Pin GPIO_PIN_9
#define INPUT_1_1_GPIO_Port GPIOC
#define INPUT_1_2_Pin GPIO_PIN_8
#define INPUT_1_2_GPIO_Port GPIOA
#define INPUT_1_3_Pin GPIO_PIN_9
#define INPUT_1_3_GPIO_Port GPIOA
#define INPUT_1_4_Pin GPIO_PIN_10
#define INPUT_1_4_GPIO_Port GPIOA
#define RS485_2_RW_Pin GPIO_PIN_15
#define RS485_2_RW_GPIO_Port GPIOA
#define RS485_2_TX_Pin GPIO_PIN_10
#define RS485_2_TX_GPIO_Port GPIOC
#define RS485_2_RX_Pin GPIO_PIN_11
#define RS485_2_RX_GPIO_Port GPIOC
#define MOTOR_2_ENA_Pin GPIO_PIN_0
#define MOTOR_2_ENA_GPIO_Port GPIOD
#define IN22_Pin GPIO_PIN_1
#define IN22_GPIO_Port GPIOD
#define IN23_Pin GPIO_PIN_2
#define IN23_GPIO_Port GPIOD
#define IN24_Pin GPIO_PIN_3
#define IN24_GPIO_Port GPIOD
#define RS485_1_RW_Pin GPIO_PIN_4
#define RS485_1_RW_GPIO_Port GPIOD
#define RS485_1_TX_Pin GPIO_PIN_5
#define RS485_1_TX_GPIO_Port GPIOD
#define RS485_1_RX_Pin GPIO_PIN_6
#define RS485_1_RX_GPIO_Port GPIOD
#define INPUT_1_5_Pin GPIO_PIN_3
#define INPUT_1_5_GPIO_Port GPIOB
#define INPUT_1_6_Pin GPIO_PIN_4
#define INPUT_1_6_GPIO_Port GPIOB
#define CAN2_STB_Pin GPIO_PIN_7
#define CAN2_STB_GPIO_Port GPIOB
#define CAN1_STB_Pin GPIO_PIN_0
#define CAN1_STB_GPIO_Port GPIOE
#define IN14_Pin GPIO_PIN_1
#define IN14_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
