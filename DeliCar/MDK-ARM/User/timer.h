#ifndef __TIMER_H_
#define __TIMER_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "uart.h"
#include "can.h"
#include "socket.h"
#include "io.h"
#include "Stepper.h"
#include <string.h>
/* USER CODE END Includes */
#define MAXIMUM_BYTE_RECEIVED 24
#define MAXIMUM_BYTE_SENT 8
/* USER CODE BEGIN Extern */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern I2C_HandleTypeDef hi2c2;
extern void RS485_1_Printf(const char* fmt, ...);
extern void RS485_2_Printf(const char* fmt, ...);
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern DMA_HandleTypeDef hdma_usart2_tx;
extern uint8_t uReceive_data[MAXIMUM_BYTE_RECEIVED];
extern uint8_t uTransmit_data[MAXIMUM_BYTE_SENT];
extern uint32_t cnt_overflow;


extern CAN_TxHeaderTypeDef TxHeader1;
extern CAN_RxHeaderTypeDef RxHeader1;
extern uint32_t TxMailbox1[3];
extern uint8_t TxData1[8];
extern uint8_t RxData1[8];

extern CAN_TxHeaderTypeDef TxHeader2;
extern CAN_RxHeaderTypeDef RxHeader2;
extern uint32_t TxMailbox2[3];
extern uint8_t TxData2[8];
extern uint8_t RxData2[8];

extern uint8_t can_test[8];
extern uint8_t one_unit;
/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint32_t micros();
void delayMicros(uint32_t microsecond);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _TIMER_H_*/