#ifndef __UART_H_
#define __UART_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdlib.h>
/* USER CODE END Includes */
#define MAXIMUM_BYTE_RECEIVED 24
#define MAXIMUM_BYTE_SENT 8
/* USER CODE BEGIN Extern */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern uint8_t uReceive_data[MAXIMUM_BYTE_RECEIVED];
extern uint8_t uTransmit_data[MAXIMUM_BYTE_SENT];
extern uint32_t cnt_overflow;

/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void PI_CM4_Read(void);
void PI_CM4_Send(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _UART_H_*/