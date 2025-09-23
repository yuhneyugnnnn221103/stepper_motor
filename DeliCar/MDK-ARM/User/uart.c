#include "uart.h"

/* Private functions ---------------------------------------------------------*/
/* USER CODE PRIVATE FUNCTIONS ' DEFINITIONS */
void PI_CM4_Read(void)
{

}
/********************************************************************************/
void PI_CM4_Send(void)
{

}
/********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) // check PI_CM4_Command
	{
		// HAL_UART_Receive_DMA(&huart2,uReceive_data,MAXIMUM_BYTE_RECEIVED);
		
	}
}