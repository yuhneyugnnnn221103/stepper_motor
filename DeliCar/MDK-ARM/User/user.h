#ifndef __USER_H_
#define __USER_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "uart.h"
#include "socket.h"
#include "timer.h"
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
/* USER CODE END Includes */
#define TCP_SOCKET     0
#define TCP_PORT       5000
#define TCP_BUF_SIZE   128
/* USER CODE BEGIN Extern */
extern SPI_HandleTypeDef hspi1;
extern int sock_stat;
extern int8_t flag_not_receive;
extern uint32_t last_data_tick;
extern int flag_ethernet_error;
extern int time_process;
extern int flag_task;
/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void RS485_1_Printf(const char* fmt, ...);
void RS485_2_Printf(const char* fmt, ...);
void W5500_Reset(void);
void W5500_Select(void);
void W5500_Unselect(void);
void W5500_ReadBuff(uint8_t* buff, uint16_t len);
void W5500_WriteBuff(uint8_t* buff, uint16_t len);
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);
void W5500_Init_StaticIP(void);
void TCP_Server_Init(void);
void TCP_Server_Poll(void);
void Ethernet_Error (int timeout_milisecond);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _USER_H_*/