#ifndef __IO_H_
#define __IO_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "uart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Extern */
extern void RS485_1_Printf(const char* fmt, ...);
extern void RS485_2_Printf(const char* fmt, ...);
extern int on;
/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void Output_enable(void);
void Output_disable(void);
void Input_test(void);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _IO_H_*/