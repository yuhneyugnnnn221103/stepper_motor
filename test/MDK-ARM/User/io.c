#include "io.h"

/* Private functions ---------------------------------------------------------*/
/* USER CODE PRIVATE FUNCTIONS ' DEFINITIONS */
void Output_enable(void)
{
		HAL_GPIO_WritePin(OUTPUT_1_GPIO_Port, OUTPUT_1_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_2_GPIO_Port, OUTPUT_2_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_3_GPIO_Port, OUTPUT_3_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_4_GPIO_Port, OUTPUT_4_Pin, 1);
	  HAL_GPIO_WritePin(OUTPUT_5_GPIO_Port, OUTPUT_5_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_6_GPIO_Port, OUTPUT_6_Pin, 1); //relay
		HAL_GPIO_WritePin(OUTPUT_7_GPIO_Port, OUTPUT_7_Pin, 1); //relay
		HAL_GPIO_WritePin(OUTPUT_8_GPIO_Port, OUTPUT_8_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_9_GPIO_Port, OUTPUT_9_Pin, 1);
		HAL_GPIO_WritePin(OUTPUT_10_GPIO_Port, OUTPUT_10_Pin, 1);
}
void Output_disable(void)
{
		HAL_GPIO_WritePin(OUTPUT_1_GPIO_Port, OUTPUT_1_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_2_GPIO_Port, OUTPUT_2_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_3_GPIO_Port, OUTPUT_3_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_4_GPIO_Port, OUTPUT_4_Pin, 0);
	  HAL_GPIO_WritePin(OUTPUT_5_GPIO_Port, OUTPUT_5_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_6_GPIO_Port, OUTPUT_6_Pin, 0); //relay
		HAL_GPIO_WritePin(OUTPUT_7_GPIO_Port, OUTPUT_7_Pin, 0); //relay
		HAL_GPIO_WritePin(OUTPUT_8_GPIO_Port, OUTPUT_8_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_9_GPIO_Port, OUTPUT_9_Pin, 0);
		HAL_GPIO_WritePin(OUTPUT_10_GPIO_Port, OUTPUT_10_Pin, 0);
}
/********************************************************************************/
void Input_test(void)
{

//		on = !HAL_GPIO_ReadPin(INPUT_1_1_GPIO_Port, INPUT_1_1_Pin);
//		RS485_2_Printf("Input 1_1 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_1_2_GPIO_Port, INPUT_1_2_Pin);
//		RS485_2_Printf("Input 1_2 state: %d \r\n",on);
//		on = HAL_GPIO_ReadPin(INPUT_1_3_GPIO_Port, INPUT_1_3_Pin);
//		RS485_2_Printf("Input 1_3 state: %d \r\n",on);
//		on = HAL_GPIO_ReadPin(INPUT_1_4_GPIO_Port, INPUT_1_4_Pin);
//		RS485_2_Printf("Input 1_4 state: %d \r\n",on);
//		on = HAL_GPIO_ReadPin(INPUT_1_5_GPIO_Port, INPUT_1_5_Pin);
//		RS485_2_Printf("Input 1_5 state: %d \r\n",on);
		on = HAL_GPIO_ReadPin(INPUT_1_6_GPIO_Port, INPUT_1_6_Pin);
		RS485_2_Printf("Input 1_6 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_1_GPIO_Port, INPUT_2_1_Pin);
//		RS485_2_Printf("Input 2_1 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_2_GPIO_Port, INPUT_2_2_Pin);
//		RS485_2_Printf("Input 2_2 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_3_GPIO_Port, INPUT_2_3_Pin);
//		RS485_2_Printf("Input 2_3 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_4_GPIO_Port, INPUT_2_4_Pin);
//		RS485_2_Printf("Input 2_4 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_5_GPIO_Port, INPUT_2_5_Pin);
//		RS485_2_Printf("Input 2_5 state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(INPUT_2_6_GPIO_Port, INPUT_2_6_Pin);
//		RS485_2_Printf("Input 2_6 state: %d \r\n",on);
//		
//		on = !HAL_GPIO_ReadPin(IN11_GPIO_Port, IN11_Pin);
//		RS485_2_Printf("In11  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN12_GPIO_Port, IN12_Pin);
//		RS485_2_Printf("In12  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN13_GPIO_Port, IN13_Pin);
//		RS485_2_Printf("In13  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN14_GPIO_Port, IN14_Pin);
//		RS485_2_Printf("In14  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN21_GPIO_Port, IN21_Pin);
//		RS485_2_Printf("In21  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN22_GPIO_Port, IN22_Pin);
//		RS485_2_Printf("In22  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN23_GPIO_Port, IN23_Pin);
//		RS485_2_Printf("In23  state: %d \r\n",on);
//		on = !HAL_GPIO_ReadPin(IN24_GPIO_Port, IN24_Pin);
//		RS485_2_Printf("In24  state: %d \r\n",on);
}