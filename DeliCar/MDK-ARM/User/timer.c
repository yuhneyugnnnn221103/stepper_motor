#include "timer.h"
#include "define.h"

extern Stepper Left; //id = 0
extern Stepper Right; //id = 1

uint32_t start = 0;
uint8_t get = 0;
uint8_t wait = 0;

uint8_t get1 = 0;
uint8_t get2 = 0;

uint8_t ena_flag = 0;

volatile uint8_t alarm_flag = 0;
volatile uint8_t feedback_cnt = 0;
volatile uint8_t feedback_flag = 0;
/* Private functions ---------------------------------------------------------*/
/* USER CODE PRIVATE FUNCTIONS ' DEFINITIONS */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)   /* 1ms */
	{
		
		/* SEND DISTANT FEEDBACK PER 50MS */
		feedback_cnt++;
		if(feedback_cnt == 50) {
			feedback_cnt = 0;
			feedback_flag = 1;
		}
		/* SEND DISTANT FEEDBACK PER 50MS */
		
		/* INPUT B */
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == 0 && wait == 0) {
			wait = 1;
			Stepper_Emergency_Stop(&Right);
			Stepper_Emergency_Stop(&Left);
			HAL_GPIO_TogglePin(MOTOR_1_ENA_Port, MOTOR_1_ENA_PIN);
			HAL_GPIO_TogglePin(MOTOR_2_ENA_Port, MOTOR_2_ENA_PIN);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
			start = HAL_GetTick();
			}
		if(HAL_GetTick()-start >= 1000 && wait == 1){
			wait = 0;
		}
		/* INPUT B */
		
		/* Driver Alarm Check */
		get1 = HAL_GPIO_ReadPin(MOTOR_1_ALARM_Port, MOTOR_1_ALARM_PIN);
		get2 = HAL_GPIO_ReadPin(MOTOR_2_ALARM_Port, MOTOR_2_ALARM_PIN);
		if(get1 == 0 || get2 == 0) {
			Stepper_Stop(&Right);
			Stepper_Stop(&Left);
			alarm_flag = 1;
			
			//char response[] = "ALARM\r\n";
			//send(TCP_SOCKET, (uint8_t*)response, strlen(response));
		}
		else {
			alarm_flag = 0;
		}
		/* Driver Alarm Check */
		
		
		cnt_overflow++;
		/*
		if(0)
		{
			//test CAN//
//			CAN_Send(&hcan1,TxData1,sizeof(TxData1));
//			memset(TxData1,one_unit,sizeof(TxData1));
//			CAN_Send(&hcan2,TxData2,sizeof(TxData2));
//			memset(TxData2,one_unit,sizeof(TxData2));
//			one_unit ++;
		}*/
			
	}
		
		
	if(htim->Instance == htim4.Instance)   /* 10us */
	{
		get = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
		if(get == 0) {
			Stepper_Stop(&Right);
			Stepper_Stop(&Left);
		}
		
			if(Left.state == stRUN)
			{
				Left.iCn_cnt++;
				if(Left.iCn_cnt >= Left.iCn)
				{
					Left.iCn_cnt = 0;
					Stepper_Control(&Left);
				}
			}
			if(Right.state == stRUN)
			{
				Right.iCn_cnt++;
				if(Right.iCn_cnt >= Right.iCn)
				{
					Right.iCn_cnt = 0;
					Stepper_Control(&Right);
				}
			}
	}
}
uint32_t micros(void)
{
	return cnt_overflow*1000 + __HAL_TIM_GET_COUNTER(&htim2);
}

void delayMicros(uint32_t microsecond) 
{
    uint32_t start = micros(); 
    while((micros() - start) < microsecond); 
}
/********************************************************************************/
