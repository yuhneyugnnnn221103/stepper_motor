#include "can.h"

/* Private functions ---------------------------------------------------------*/
/* USER CODE PRIVATE FUNCTIONS ' DEFINITIONS */
void CAN_IC_Enable(CAN_HandleTypeDef *hcan)
{
		if (hcan->Instance == CAN1)
		{
				HAL_GPIO_WritePin(CAN1_STB_GPIO_Port, CAN1_STB_Pin, 0);
		}
		else if (hcan->Instance == CAN2)
		{
				HAL_GPIO_WritePin(CAN2_STB_GPIO_Port, CAN2_STB_Pin, 0);
		}	
}
void CAN_IC_Disable(CAN_HandleTypeDef *hcan)
{
		if (hcan->Instance == CAN1)
		{
				HAL_GPIO_WritePin(CAN1_STB_GPIO_Port, CAN1_STB_Pin, 1);
		}
		else if (hcan->Instance == CAN2)
		{
				HAL_GPIO_WritePin(CAN2_STB_GPIO_Port, CAN2_STB_Pin, 1);
		}		
}
void CAN_Tx_Header_Init(CAN_TxHeaderTypeDef *TxHeader, int ID, int data_length)
{
		TxHeader->DLC=data_length;
		TxHeader->ExtId=0;
		TxHeader->IDE=CAN_ID_STD;
		TxHeader->RTR=CAN_RTR_DATA;
		TxHeader->StdId=ID;
		TxHeader->TransmitGlobalTime=DISABLE;
}

void CAN_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canFilterConfig, int FilterBank, int SlaveStartFilterBank, int FilterIdHigh, int FilterMaskIdHigh)
{
		canFilterConfig->FilterActivation = CAN_FILTER_ENABLE;
		canFilterConfig->FilterBank = FilterBank; // S? th? t? c?a b? l?c
		canFilterConfig->FilterFIFOAssignment = CAN_RX_FIFO0;
		canFilterConfig->FilterIdHigh = FilterIdHigh<<5;
		canFilterConfig->FilterIdLow = 0x0000;  // Ph?n th?p c?a ID c?n nh?n
		canFilterConfig->FilterMaskIdHigh =FilterMaskIdHigh<<5;
		canFilterConfig->FilterMaskIdLow = 0x0000;  // Ph?n th?p c?a mask
		canFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;  // Ch? d? b? l?c: ID + Mask
		canFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;  // Ki?u ID: 32-bit
		canFilterConfig->SlaveStartFilterBank = SlaveStartFilterBank;	
		
		HAL_CAN_ConfigFilter(hcan, canFilterConfig);
}
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,int ID_Tx, int data_lenth, CAN_FilterTypeDef *canFilterConfig,int FilterBank, int SlaveStartFilterBank, int FilterIdHigh, int FilterMaskIdHigh)
{
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		CAN_Tx_Header_Init(TxHeader,ID_Tx,data_lenth);
		CAN_Filter_Config(hcan,canFilterConfig,FilterBank,SlaveStartFilterBank,FilterIdHigh,FilterMaskIdHigh);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		if (hcan->Instance == CAN1)
		{
				HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1, RxData1);
		}
		else if (hcan->Instance == CAN2)
		{
				HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2);
		}	
}
void CAN_Send(CAN_HandleTypeDef *hcan, uint8_t *TxData, uint8_t size)
{
		if (hcan->Instance == CAN1)
		{
					for(int i = 0; i< size;i++)
					{
							TxData1[i] = TxData[i];
					}
					HAL_CAN_AddTxMessage(&hcan1,&TxHeader1,&TxData1[0],&TxMailbox1[0]);
		}
		else if (hcan->Instance == CAN2)
		{
					for(int i = 0; i< size;i++)
					{
							TxData2[i] = TxData[i];
					}
					HAL_CAN_AddTxMessage(&hcan2,&TxHeader2,&TxData2[0],&TxMailbox2[0]);
		}
}
/********************************************************************************/