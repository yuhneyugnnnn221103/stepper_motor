#ifndef __CAN_H_
#define __CAN_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Extern */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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

/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void CAN_Tx_Header_Init(CAN_TxHeaderTypeDef *TxHeader, int ID, int data_length);
void CAN_IC_Enable(CAN_HandleTypeDef *hcan);
void CAN_IC_Disable(CAN_HandleTypeDef *hcan);
void CAN_Filter_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canFilterConfig, int FilterBank, int SlaveStartFilterBank, int FilterIdHigh, int FilterMaskIdHigh);
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader,int ID_Tx, int data_lenth, CAN_FilterTypeDef *canFilterConfig,int FilterBank, int SlaveStartFilterBank, int FilterIdHigh, int FilterMaskIdHigh);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Send(CAN_HandleTypeDef *hcan, uint8_t *TxData, uint8_t size);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _CAN_H_*/