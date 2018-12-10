/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "gpio.h"
#include "chassis.h"
#include "imu.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CAN_FilterConfTypeDef can1_filetr;
CAN_FilterConfTypeDef can2_filetr;

CanTxMsgTypeDef Tx1Message;
CanRxMsgTypeDef	Rx1Message;

CanTxMsgTypeDef Tx2Message;
CanRxMsgTypeDef	Rx2Message;

extern _chassis chassisPara;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_9TQ;
  hcan1.Init.BS2 = CAN_BS2_4TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);
	
	can1_filetr.FilterIdHigh=0X0000;     //32λID
  can1_filetr.FilterIdLow=0X0000;
	can1_filetr.FilterMaskIdHigh=0X0000; //32λMASK
	can1_filetr.FilterMaskIdLow=0X0000;  
	can1_filetr.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
	can1_filetr.FilterNumber=0;          //������0
	can1_filetr.FilterMode=CAN_FILTERMODE_IDMASK;
	can1_filetr.FilterScale=CAN_FILTERSCALE_32BIT;
	can1_filetr.FilterActivation=ENABLE; //�����˲���0
	can1_filetr.BankNumber=14;	
	
	HAL_CAN_ConfigFilter(&hcan1,&can1_filetr);
	
	hcan1.pTxMsg = &Tx1Message;
	hcan1.pRxMsg = &Rx1Message;
	
	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);
	
}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_9TQ;
  hcan2.Init.BS2 = CAN_BS2_4TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);
	
	can2_filetr.FilterIdHigh=0X0000;     //32λID
  can2_filetr.FilterIdLow=0X0000;
	can2_filetr.FilterMaskIdHigh=0X0000; //32λMASK
	can2_filetr.FilterMaskIdLow=0X0000;  
	can2_filetr.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
	can2_filetr.FilterNumber=14;          //������0
	can2_filetr.FilterMode=CAN_FILTERMODE_IDMASK;
	can2_filetr.FilterScale=CAN_FILTERSCALE_32BIT;
	can2_filetr.FilterActivation=ENABLE; //�����˲���0
	HAL_CAN_ConfigFilter(&hcan2,&can2_filetr);
	
	hcan2.pTxMsg = &Tx2Message;
	hcan2.pRxMsg = &Rx2Message;
	
	__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_FMP0);
	
}

static int HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
//    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 2);
//    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
//    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 4);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

HAL_StatusTypeDef can_send_msg(uint8_t flag, CAN_HandleTypeDef* hcan, CAN_MessageID _id, int16_t *data){
	
	if(flag){
		
		hcan->pTxMsg->RTR = CAN_RTR_DATA;
		hcan->pTxMsg->IDE = CAN_ID_STD;
		
		switch(_id){
			case CAN_IMU_TxID:{
				hcan->pTxMsg->StdId = CAN_IMU_TxID;
				hcan->pTxMsg->DLC = 8;
				hcan->pTxMsg->Data[0] = (uint8_t)(*(data+0)>>8);
				hcan->pTxMsg->Data[1] = (uint8_t)(*(data+0));
				hcan->pTxMsg->Data[2] = (uint8_t)(*(data+1)>>8);
				hcan->pTxMsg->Data[3] = (uint8_t)(*(data+1));
				hcan->pTxMsg->Data[4] = (uint8_t)(*(data+2)>>8);
				hcan->pTxMsg->Data[5] = (uint8_t)(*(data+2));
				hcan->pTxMsg->Data[6] = (uint8_t)(*(data+3)>>8);
				hcan->pTxMsg->Data[7] = (uint8_t)(*(data+3));
				break;
			}
			case CAN_WHEEL_TxID:{
				hcan->pTxMsg->StdId = CAN_WHEEL_TxID;
				hcan->pTxMsg->DLC = 8;
				hcan->pTxMsg->Data[0] = (uint8_t)(*(data+1)>>8);
				hcan->pTxMsg->Data[1] = (uint8_t)(*(data+1));
				hcan->pTxMsg->Data[2] = (uint8_t)(*(data+0)>>8);
				hcan->pTxMsg->Data[3] = (uint8_t)(*(data+0));
				hcan->pTxMsg->Data[4] = (uint8_t)(*(data+3)>>8);
				hcan->pTxMsg->Data[5] = (uint8_t)(*(data+3));
				hcan->pTxMsg->Data[6] = (uint8_t)(*(data+2)>>8);
				hcan->pTxMsg->Data[7] = (uint8_t)(*(data+2));
				break;
			}
			default :break;
		}
		
		return HAL_CAN_Transmit(hcan,50);
	}
	return NULL;
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{

   switch(hcan->pRxMsg->StdId){
		 case CAN_IMU_RxID:{
			 static float compensate = 0;
			 imu_yaw.yaw = 0.01f *((hcan->pRxMsg->Data[0]<<24) + (hcan->pRxMsg->Data[1]<<16) \
															+ (hcan->pRxMsg->Data[2] <<8) + (hcan->pRxMsg->Data[3]) ) - compensate;
			  if(!imu_yaw.flag){
				 if(imu_yaw.cycle_calibration){
					 if(imu_yaw.yaw !=0) {
						 imu_yaw.cycle_calibration --;
					 }
					 else{
						 imu_yaw.flag = 1;
					 }
					 
				 }
				 else {
					 compensate = imu_yaw.yaw;
					 imu_yaw.flag = 1;
				 }
			 }
			 break ;
		 }
		 case CAN_WHEEL_RxBeginID:{
			 Wheel_Para.feedback.Speed[1]= hcan->pRxMsg->Data[2]*256 + hcan->pRxMsg->Data[3];
			 break;
		 }
		 case CAN_WHEEL_RxBeginID+1:{
			 Wheel_Para.feedback.Speed[0]= hcan->pRxMsg->Data[2]*256 + hcan->pRxMsg->Data[3];
			 break;
		 }
		 case CAN_WHEEL_RxBeginID+2:{
			 Wheel_Para.feedback.Speed[3]= hcan->pRxMsg->Data[2]*256 + hcan->pRxMsg->Data[3];
			 break;
		 }
		 case CAN_WHEEL_RxBeginID+3:{
			 Wheel_Para.feedback.Speed[2]= hcan->pRxMsg->Data[2]*256 + hcan->pRxMsg->Data[3];
			 break;
		 }
		 default :break;
	 }
	 __HAL_CAN_ENABLE_IT(hcan,CAN_IT_FMP0);
}


void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
