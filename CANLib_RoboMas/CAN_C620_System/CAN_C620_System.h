/*
 * CAN_Main_RoboMas.h
 *
 *  Created on: 7 8, 2023
 *      Author: Emile
 */

#ifndef CAN_C620_SYSTEM_H_
#define CAN_C620_SYSTEM_H_


#include "main.h"
#include "CAN_C620_Def.h"

#define CAN_TXBUFFER_SIZE    (512)


// Mailbox2を使う
void C620_WhenTxMailboxCompleteCallbackCalled(CAN_HandleTypeDef *phcan);

void C620_WhenTxMailboxAbortCallbackCalled(CAN_HandleTypeDef *phcan);

void C620_WhenCANRxFifo1MsgPending(CAN_HandleTypeDef *phcan);

HAL_StatusTypeDef C620_SendBytes(CAN_HandleTypeDef *phcan, uint32_t StdId, uint8_t *bytes, uint32_t size);

void Init_C620_CAN_System(CAN_HandleTypeDef *phcan);

C620_FeedbackData Get_C620_FeedbackData(C620_DeviceInfo *device_info);


#endif