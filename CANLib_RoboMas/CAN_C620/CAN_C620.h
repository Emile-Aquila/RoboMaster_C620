//
// Created by emile on 23/07/13.
//

#ifndef CAN_C620_H
#define CAN_C620_H

#include "CAN_C620_Def.h"
#include "CAN_C620_System.h"


void C620_Ctrl_Struct_init(C620_Ctrl_StructTypedef *ctrl_struct);

void C620_Init(C620_DeviceInfo dev_info_array[], uint8_t size);  // C620_Init(dev[], 2)的なのを想定してる.

void C620_SendRequest(C620_DeviceInfo dev_info_array[], uint8_t size, float update_freq_hz, CAN_HandleTypeDef *phcan);


void C620_ChangeControl(C620_DeviceInfo *dev_info, C620_CTRL_TYPE new_ctrl_type);

void C620_SetTarget(C620_DeviceInfo *device_info, float target_value);

void C620_WaitForConnect(C620_DeviceInfo dev_info_array[], uint8_t size);

void C620_ControlEnable(C620_DeviceInfo *dev_info);

void C620_ControlDisable(C620_DeviceInfo *dev_info);


#endif //CAN_C620_H
