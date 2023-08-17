//
// Created by emile on 23/07/13.
//

#include "CAN_C620.h"
#include "CAN_C620_Def.h"
#include "CAN_C620_System.h"
#include "math.h"
#include "stdio.h"


float clip_f(float var, float ref) {
    float abs_ref = fabsf(ref);
    return fmaxf(fminf(var, abs_ref), -abs_ref);
}

int16_t c620_current_f2int(float current) {
    return (int16_t) (current * 16384.0f / 20.0f);
}


void C620_Ctrl_Struct_init(C620_Ctrl_StructTypedef *ctrl_struct) {
    ctrl_struct->_target_value = 0.0f;
    ctrl_struct->_enable_flag = 0;
    C620_PID_Ctrl_init(&(ctrl_struct->pid));
}

void C620_Init(C620_DeviceInfo dev_info_array[], uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        C620_Ctrl_Struct_init(&(dev_info_array[i].ctrl_param));
    }
}

void C620_SendRequest(C620_DeviceInfo dev_info_array[], uint8_t size, float update_freq_hz, CAN_HandleTypeDef *phcan) {
    uint8_t data1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t data2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t flag_1 = 0, flag_2 = 0;
    int16_t request_value = 0;
    float diff = 0.0f, t_current = 0.0f;
    C620_FeedbackData fb_data;

    for (uint8_t i = 0; i < size; i++) {
        if (!(dev_info_array[i].ctrl_param._enable_flag))continue;
        fb_data = Get_C620_FeedbackData(&dev_info_array[i]);
        if (dev_info_array[i].device_id == 0) {
            printf("[C620] device_id is not 0\n\r");
            continue;
        }

        if (dev_info_array[i].ctrl_param.ctrl_type == C620_CTRL_CURRENT) {
            t_current = dev_info_array[i].ctrl_param._target_value;
        } else {
            diff = dev_info_array[i].ctrl_param._target_value;
            switch (dev_info_array[i].ctrl_param.ctrl_type) {
                case C620_CTRL_POS:
                    diff -= fb_data.position;
                    break;
                case C620_CTRL_VEL:
                    diff -= fb_data.velocity;
                    break;
                default:
                    diff = 0.0f;
                    break;
            }

            if (dev_info_array[i].ctrl_param.accel_limit == C620_ACCEL_LIMIT_ENABLE) {
                diff = clip_f(diff, dev_info_array[i].ctrl_param.accel_limit_size);
            }
            t_current = C620_PID_Ctrl(&(dev_info_array[i].ctrl_param.pid), diff,
                                      (dev_info_array[i].ctrl_param._target_value), update_freq_hz);
        }
        // 目標値の計算
        request_value = c620_current_f2int(clip_f(t_current, 20.0f));

        // 各モーターの目標値の設定
        if (dev_info_array[i].device_id < 5) {
            flag_1 = 1;
            for (uint8_t j = 0; j < 2; j++) {
                data1[(dev_info_array[i].device_id - 1) * 2 + j] = (request_value >> ((!j) * 8)) & 0b11111111;
            }
        } else if (dev_info_array[i].device_id >= 5) {
            flag_2 = 1;
            for (uint8_t j = 0; j < 2; j++) {
                data2[(dev_info_array[i].device_id - 5) * 2 + j] = (request_value >> ((!j) * 8)) & 0b11111111;
            }
        }
    }
    if (flag_1)C620_SendBytes(phcan, 0x200, (uint8_t *) data1, sizeof(data1));
    if (flag_2)C620_SendBytes(phcan, 0x1FF, (uint8_t *) data2, sizeof(data2));
}

void C620_WaitForConnect(C620_DeviceInfo dev_info_array[], uint8_t size) {
    uint8_t flag = 0;
    printf("[C620] Wait for Connection...\n");
    while (!flag) {
        flag = 1;
        for (uint8_t i = 0; i < size; i++) {
            if (!Get_C620_FeedbackData(&dev_info_array[i]).get_flag) {
                flag = 0;
                break;
            }
        }
    }
    printf("[C620] All Connected!\n");
}

void C620_ChangeControl(C620_DeviceInfo *dev_info, C620_CTRL_TYPE new_ctrl_type) {
    C620_Ctrl_Struct_init(&(dev_info->ctrl_param));
    dev_info->ctrl_param.ctrl_type = new_ctrl_type;
}

void C620_SetTarget(C620_DeviceInfo *device_info, float target_value) {
    device_info->ctrl_param._target_value = target_value;
}

void C620_ControlEnable(C620_DeviceInfo *dev_info) {
    dev_info->ctrl_param._enable_flag = 1;
}

void C620_ControlDisable(C620_DeviceInfo *dev_info) {
    dev_info->ctrl_param._enable_flag = 0;
}



