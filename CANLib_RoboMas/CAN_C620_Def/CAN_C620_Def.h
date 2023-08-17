//
// Created by emile on 23/07/13.
//

#ifndef _CAN_C620_DEF_H
#define _CAN_C620_DEF_H

#include "stdint.h"
#include "C620_Control.h"

typedef enum {
    C620_CTRL_POS = 0,
    C620_CTRL_VEL = 1,
    C620_CTRL_CURRENT = 2
} C620_CTRL_TYPE;  // C620の制御タイプ


typedef enum {
    C620_ACCEL_LIMIT_ENABLE = 0,
    C620_ACCEL_LIMIT_DISABLE = 1
} C620_ACCEL_LIMIT;  // 制限


typedef enum {
    C620_USE_OFFSET_POS_DISABLE = 0,
    C620_USE_OFFSET_POS_ENABLE = 1
} C620_USE_OFFSET_POS;  // M3508自体のencoderのoffset処理を行うか


typedef struct {
    C620_PID_StructTypedef pid;
    C620_CTRL_TYPE ctrl_type;
    C620_USE_OFFSET_POS use_internal_offset;
    C620_ACCEL_LIMIT accel_limit;
    float accel_limit_size;
    float _target_value;
    uint8_t _enable_flag;
} C620_Ctrl_StructTypedef;


typedef struct C620_DeviceInfo {
    uint8_t device_id;
    C620_Ctrl_StructTypedef ctrl_param;
} C620_DeviceInfo;


typedef struct C620_FeedbackData {
    uint8_t device_id;
    uint8_t get_flag;
    float position;
    float velocity;
    float current;
} C620_FeedbackData;


typedef struct c620_feedback_data_raw {
    uint8_t _get_counter; // dataを受け取った回数 (offset計算用, max:128)
    int64_t _rot_num;  //回転数
    uint16_t pos;
    uint16_t _internal_offset_pos;  // encoderの初期位置自体のoffset
    uint16_t pos_pre;

    int16_t vel;
    int16_t cur;
} c620_feedback_data_raw;


#endif //ROBOMASTER_M3508_TEST_CAN_C620_DEF_H
