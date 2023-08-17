//
// Created by emile on 23/07/19.
//


#include "C620_Control.h"

void C620_PID_Ctrl_init(C620_PID_StructTypedef *params) {
    params->_integral = 0.0f;
    params->_prev_value = 0.0f;
}

float C620_PID_Ctrl(C620_PID_StructTypedef *params, float value_diff, float target_value, float update_freq) {
    params->_integral += (value_diff + (params->_prev_value)) / 2.0f / update_freq; // 積分(台形近似)
    float diff = (value_diff - params->_prev_value);  // 差分
    params->_prev_value = value_diff;
    return (value_diff * params->kp + params->_integral * params->ki + diff * params->kd + target_value * params->kff);
}

