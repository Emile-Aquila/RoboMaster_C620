//
// Created by emile on 23/07/19.
//

#ifndef C620_CONTROL_H
#define C620_CONTROL_H

#include "stdint.h"

typedef struct {
    //gains
    float kp;
    float ki;
    float kd;
    float kff;

    //values
    float _integral;
    float _prev_value;
} C620_PID_StructTypedef;


void C620_PID_Ctrl_init(C620_PID_StructTypedef *params);

float C620_PID_Ctrl(C620_PID_StructTypedef *params, float value_diff, float target_value, float update_freq);


#endif //C620_CONTROL_H
