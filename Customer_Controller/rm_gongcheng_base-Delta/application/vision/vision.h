//
// Created by HeYee on 2024/10/20.
//

#ifndef VISION_H
#define VISION_H

#include "controller.h" 
#include "stdint.h"
#include "robot_def.h"

typedef struct
{
    PIDInstance vision_PID;
    float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
    float fire;
    float time_diff;
} vision_Controller_s;

typedef struct {
    float pid_ref_y;
    float pid_ref_p;
    uint16_t found_flag;
}PIDRefs;

PIDRefs* vision_to_control(float* Pitch,float* Yaw);
void vision_to_init();

#endif //BASIC_FRAMEWORK_VISION_H


