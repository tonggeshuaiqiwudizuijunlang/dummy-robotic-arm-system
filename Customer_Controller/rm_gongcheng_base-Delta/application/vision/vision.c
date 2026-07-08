//
// Created by HeYee on 2024/10/20.
//
#include "robot_cmd.h"
#include "vision.h"
#include "master_process.h"
#include "seasky_protocol.h"
#include "gimbal.h"
#include "dji_motor.h"
struct Vision_angle_one
{
    float pitch;
    float yaw;
} vision_angle;

PIDRefs* refs_vision;
 
vision_Controller_s *vision_controller;
float turned_pitch;
float turned_yaw;
void vision_to_init()
{  
    vision_angle.pitch = 0;
    vision_angle.yaw = 0;
    vision_controller->vision_PID = (PIDInstance){
            .Kp = 5.0,
            .Ki = 1.0,
            .Kd = 0.1,
            //.IntegralLimit = 10,
            //.Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,  //PID_Integral_Limit
            .MaxOut = 1.5, // 看角度 ..
    };
}

PIDRefs* vision_to_control(float* Pitch,float* Yaw)
{
    vision_angle.pitch = Get_pitch();
    vision_angle.yaw = Get_yaw();
    if(abs(vision_angle.pitch)<1.0 && abs(vision_angle.yaw)<1.0 && abs(vision_angle.pitch)>0 && abs(vision_angle.yaw)>0)
    {
        refs_vision->found_flag = 1;
    }
    else{
         refs_vision->found_flag = 0;
    }
    turned_yaw = *Yaw;
    turned_pitch = *Pitch;
    // refs_vision->pid_ref_y =PID_increment(&(vision_controller->vision_PID), turned_yaw, *Get_yaw());
    // refs_vision->pid_ref_p= PID_increment(&(vision_controller->vision_PID), turned_pitch, *Get_pitch());
    return refs_vision;
}
