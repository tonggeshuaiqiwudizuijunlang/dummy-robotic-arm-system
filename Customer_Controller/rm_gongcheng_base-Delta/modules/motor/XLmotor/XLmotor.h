#ifndef __XLMOTOR_H
#define __XLMOTOR_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#define XL_GEAR_RATIO 176.0f // 减速比
#define XL_MOTOR_MX_CNT 2 // 最多允许4个Xl电机使用多电机指令,挂载在一条总线上

#define XL_P_MIN  (-180.0f)//这个是直接从达妙上抄的，后续还要更改
#define XL_P_MAX  180.0f
#define XL_V_MIN  (-15.0f)
#define XL_V_MAX  15.0f
#define XL_T_MIN  (-9.0f)
#define XL_T_MAX   9.0f


/* XL电机CAN反馈信息*/
typedef struct
{
    uint8_t id;
    uint16_t last_position;        
    uint16_t position;
    float omega;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius
    uint16_t torque;
    
} XL_Motor_Measure_s;

typedef struct
{
    float position;
    uint16_t velocity;
    uint16_t torque;

}XL_Motor_Send_s;

enum Enum_XL_Motor_Status
{
    XL_Motor_ControlModes_Position,//位置闭环
    XL_Motor_ControlModes_Speed,//速度闭环
    XL_Motor_ControlModes_Torque,//力矩闭环
    XL_Motor_ControlModes_Position_xiebo,
    XL_Motor_ControlModes_Speed_xiebo,
    XL_Motor_ControlModes_MIT,//MIT控制模式，但是跟达妙的MIT不一样
};

typedef struct
{
    XL_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    XL_Motor_Send_s XL_Motor_Send;

    float *other_angle_feedback_ptr;
    float *other_torque_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    CANInstance *motor_can_instance; // 电机CAN实例

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e motor_status; // 启停标志
    enum Enum_XL_Motor_Status Motor_Driver_Mode;//控制模式

    DaemonInstance* daemon;
    float dt;

    uint16_t feed_cn;

} XLMotorInstance;

void XL_Decode(CANInstance *motor_can);
void XL_ControInit();
void XLMotor_Mode_Set(XLMotorInstance *Motor, enum Enum_XL_Motor_Status Motor_Driver_Mode);
void XLMotorEnable(XLMotorInstance *Motor);
XLMotorInstance *XL_Init(Motor_Init_Config_s *Config);
void XL_Position_Set(XLMotorInstance *Motor, float angle_);
void XLMotorGetError(XLMotorInstance *Motor);
void XLMotorClearError(XLMotorInstance *Motor);
#endif