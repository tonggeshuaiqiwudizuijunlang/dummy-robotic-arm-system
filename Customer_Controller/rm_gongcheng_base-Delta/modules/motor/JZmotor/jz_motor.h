/**
 * @file jz_motor.h
 * @author Jiaxing He
 * @brief 机致电机h文件
 * @version 0.1
 * @date 2024-11-01
 */
#ifndef JZ_MOTOR_H
#define JZ_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define JZ_MOTOR_CNT 4

#define JZ_P_MIN  (-12.5f)
#define JZ_P_MAX  12.5f
#define JZ_V_MIN  (-58.639f)
#define JZ_V_MAX  58.639f
#define JZ_T_MIN  (-6.0f)
#define JZ_T_MAX   6.0f

#define I_MIN -2000
#define I_MAX 2000
#define T_MIN -18.0f // N·m
#define T_MAX 18.0f
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_JZ (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_JZ 0.00512f  // 电流设定值转换成扭矩的系数，这里对应的是16T

typedef enum
{
    JZ_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    JZ_CMD_RESET_MODE = 0xfd,   // 停止电机，对外无力
    JZ_CMD_T_MODE = 0xf9,       // 切换为力矩控制模式
    JZ_CMD_ST_MODE = 0xfa,      // 切换为速度力矩控制模式
    JZ_CMD_AST_MODE = 0xfb,     // 切换为位置速度力矩三闭环模式
    JZ_CMD_ZERO_POSITION = 0xfe // 设定电机当前位置为机械零位,同时清除多圈计数,需要在电机未使能的状态下执行,此位置掉电不保存
}JZMotor_Mode_e;


/* JZ电机CAN反馈信息*/
typedef struct
{
    uint8_t id;
    float last_position;
    float position;
    float angle_single_round; // 单圈角度
    int16_t velocity;
    int16_t real_current;
    // uint16_t last_ecd;        // 上一次读取的编码器值
    // uint16_t ecd;             // 当前编码器值
    // float angle_single_round; // 单圈角度
    // float speed_rads;         // speed rad/s
    // int16_t real_current;     // 实际电流

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} JZ_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}JZMotor_Send_s;

typedef struct
{
    JZ_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    float dt;
} JZMotorInstance;

JZMotorInstance *JZMotorInit(Motor_Init_Config_s *config);
void JZMotorStop(JZMotorInstance *motor);
void JZMotorEnable(JZMotorInstance *motor);
void JZMotorOuterLoop(JZMotorInstance *motor, Closeloop_Type_e outer_loop);
void JZMotorCaliEncoder(JZMotorInstance *motor);
void JZMotorSetRef(JZMotorInstance *motor, float ref);
void JZMotorControl();

#endif // !JZ_MOTOR_H
