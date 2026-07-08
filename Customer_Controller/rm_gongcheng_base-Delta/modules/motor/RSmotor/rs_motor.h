/**
 * @file rs_motor.h
 * @author yjf 
 * @brief 灵足电机h文件
 * @version 0.1
 * @date 2025-12-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef RS_MOTOR_H
#define RS_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define RS_MOTOR_CNT 4

#define RS_P_MIN  (-12.57f) //  rad (= 2 * pi)
#define RS_P_MAX  12.57f
#define RS_V_MIN  (-50.0f)  //  rad/s
#define RS_V_MAX  50.0f
#define RS_T_MIN  (-5.5f)   //  N·m
#define RS_T_MAX  5.5f

#define RS_KP_MAX 100.0f
#define RS_KD_MAX 5.0f

typedef enum { 
    RS_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    RS_CMD_RESET_MODE = 0xfd,   // 停止电机，对外无力
    RS_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    RS_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} RSMotor_Mode_e;
 /* RS电机CAN反馈信息*/
typedef struct
{
    uint8_t id;                 //电机id
    float last_position;
    float position;            //单圈角度 单位度
    float velocity;            //角速度   单位rad/s
    float torque;
    float temperature;

    float angle_single_round; // 单圈角度
    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} RS_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}RSMotor_Send_s;

typedef struct
{
    RS_Motor_Measure_s measure;            // 电机测量值
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

    float MIT_position;;
    float MIT_velocity;
    float MIT_kp;
    float MIT_kd;
    float MIT_torque;
} RSMotorInstance;

/**
 * @brief 初始化RS电机
 * 
 * @param config 电机初始化配置
 * @return RSMotorInstance* 电机实例指针
 */
RSMotorInstance *RSMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置电机参考值
 * 
 * @param motor 电机实例
 * @param ref 参考值
 */
void RSMotorSetRef(RSMotorInstance *motor, float ref);

/**
 * @brief 启用电机
 * 
 * @param motor 电机实例
 */
void RSMotorEnable(RSMotorInstance *motor);

/**
 * @brief 停止电机
 * 
 * @param motor 电机实例
 */
void RSMotorStop(RSMotorInstance *motor);

/**
 * @brief 设置电机外环类型
 * 
 * @param motor 电机实例
 * @param type 闭环类型
 */
void RSMotorOuterLoop(RSMotorInstance *motor, Closeloop_Type_e type);

/**
 * @brief 将当前编码器位置设为零位
 * 
 * @param motor 电机实例
 */
void RSMotorCaliEncoder(RSMotorInstance *motor);

/**
 * @brief 修改电机CAN ID
 * 
 * @param motor 电机实例
 * @param can_id 新的电机CAN ID
 */
void RSMotorChangeCanID(RSMotorInstance *motor, uint16_t can_id);

/**
 * @brief 修改主机CAN ID
 * 
 * @param motor 电机实例
 * @param can_id 新的主机CAN ID
 */
void RSMotorChangeMasterCanID(RSMotorInstance *motor, uint16_t can_id);

/**
 * @brief 初始化RS电机控制任务
 */
void RSMotorControlInit(void);
void RSMotorSetMIT(RSMotorInstance *motor);
void RSMotorMITSet(RSMotorInstance *motor, float pos, float vel, float kp, float kd, float torque);
#endif // !RS_MOTOR_H