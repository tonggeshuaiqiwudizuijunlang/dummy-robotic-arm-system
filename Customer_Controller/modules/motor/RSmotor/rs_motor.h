/**
 * @file rs_motor.h
 * @brief 灵足 RS 电机驱动，适配当前 RA6M5 CAN BSP
 */
#ifndef RS_MOTOR_H
#define RS_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define RS_MOTOR_CNT 4

#define RS_P_MIN  (-12.57f)
#define RS_P_MAX  12.57f
#define RS_V_MIN  (-50.0f)
#define RS_V_MAX  50.0f
#define RS_T_MIN  (-5.5f)
#define RS_T_MAX  5.5f

#define RS_KP_MAX 100.0f
#define RS_KD_MAX 5.0f

typedef enum {
    RS_CMD_MOTOR_MODE = 0xfc,
    RS_CMD_RESET_MODE = 0xfd,
    RS_CMD_ZERO_POSITION = 0xfe,
    RS_CMD_CLEAR_ERROR = 0xfb,
} RSMotor_Mode_e;

typedef struct
{
    uint8_t id;
    float last_position;
    float position;
    float velocity;
    float torque;
    float temperature;

    float angle_single_round;
    float total_angle;
    int32_t total_round;
} RS_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} RSMotor_Send_s;

typedef struct
{
    RS_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;

    CANInstance *motor_can_instance;
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;

    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;

    float MIT_position;
    float MIT_velocity;
    float MIT_kp;
    float MIT_kd;
    float MIT_torque;
} RSMotorInstance;

RSMotorInstance *RSMotorInit(Motor_Init_Config_s *config);
void RSMotorSetRef(RSMotorInstance *motor, float ref);
void RSMotorEnable(RSMotorInstance *motor);
void RSMotorStop(RSMotorInstance *motor);
void RSMotorOuterLoop(RSMotorInstance *motor, Closeloop_Type_e type);
void RSMotorCaliEncoder(RSMotorInstance *motor);
void RSMotorChangeCanID(RSMotorInstance *motor, uint16_t can_id);
void RSMotorChangeMasterCanID(RSMotorInstance *motor, uint16_t can_id);
void RSMotorControl(void);
void RSMotorSetMIT(RSMotorInstance *motor);
void RSMotorMITSet(RSMotorInstance *motor, float pos, float vel, float kp, float kd, float torque);
void RSMotorMITControl(RSMotorInstance *motor, float position, float velocity, float kp, float kd, float torque);

#endif // RS_MOTOR_H
