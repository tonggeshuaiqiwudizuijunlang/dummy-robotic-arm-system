/**
 * @file dji_motor.h
 * @brief DJI智能电机驱动，适配当前 RA6M5 CAN BSP
 */
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DJI_MOTOR_CNT 12

#define SPEED_SMOOTH_COEF 0.85f
#define CURRENT_SMOOTH_COEF 0.9f
#define ECD_ANGLE_COEF_DJI 0.043945f

#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f
#endif

typedef struct
{
    uint8_t id;
    uint16_t last_ecd;
    uint16_t ecd;
    float angle_single_round;
    float speed_aps;
    int16_t real_current;
    uint8_t temperature;

    float total_angle;
    int32_t total_round;
} DJI_Motor_Measure_s;

typedef enum
{
    PID_MODE,
    OPENLOOP_MODE,
    CURRENT_MODE,
} Dji_MODE;

typedef struct
{
    DJI_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;
    Dji_MODE dji_mode;

    CANInstance *motor_can_instance;
    uint32_t expected_rx_id;
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;

    DaemonInstance *daemon;
    uint32_t feed_cnt;
    uint32_t recv_count;
    uint32_t last_rx_id;
    float dt;
} DJIMotorInstance;

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void DJIMotorControl(void);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);
void DJiMotorSetMode(DJIMotorInstance *motor, Dji_MODE mode);
uint8_t DJIMotorIsOnline(DJIMotorInstance *motor);
uint32_t DJIMotorGetExpectedRxId(DJIMotorInstance *motor);
uint32_t DJIMotorGetLastRxId(DJIMotorInstance *motor);
uint32_t DJIMotorGetRxCount(DJIMotorInstance *motor);
uint32_t DJIMotorGetLastInitError(void);

#endif // DJI_MOTOR_H
