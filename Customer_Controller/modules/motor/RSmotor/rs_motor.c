#include "rs_motor.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef RAD_2_DEGREE
#define RAD_2_DEGREE 57.2957795f
#endif

static uint8_t idx = 0;
static RSMotorInstance *rs_motor_instance[RS_MOTOR_CNT] = {NULL};

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1U << bits) - 1U)) / span);
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x) * span / ((float)((1U << bits) - 1U)) + offset;
}

static void DecodeRSMotor(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    RSMotorInstance *motor = (RSMotorInstance *)motor_can->id;
    if (motor == NULL)
        return;

    RS_Motor_Measure_s *measure = &(motor->measure);
    if (motor->daemon != NULL)
        DaemonReload(motor->daemon);

    measure->last_position = measure->position;
    measure->id = rxbuff[0];

    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, RS_P_MIN, RS_P_MAX, 16) * -1.0f;

    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, RS_V_MIN, RS_V_MAX, 12) * RAD_2_DEGREE;

    tmp = (uint16_t)(((rxbuff[4] & 0x0F) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, RS_T_MIN, RS_T_MAX, 12);

    measure->temperature = (float)((rxbuff[6] << 8) | rxbuff[7]) * 0.1f;
}

static void RSMotorSetMode(RSMotor_Mode_e cmd, RSMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    memset(motor->motor_can_instance->tx_buff, 0xff, 7);
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instance, motor->motor_can_instance->tx_id, motor->motor_can_instance->tx_buff, 8);
}

void RSMotorSetMIT(RSMotorInstance *motor)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    memset(motor->motor_can_instance->tx_buff, 0xff, 6);
    motor->motor_can_instance->tx_buff[6] = 0x02;
    motor->motor_can_instance->tx_buff[7] = 0xFD;
    CANTransmit(motor->motor_can_instance, motor->motor_can_instance->tx_id, motor->motor_can_instance->tx_buff, 8);
}

void RSMotorCaliEncoder(RSMotorInstance *motor)
{
    RSMotorSetMode(RS_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

static void RSMotorLostCallback(void *motor_ptr)
{
    RSMotorInstance *motor = (RSMotorInstance *)motor_ptr;
    if (motor != NULL)
        RSMotorSetMode(RS_CMD_MOTOR_MODE, motor);
}

void RSMotorChangeCanID(RSMotorInstance *motor, uint16_t can_id)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;
    memset(motor->motor_can_instance->tx_buff, 0xff, 6);
    motor->motor_can_instance->tx_buff[6] = (uint8_t)can_id;
    motor->motor_can_instance->tx_buff[7] = 0xfa;
    CANTransmit(motor->motor_can_instance, motor->motor_can_instance->tx_id, motor->motor_can_instance->tx_buff, 8);
}

void RSMotorChangeMasterCanID(RSMotorInstance *motor, uint16_t can_id)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;
    memset(motor->motor_can_instance->tx_buff, 0xff, 6);
    motor->motor_can_instance->tx_buff[6] = (uint8_t)can_id;
    motor->motor_can_instance->tx_buff[7] = 0x01;
    CANTransmit(motor->motor_can_instance, motor->motor_can_instance->tx_id, motor->motor_can_instance->tx_buff, 8);
}

RSMotorInstance *RSMotorInit(Motor_Init_Config_s *config)
{
    if (config == NULL || idx >= RS_MOTOR_CNT)
        return NULL;

    RSMotorInstance *instance = (RSMotorInstance *)malloc(sizeof(RSMotorInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(RSMotorInstance));

    instance->motor_type = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;

    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DecodeRSMotor;
    config->can_init_config.id = instance;
    if (config->can_init_config.rx_id == 0U)
        config->can_init_config.rx_id = config->can_init_config.tx_id;
    instance->motor_can_instance = CANRegister(&config->can_init_config);
    if (instance->motor_can_instance == NULL)
    {
        free(instance);
        return NULL;
    }

    Daemon_Init_Config_s conf = {
        .callback = RSMotorLostCallback,
        .owner_id = instance,
        .reload_count = 10,
    };
    instance->daemon = DaemonRegister(&conf);

    RSMotorEnable(instance);
    RSMotorSetMode(RS_CMD_MOTOR_MODE, instance);
    DWT_Delay(0.1);

    rs_motor_instance[idx++] = instance;
    return instance;
}

void RSMotorSetRef(RSMotorInstance *motor, float ref)
{
    if (motor != NULL)
        motor->motor_controller.pid_ref = ref;
}

void RSMotorEnable(RSMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_ENALBED;
}

void RSMotorMITSet(RSMotorInstance *motor, float pos, float vel, float kp, float kd, float torque)
{
    if (motor == NULL)
        return;
    motor->MIT_position = pos;
    motor->MIT_velocity = vel;
    motor->MIT_kp = kp;
    motor->MIT_kd = kd;
    motor->MIT_torque = torque;
}

void RSMotorStop(RSMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_STOP;
}

void RSMotorOuterLoop(RSMotorInstance *motor, Closeloop_Type_e type)
{
    if (motor != NULL)
        motor->motor_settings.outer_loop_type = type;
}

static void RSMotorControlOne(RSMotorInstance *motor)
{
    float pid_measure;
    float pid_ref = motor->motor_controller.pid_ref;
    float output = pid_ref;
    RS_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    CANInstance *motor_can = motor->motor_can_instance;

    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        pid_ref *= -1.0f;
        output *= -1.0f;
    }

    if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
    {
        pid_measure = (setting->angle_feedback_source == OTHER_FEED && motor->motor_controller.other_angle_feedback_ptr != NULL) ?
                          *motor->motor_controller.other_angle_feedback_ptr :
                          measure->position;
        pid_ref = PIDCalculate(&motor->motor_controller.angle_PID, pid_measure, pid_ref);
        output = pid_ref;
    }

    if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && motor->motor_controller.speed_feedforward_ptr != NULL)
            pid_ref += *motor->motor_controller.speed_feedforward_ptr;

        pid_measure = (setting->speed_feedback_source == OTHER_FEED && motor->motor_controller.other_speed_feedback_ptr != NULL) ?
                          *motor->motor_controller.other_speed_feedback_ptr :
                          measure->velocity;
        pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure, pid_ref);
        output = pid_ref;
    }

    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) && motor->motor_controller.current_feedforward_ptr != NULL)
        pid_ref += *motor->motor_controller.current_feedforward_ptr;
    if (setting->close_loop_type & CURRENT_LOOP)
    {
        pid_ref = PIDCalculate(&motor->motor_controller.current_PID, measure->torque, pid_ref);
        output = pid_ref;
    }

    LIMIT_MIN_MAX(output, RS_T_MIN, RS_T_MAX);
    uint16_t tmp = float_to_uint(output, RS_T_MIN, RS_T_MAX, 12);
    if (motor->stop_flag == MOTOR_STOP)
        tmp = float_to_uint(0.0f, RS_T_MIN, RS_T_MAX, 12);

    memset(motor_can->tx_buff, 0, 8);
    motor_can->tx_buff[6] = (tmp >> 8) & 0x0F;
    motor_can->tx_buff[7] = tmp & 0xff;

    CANTransmit(motor_can, motor_can->tx_id, motor_can->tx_buff, 8);
}

void RSMotorControl(void)
{
    for (size_t i = 0; i < idx; i++)
    {
        RSMotorControlOne(rs_motor_instance[i]);
    }
}

void RSMotorMITControl(RSMotorInstance *motor, float position, float velocity, float kp, float kd, float torque)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    uint8_t *tx_buff = motor->motor_can_instance->tx_buff;

    LIMIT_MIN_MAX(position, RS_P_MIN, RS_P_MAX);
    LIMIT_MIN_MAX(velocity, RS_V_MIN, RS_V_MAX);
    LIMIT_MIN_MAX(kp, 0, RS_KP_MAX);
    LIMIT_MIN_MAX(kd, 0, RS_KD_MAX);
    LIMIT_MIN_MAX(torque, RS_T_MIN, RS_T_MAX);

    uint16_t position_uint = float_to_uint(position, RS_P_MIN, RS_P_MAX, 16);
    uint16_t velocity_uint = float_to_uint(velocity, RS_V_MIN, RS_V_MAX, 12);
    uint16_t kp_uint = float_to_uint(kp, 0, RS_KP_MAX, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, RS_KD_MAX, 12);
    uint16_t torque_uint = float_to_uint(torque, RS_T_MIN, RS_T_MAX, 12);

    tx_buff[0] = (position_uint >> 8) & 0xFF;
    tx_buff[1] = position_uint & 0xFF;
    tx_buff[2] = (velocity_uint >> 4) & 0xFF;
    tx_buff[3] = ((velocity_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    tx_buff[4] = kp_uint & 0xFF;
    tx_buff[5] = (kd_uint >> 4) & 0xFF;
    tx_buff[6] = ((kd_uint & 0x0F) << 4) | ((torque_uint >> 8) & 0x0F);
    tx_buff[7] = torque_uint & 0xFF;

    CANTransmit(motor->motor_can_instance, motor->motor_can_instance->tx_id, tx_buff, 8);
}
