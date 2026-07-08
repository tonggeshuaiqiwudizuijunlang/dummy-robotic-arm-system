#include "dji_motor.h"
#include "bsp_dwt.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx = 0;
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL};
static CANInstance sender_assignment[3];
static uint8_t sender_enable_flag[3] = {0};
static uint32_t last_init_error = 0U;

static float DJIMotor_Limit_Output(const DJIMotorInstance *motor, float output)
{
    float max_output = 32767.0f;
    if (motor != NULL && motor->motor_controller.current_PID.MaxOut > 0.0f)
        max_output = motor->motor_controller.current_PID.MaxOut;

    if (max_output > 32767.0f)
        max_output = 32767.0f;
    if (output > max_output)
        return max_output;
    if (output < -max_output)
        return -max_output;
    return output;
}

static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = (uint8_t)(config->tx_id - 1U);
    uint8_t motor_send_num = 0;
    uint8_t motor_grouping = 0;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4U)
        {
            motor_send_num = motor_id;
            motor_grouping = 1U; // 0x200
        }
        else
        {
            motor_send_num = motor_id - 4U;
            motor_grouping = 0U; // 0x1FF
        }
        config->rx_id = 0x200U + motor_id + 1U;
        break;

    case GM6020:
        if (motor_id < 4U)
        {
            motor_send_num = motor_id;
            motor_grouping = 0U; // 0x1FF
        }
        else
        {
            motor_send_num = motor_id - 4U;
            motor_grouping = 2U; // 0x2FF
        }
        config->rx_id = 0x204U + motor_id + 1U;
        break;

    default:
        config->rx_id = config->tx_id;
        break;
    }

    motor->expected_rx_id = config->rx_id;
    sender_enable_flag[motor_grouping] = 1U;
    motor->message_num = motor_send_num;
    motor->sender_group = motor_grouping;
}

static void DecodeDJIMotor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    if (motor == NULL)
        return;
    if (_instance->current_rx_id != motor->expected_rx_id)
        return;

    DJI_Motor_Measure_s *measure = &motor->measure;

    if (motor->daemon != NULL)
        DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    motor->recv_count++;
    motor->last_rx_id = _instance->current_rx_id;

    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (int16_t)((1.0f - CURRENT_SMOOTH_COEF) * (float)measure->real_current +
                                      CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5])));
    measure->temperature = rxbuff[6];

    if ((int32_t)measure->ecd - (int32_t)measure->last_ecd > 4096)
        measure->total_round--;
    else if ((int32_t)measure->ecd - (int32_t)measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = (float)measure->total_round * 360.0f + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    if (motor != NULL)
        DJIMotorStop(motor);
}

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    last_init_error = 0U;

    if (config == NULL || idx >= DJI_MOTOR_CNT)
    {
        last_init_error = 1U;
        return NULL;
    }

    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    if (instance == NULL)
    {
        last_init_error = 2U;
        return NULL;
    }
    memset(instance, 0, sizeof(DJIMotorInstance));

    instance->motor_type = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;

    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    MotorSenderGrouping(instance, &config->can_init_config);

    config->can_init_config.can_module_callback = DecodeDJIMotor;
    config->can_init_config.id = instance;
    config->can_init_config.rx_id = 0U;
    instance->motor_can_instance = CANRegister(&config->can_init_config);
    if (instance->motor_can_instance == NULL)
    {
        last_init_error = 3U;
        free(instance);
        return NULL;
    }

    if (idx == 0U)
    {
        memset(sender_assignment, 0, sizeof(sender_assignment));
        sender_assignment[0].p_ctrl = instance->motor_can_instance->p_ctrl;
        sender_assignment[0].p_cfg = instance->motor_can_instance->p_cfg;
        sender_assignment[0].tx_id = 0x1FFU;
        sender_assignment[1].p_ctrl = instance->motor_can_instance->p_ctrl;
        sender_assignment[1].p_cfg = instance->motor_can_instance->p_cfg;
        sender_assignment[1].tx_id = 0x200U;
        sender_assignment[2].p_ctrl = instance->motor_can_instance->p_ctrl;
        sender_assignment[2].p_cfg = instance->motor_can_instance->p_cfg;
        sender_assignment[2].tx_id = 0x2FFU;
    }

    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2,
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (motor == NULL)
        return;
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    if (motor != NULL)
        motor->stop_flag = MOTOR_ENALBED;
}

void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    if (motor != NULL)
        motor->motor_settings.outer_loop_type = outer_loop;
}

void DJiMotorSetMode(DJIMotorInstance *motor, Dji_MODE mode)
{
    if (motor != NULL)
        motor->dji_mode = mode;
}

void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;
    motor->motor_controller.pid_ref = ref;
}

uint8_t DJIMotorIsOnline(DJIMotorInstance *motor)
{
    if (motor == NULL || motor->daemon == NULL)
        return 0U;
    return DaemonIsOnline(motor->daemon);
}

uint32_t DJIMotorGetExpectedRxId(DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0U;
    return motor->expected_rx_id;
}

uint32_t DJIMotorGetLastRxId(DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0U;
    return motor->last_rx_id;
}

uint32_t DJIMotorGetRxCount(DJIMotorInstance *motor)
{
    if (motor == NULL)
        return 0U;
    return motor->recv_count;
}

uint32_t DJIMotorGetLastInitError(void)
{
    return last_init_error;
}

void DJIMotorControl(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DJIMotorInstance *motor = dji_motor_instance[i];
        Motor_Control_Setting_s *motor_setting = &motor->motor_settings;
        Motor_Controller_s *motor_controller = &motor->motor_controller;
        DJI_Motor_Measure_s *measure = &motor->measure;
        float pid_ref = motor_controller->pid_ref;
        float pid_measure;

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            pid_measure = (motor_setting->angle_feedback_source == OTHER_FEED && motor_controller->other_angle_feedback_ptr != NULL) ?
                              *motor_controller->other_angle_feedback_ptr :
                              measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if ((motor_setting->feedforward_flag & SPEED_FEEDFORWARD) && motor_controller->speed_feedforward_ptr != NULL)
                pid_ref += *motor_controller->speed_feedforward_ptr;
            pid_measure = (motor_setting->speed_feedback_source == OTHER_FEED && motor_controller->other_speed_feedback_ptr != NULL) ?
                              *motor_controller->other_speed_feedback_ptr :
                              measure->speed_aps;
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if ((motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) && motor_controller->current_feedforward_ptr != NULL)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
            pid_ref = PIDCalculate(&motor_controller->current_PID, (float)measure->real_current, pid_ref);
        if (motor->dji_mode == CURRENT_MODE)
            pid_ref = motor_controller->pid_ref;
        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        int16_t set = (int16_t)DJIMotor_Limit_Output(motor, pid_ref);
        uint8_t group = motor->sender_group;
        uint8_t num = motor->message_num;
        sender_assignment[group].tx_buff[2U * num] = (uint8_t)(set >> 8);
        sender_assignment[group].tx_buff[2U * num + 1U] = (uint8_t)(set & 0x00FF);

        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2U * num, 0, sizeof(uint16_t));
    }

    for (size_t i = 0; i < 3U; ++i)
    {
        if (sender_enable_flag[i])
            CANTransmit(&sender_assignment[i], sender_assignment[i].tx_id, sender_assignment[i].tx_buff, 8);
    }
}
