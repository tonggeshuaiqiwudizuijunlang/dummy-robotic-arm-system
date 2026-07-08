/**
 * @file jz_motor.c
 * @author Jiaxing He
 * @brief 机致电机c文件
 * @version 0.1
 * @date 2024-11-01
 */
#include "jz_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"

static uint8_t idx = 0;
static JZMotorInstance *jz_motor_instance[JZ_MOTOR_CNT] = {NULL};
static osThreadId jz_task_handle[JZ_MOTOR_CNT];
static CANInstance *sender_instance; 

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DecodeJZMotor(CANInstance *_instance)
{
    uint8_t *rx_buff = _instance->rx_buff;
    JZMotorInstance *motor = (JZMotorInstance *)_instance->id;
    JZ_Motor_Measure_s *measure = &motor->measure;

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    measure->position = measure->position;
    measure->position = (rx_buff[1] << 8) + rx_buff[2];

    //measure->angle_single_round = ECD_ANGLE_COEF_JZ * measure->position;

    measure->velocity = (int16_t)(((rx_buff[3]&0xFF)<<4)+((rx_buff[4]&0xF0)>>4))-2048;

    measure->real_current = (int16_t)(((rx_buff[4]&0xF0)<<8)+rx_buff[5])-2048;
    
}

static void JZMotorSetMode(JZMotor_Mode_e cmd, JZMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instance, 1);
}

static void JZMotorLostCallback(void *motor_ptr)
{
    JZMotorInstance *motor = (JZMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[jz_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

JZMotorInstance *JZMotorInit(Motor_Init_Config_s *config)
{
    JZMotorInstance *instance = (JZMotorInstance *)malloc(sizeof(JZMotorInstance));
    memset(instance, 0, sizeof(JZMotorInstance));

    instance->motor_type = config->motor_type;                         // 我们现在只使用HO10010
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DecodeJZMotor;
    config->can_init_config.id = instance; 
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = JZMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    JZMotorEnable(instance);
    JZMotorSetMode(JZ_CMD_MOTOR_MODE, instance);
    DWT_Delay(0.1);
    JZMotorCaliEncoder(instance);
    DWT_Delay(0.1);
    jz_motor_instance[idx++] = instance;
    return instance;
}

void JZMotorStop(JZMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void JZMotorEnable(JZMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void JZMotorOuterLoop(JZMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

void JZMotorCaliEncoder(JZMotorInstance *motor)
{
    JZMotorSetMode(JZ_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

void JZMotorSetRef(JZMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
void JZMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    JZMotorInstance *motor;
    JZ_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    uint16_t tmp;
    CANInstance *motor_can = motor->motor_can_instance;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = jz_motor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->motor_controller.pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->motor_controller.other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            // measure单位是rad,ref是角度,统一到angle下计算,方便建模
            pid_ref = PIDCalculate(&motor->motor_controller.angle_PID, pid_measure * RAD_2_DEGREE, pid_ref);
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->motor_controller.speed_feedforward_ptr;

            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->motor_controller.other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity;
            // measure单位是rad / s ,ref是angle per sec,统一到angle下计算
            pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure * RAD_2_DEGREE, pid_ref);
        }

        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor->motor_controller.current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->motor_controller.current_PID, measure->real_current, pid_ref);
        }

        set = pid_ref;

        LIMIT_MIN_MAX(set, T_MIN, T_MAX);
        tmp = float_to_uint(set, T_MIN, T_MAX, 12);
        if (motor->stop_flag == MOTOR_STOP)
            tmp = float_to_uint(0, T_MIN, T_MAX, 12);
        motor_can->tx_buff[6] = (tmp >> 8);
        motor_can->tx_buff[7] = tmp & 0xff;  //直接发送电流  发送协议中Byte 7中力矩命令高四 Byte中力矩命令低八

        if(idx)
            CANTransmit(motor_can, 0.5);
    }
}
