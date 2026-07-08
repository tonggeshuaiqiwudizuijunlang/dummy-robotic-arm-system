/**
 * @file rs_motor.c
 * @author yjf
 * @brief 灵足电机c文件
 * @version 0.1
 * @date 2025-12-09
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "rs_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "string.h"
#include "user_lib.h"
#include "daemon.h"
#include "stdlib.h"

static uint8_t idx = 0;
static RSMotorInstance *rs_motor_instance[RS_MOTOR_CNT] = {NULL};
static osThreadId rs_task_handle[RS_MOTOR_CNT];

static CANInstance *sender_instance;

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DecodeRSMotor(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据
    uint8_t *rxbuff = motor_can->rx_buff;
    RSMotorInstance *motor = (RSMotorInstance *)motor_can->id;
    RS_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->daemon);

    measure->last_position = measure->position;
    // 解析ID
    motor->measure.id = rxbuff[0];
    // 解析角度 (Byte1-2)
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    //* 反向
    measure->position = uint_to_float(tmp, RS_P_MIN, RS_P_MAX, 16) * -1.f;
    // // 将角度规范化到 [0, 360] 范围内
    // measure->position = fmodf(measure->position * RAD_2_DEGREE + 360.0f, 360.0f);
    // if (measure->position < 0)
    //     measure->position += 360.0f;
    // if (measure->position >= 360.0f)
    //     measure->position = 0.0f;

    // 解析速度 (Byte3 和 Byte4[7-4])
    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, RS_V_MIN, RS_V_MAX, 12) * RAD_2_DEGREE;

    // 解析扭矩 (Byte4[3-0] 和 Byte5)
    tmp = (uint16_t)(((rxbuff[4] & 0x0F) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, RS_T_MIN, RS_T_MAX, 12);

    // 解析温度 (Byte6-7)
    measure->temperature = (float)((rxbuff[6] << 8) | rxbuff[7]) * 0.1f;
}

static void RSMotorSetMode(RSMotor_Mode_e cmd, RSMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instance, 1);
}
void RSMotorSetMIT(RSMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 6);   // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[6] = (uint8_t)0x02; // 最后一位是命令id
    motor->motor_can_instance->tx_buff[7] = 0xFD;          // MIT模式命令
    CANTransmit(motor->motor_can_instance, 1);
}
void RSMotorCaliEncoder(RSMotorInstance *motor)
{
    RSMotorSetMode(RS_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
static void RSMotorLostCallback(void *motor_ptr)
{
    RSMotorInstance *motor = (RSMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[rs_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
    RSMotorSetMode(RS_CMD_MOTOR_MODE, motor); // 重新使能
}
void RSMotorChangeCanID(RSMotorInstance *motor, uint16_t can_id)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 6); // 发送电机指令的时候前面6bytes都是0xff
    motor->motor_can_instance->tx_buff[6] = (uint8_t)can_id;
    motor->motor_can_instance->tx_buff[7] = 0xfa;
}
void RSMotorChangeMasterCanID(RSMotorInstance *motor, uint16_t can_id)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 6); // 发送电机指令的时候前面6bytes都是0xff
    motor->motor_can_instance->tx_buff[6] = (uint8_t)can_id;
    motor->motor_can_instance->tx_buff[7] = 0x01;
}
RSMotorInstance *RSMotorInit(Motor_Init_Config_s *config)
{
    RSMotorInstance *instance = (RSMotorInstance *)malloc(sizeof(RSMotorInstance));
    memset(instance, 0, sizeof(RSMotorInstance));

    instance->motor_type = config->motor_type;                         // 我们现在只使用 RS05
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DecodeRSMotor;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = RSMotorLostCallback,
        .owner_id = instance,
        .reload_count = 10, // 检测周期100ms
    };
    instance->daemon = DaemonRegister(&conf);

    RSMotorEnable(instance);
    RSMotorSetMode(RS_CMD_MOTOR_MODE, instance);
    DWT_Delay(0.1);
    // RSMotorCaliEncoder(instance); // 将当前编码器位置作为零位
    DWT_Delay(0.1);

    // 初始化完成后加入实例数组
    rs_motor_instance[idx++] = instance;
    return instance;
}
void RSMotorSetRef(RSMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void RSMotorEnable(RSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}
void RSMotorMITSet(RSMotorInstance *motor, float pos, float vel, float kp, float kd, float torque)
{
    motor->MIT_position = pos;
    motor->MIT_velocity = vel;
    motor->MIT_kp = kp;
    motor->MIT_kd = kd;
    motor->MIT_torque = torque;
}
void RSMotorStop(RSMotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void RSMotorOuterLoop(RSMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void RSMotorTask(void const *argument)
{
    float set, pid_measure, pid_ref;
    RSMotorInstance *motor = (RSMotorInstance *)argument;
    RS_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    CANInstance *motor_can = motor->motor_can_instance;
    uint16_t tmp;
    float output;
    while (1)
    {

        pid_ref = motor->motor_controller.pid_ref;
        output = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            pid_ref *= -1;
            output *= -1;
        }

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->motor_controller.other_angle_feedback_ptr;
            else
                pid_measure = measure->position;
            // measure单位是rad
            pid_ref = PIDCalculate(&motor->motor_controller.angle_PID, pid_measure, pid_ref);
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->motor_controller.speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor->motor_controller.other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity;
            // measure单位是rad / s
            pid_ref = PIDCalculate(&motor->motor_controller.speed_PID, pid_measure, pid_ref);
        }

        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor->motor_controller.current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->motor_controller.current_PID, measure->torque, pid_ref);
        }

        set = pid_ref;
        set = output;

        LIMIT_MIN_MAX(set, RS_T_MIN, RS_T_MAX);
        tmp = float_to_uint(set, RS_T_MIN, RS_T_MAX, 12);
        if (motor->stop_flag == MOTOR_STOP)
            tmp = float_to_uint(0, RS_T_MIN, RS_T_MAX, 12);

        // 打包数据
        memset(motor_can->tx_buff, 0, 8);
        motor_can->tx_buff[6] = (tmp >> 8) & 0x0F;
        motor_can->tx_buff[7] = tmp & 0xff;

        CANTransmit(motor_can, 1);

        osDelay(2); // 500Hz
    }
}
void RSMotorControlInit()
{
    // 创建所有注册的RS电机任务
    if (!idx)
        return;

    for (size_t i = 0; i < idx; i++)
    {
        char task_name[8];
        snprintf(task_name, sizeof(task_name), "rs%d", (int)i);
        osThreadDef(task_name, RSMotorTask, osPriorityNormal, 0, 128);
        rs_task_handle[i] = osThreadCreate(osThread(task_name), rs_motor_instance[i]);
    }
}

// @todo 传入MIT模式的5个参数进行控制
void RSMotorMITControl(RSMotorInstance *motor, float position, float velocity, float kp, float kd, float torque)
{
    uint8_t *tx_buff = motor->motor_can_instance->tx_buff;

    // 限制参数范围
    LIMIT_MIN_MAX(position, RS_P_MIN, RS_P_MAX);
    LIMIT_MIN_MAX(velocity, RS_V_MIN, RS_V_MAX);
    LIMIT_MIN_MAX(kp, 0, RS_KP_MAX);
    LIMIT_MIN_MAX(kd, 0, RS_KD_MAX);
    LIMIT_MIN_MAX(torque, RS_T_MIN, RS_T_MAX);

    // 打包MIT控制数据
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

    CANTransmit(motor->motor_can_instance, 1);
}
