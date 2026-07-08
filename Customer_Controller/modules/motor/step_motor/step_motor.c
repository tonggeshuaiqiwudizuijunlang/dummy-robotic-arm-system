#include "step_motor.h"
#include "robot_types.h"
/* CAN 监听实例 */
CANInstance *Joint_can_instance = NULL;
/* Dummy 电机实例 */
Joint_t dummy_joints[ARM_JOINT_COUNT + 1];
StepMotor_Config_t joint_configs[ARM_JOINT_COUNT + 1];
/* 全局单例初始化标记 */
static bool is_driver_initialized = false;

/**
 * @brief 通用发送函数
 * @param id 电机ID
 * @param cmd 命令码
 * @param data 数据指针 (可以为NULL)
 * @param len 长度
 */
static void Send_Generic(uint8_t id, uint8_t cmd, void *data, uint8_t len)
{
    if (Joint_can_instance == NULL || id > ARM_JOINT_COUNT || len > 8)
        return;
    // 构造 ID: [Node 4bit] [Cmd 7bit]
    uint32_t std_id = ((uint32_t)id << 7) | cmd;

    CANTransmit(Joint_can_instance, std_id, (uint8_t *)data, len);
}
static void Send_Float(uint8_t id, uint8_t cmd, float val)
{
    Send_Generic(id, cmd, &val, 4);
}

static void Send_U32(uint8_t id, uint8_t cmd, uint32_t val) // 虽然API叫Set_ID等，但底层发的是U32
{
    Send_Generic(id, cmd, &val, 4);
}

/**
 * @brief CAN 接收回调 (从 BSP 层过来)
 */
static void Joint_Motor_Callback(CANInstance *_instance)
{
    /* 1. 提取 ID */
    uint32_t rx_id = _instance->current_rx_id;

    /* 2. 拆解 ID (高4位=ID, 低7位=CMD) */
    uint8_t node_id = (rx_id >> 7) & 0x0F;
    uint8_t cmd = rx_id & 0x7F;

    if (node_id < 1 || node_id > ARM_JOINT_COUNT)
        return;

    Joint_t *joint = &dummy_joints[node_id];
    StepMotor_Config_t *joint_config = &joint_configs[node_id];
    switch (cmd)
    {
    case CMD_GET_CURRENT: // 0x21 Response
    {
        // Data: [Float Current (4)] + [FinishFlag (1)]
        if (_instance->rx_len >= 4)
            memcpy(&joint->current, _instance->rx_buff, 4);
        if (_instance->rx_len >= 5)
            joint->is_finished = _instance->rx_buff[4];
        break;
    }
    case CMD_GET_VELOCITY: // 0x22 Response
    {
        // Data: [Float Velocity (4)] + [FinishFlag (1)]
        if (_instance->rx_len >= 4)
            memcpy(&joint->velocity, _instance->rx_buff, 4);
        break;
    }
    case CMD_GET_POSITION: // 0x23 Response (包含 0x05 带有 ACK 时的回复)
    {
        if (_instance->rx_len >= 4)
        {
            memcpy(&joint->turns, _instance->rx_buff, 4);
            joint->reduction_angle = (joint->turns * 360.0f / joint_config->reduction_ratio); // 转换为角度
        }
        if (_instance->rx_len >= 5)
            joint->is_finished = _instance->rx_buff[4];

        break;
    }
    case CMD_GET_OFFSET: // 0x24 Response
    {
        if (_instance->rx_len >= 4)
            memcpy(&joint->raw_offset, _instance->rx_buff, 4); // int32
        break;
    }
    case CMD_GET_TEMP: // 0x25 Response
    {
        if (_instance->rx_len >= 4)
            memcpy(&joint->temperature, _instance->rx_buff, 4);
        break;
    }
    default:
        break;
    }
}

/* 掉线回调 */
static void Joint_Motor_LostCallback(void *motor_ptr)
{
    Joint_t *joint = (Joint_t *)motor_ptr;
    Joint_Motor_Enable(joint->id, true);
}

/* 初始化函数 */
void Joint_Motor_Init(StepMotor_Config_t *configs)
{
    if (is_driver_initialized)
        return;

    // 1. 初始化 CAN BSP (注册回调放在前面)
    CAN_Init_Config_s conf = {0};
    conf.p_ctrl = &motor_ctrl_can_ctrl;
    conf.p_cfg = &motor_ctrl_can_cfg;
    conf.can_module_callback = Joint_Motor_Callback;
    Joint_can_instance = BSP_CAN_Init(&conf);
    // 2. 保存配置
    if (configs != NULL)
    {
        for (uint8_t i = 1; i <= ARM_JOINT_COUNT; i++)
        {
            joint_configs[i] = configs[i - 1]; // 传入数组0-6，存入1-7
        }
    }
    /* 1. 清空结构体 */
    memset(dummy_joints, 0, sizeof(dummy_joints));
    for (uint8_t i = 1; i <= ARM_JOINT_COUNT; i++)
    {
        dummy_joints[i].id = (uint8_t)i;
    }
    Joint_Motor_Enable(0, true);
    is_driver_initialized = true;
}

void Joint_Motor_Enable(uint8_t id, bool enable)
{
    /* Slave Case 0x01: *(uint32_t*) (RxData) == 1 */
    /* Master 发送 uint32_t */
    Send_U32(id, CMD_ENABLE, enable);
}

/*
 * [新实现] 位置控制 + 速度限制
 * @param id 电机编号
 * @param angle_degree 目标关节角度 (度)
 * @param speed_limit_degree 速度限制 (度/秒)
 */
void Joint_Motor_Set_Pos_With_SpeedLimit(uint8_t id, float angle_degree, float speed_limit_degree)
{
    if (id < 1 || id > ARM_JOINT_COUNT)
        return;

    // 1. 获取配置
    float ratio = joint_configs[id].reduction_ratio;
    if (ratio < 0.001f)
        ratio = 1.0f; // 避免除零

    // 2. 角度(度) -> 电机圈数
    // 公式: 电机圈数 = (关节角度 / 360) * 减速比
    if (joint_configs[id].joint_max_angle < angle_degree)
        angle_degree = joint_configs[id].joint_max_angle;
    if (joint_configs[id].joint_min_angle > angle_degree)
        angle_degree = joint_configs[id].joint_min_angle;

    float target_motor_turns = (angle_degree / 360.0f) * ratio;
    // 3. 处理方向反转
    if (joint_configs[id].reverse_flag)
        target_motor_turns = -target_motor_turns;

    // 4. 速度换算: 度/秒 -> 圈/秒
    float target_motor_speed = (speed_limit_degree / 360.0f) * ratio;
    // 速度总是正数，不需要反转符号 (除非特定的速度环控制，但通常由位置环内部处理方向)
    target_motor_speed = fabsf(target_motor_speed);

    // 5. 组包发送
    uint8_t buffer[8];
    memcpy(buffer, &target_motor_turns, 4);
    memcpy(buffer + 4, &target_motor_speed, 4);

    Send_Generic(id, CMD_POS_VEL_LIMIT, buffer, 8);
}

/*
 * 位置控制 + 时间限制 (指定时间到达)
 * @param id 电机编号
 * @param angle_degree 目标关节角度 (度)
 * @param time_limit 指定到达时间 (为浮点数，代表秒)
 */
void Joint_Motor_Set_Pos_With_Time(uint8_t id, float angle_degree, float time_limit)
{
    if (id < 1 || id > ARM_JOINT_COUNT)
        return;

    // 1. 获取配置
    float ratio = joint_configs[id].reduction_ratio;
    if (ratio < 0.001f)
        ratio = 1.0f; // 避免除零

    // 2. 角度(度) -> 电机圈数
    // 公式: 电机圈数 = (关节角度 / 360) * 减速比
    if (joint_configs[id].joint_max_angle < angle_degree)
        angle_degree = joint_configs[id].joint_max_angle;
    if (joint_configs[id].joint_min_angle > angle_degree)
        angle_degree = joint_configs[id].joint_min_angle;

    float target_motor_turns = (angle_degree / 360.0f) * ratio;

    // 3. 处理方向反转
    if (joint_configs[id].reverse_flag)
        target_motor_turns = -target_motor_turns;

    // 时间要求是非负的，不需要转换单位
    if (time_limit < 0.0f)
        time_limit = 0.0f;

    // 4. 组包发送
    // 根据 Dummy 通信协议，CMD_SET_POS_TIME(0x06) 通常是 float 位置 + float 时间
    uint8_t buffer[8];
    memcpy(buffer, &target_motor_turns, 4);
    memcpy(buffer + 4, &time_limit, 4);
    buffer[4] |= 0x01;

    Send_Generic(id, CMD_SET_POS_TIME, buffer, 8);
}

void Joint_Motor_Set_Pos(uint8_t id, float turns, bool need_ack)
{
    /* Slave Case 0x05:
       Data[0-3]: Float (Turns)
       Data[4]:   ACK Flag
    */
    /* 注意：此函数名为Set_Pos，通常由底层的测试代码直接调用。
       如果要统一单位，建议也做一下转换。但为了兼容旧代码，暂不修改此函数，
       或者你可以明确它输入的是 Turns (电机圈数) 还是 Angle (关节角度)。
       根据用户需求，既然在 Init 配了减速比，这里也应该转。
       但我这里先保持原样，假定此函数是底层调试用，直接传圈数。
       上层应用主要用 Joint_Motor_Set_Pos_With_SpeedLimit。
    */
    uint8_t buffer[5];
    memcpy(buffer, &turns, 4);
    buffer[4] = need_ack ? 1 : 0;

    Send_Generic(id, CMD_SET_POSITION, buffer, 5);
}

void Joint_Motor_Set_Speed(uint8_t id, float speed_turns)
{
    /* Slave Case 0x04: Float */
    Send_Float(id, CMD_SET_VELOCITY, speed_turns);
}

void Joint_Motor_Set_Current(uint8_t id, float current)
{
    /* Slave Case 0x03: Float (Slave 内部会 x1000 转 int32) */
    Send_Float(id, CMD_SET_CURRENT, current);
}

void Joint_Motor_Query_State(uint8_t id)
{
    /* 轮询 0x23 (Pos) 和 0x25 (Temp) */
    /* 这些是 Inquiry CMD，发送时不需要带数据，Slave 收到后会回复 */
    Send_Generic(id, CMD_GET_POSITION, NULL, 0);
}

void Joint_Motor_Set_ID(uint8_t current_id, uint8_t new_id)
{
    // Slave Case 0x11: Data is uint32_t new_id
    uint32_t id_u32 = new_id;
    Send_U32(current_id, CMD_SET_NODE_ID, id_u32);
}

void Joint_Motor_Set_Zero(uint8_t id)
{
    // Cmd 0x15: Apply Home Position (Store to EEPROM)
    Send_Generic(id, CMD_APPLY_HOME, NULL, 0);
}

void Joint_Motor_Reboot(uint8_t id)
{
    // Slave Case 0x7F
    Send_Generic(id, CMD_REBOOT, NULL, 0);
}

void Joint_Motor_Check_Connection(void)
{
    float current_time = DWT_GetTimeline_ms();

    for (int i = 1; i <= ARM_JOINT_COUNT; i++)
    {
        Joint_t *joint = &dummy_joints[i];
        // 只有当之前是在线状态，才需要检测是否掉线
        if (joint->is_online)
        {
            // 如果 (当前时间 - 上次时间) > 阈值
            if ((current_time - joint->last_online_time) > MOTOR_OFFLINE_TIMEOUT_MS)
            {
                // 标记为离线
                joint->is_online = false;
                Joint_Motor_LostCallback(joint);
            }
        }
    }
}

float Joint_Motor_Get_Angle(uint8_t id)
{
    if (id < 1 || id > ARM_JOINT_COUNT)
        return 0.0f;
    return dummy_joints[id].reduction_angle; // 返回转换后的角度
}

uint8_t Joint_Motor_Get_State(uint8_t id)
{
    if (id < 1 || id > ARM_JOINT_COUNT)
        return 0;
    return dummy_joints[id].is_finished ? 1 : 0; // 简单返回是否完成
}

void Joint_All_Motor_Set(float joint1_angle, float joint2_angle, float joint3_angle, float joint4_angle, float joint5_angle, float joint6_angle)
{
    Joint_Motor_Set_Pos_With_SpeedLimit(1, joint1_angle, 10);
    Joint_Motor_Set_Pos_With_SpeedLimit(2, joint2_angle, 10);
    Joint_Motor_Set_Pos_With_SpeedLimit(3, joint3_angle, 10);
    Joint_Motor_Set_Pos_With_SpeedLimit(4, joint4_angle, 10);
    Joint_Motor_Set_Pos_With_SpeedLimit(5, joint5_angle, 10);
    Joint_Motor_Set_Pos_With_SpeedLimit(6, joint6_angle, 10);
}

void CANTransmit_Test(void)
{
}

// ==========================
// 夹爪控制状态机 API
// ==========================

#define GRIPPER_STATE_IDLE 0      // 空闲
#define GRIPPER_STATE_WAIT_LOAD 1 // 等待受力
#define GRIPPER_STATE_LOCKED 2    // 已锁定

/**
 * @brief 夹爪状态机任务
 * @param config       夹爪的各项参数配置结构体
 * @param gripper_mode 当前夹爪模式要求 (0: 松开, 1: 自动力控抓取)
 */
void Joint_Motor_Gripper_Task(Gripper_Config_t *config, uint8_t gripper_mode)
{
    if (config == NULL || config->id == 0 || config->id > ARM_JOINT_COUNT)
        return;

    static uint8_t grab_state = GRIPPER_STATE_IDLE;
    static uint32_t loop_count = 0; // 按RTOS周期累加计数器

    uint8_t id = config->id;

    if (gripper_mode == GRIPPER_RELEASE)
    {
        // 只要是释放模式，就一直给它发松开的位置或者单次发（这里为了让它动，直接下发位置）
        Joint_Motor_Set_Pos_With_SpeedLimit(id, config->release_angle, config->speed_limit);
        grab_state = GRIPPER_STATE_IDLE;
        loop_count = 0;
    }
    else if (gripper_mode == GRIPPER_AUTO_GRAB)
    {
        if (grab_state == GRIPPER_STATE_IDLE)
        {
            // 从空闲切成抓取，开始往目标点进发
            dummy_joints[id].current = 0.0f; // 清除历史反馈电流
            Joint_Motor_Set_Pos_With_SpeedLimit(id, config->target_grab_angle, config->speed_limit);
            grab_state = GRIPPER_STATE_WAIT_LOAD;
        }
        else if (grab_state == GRIPPER_STATE_WAIT_LOAD)
        {
            // 正在抓取途中，开始监测电流
            static uint8_t query_div = 0;
            if (++query_div >= 5)
            {
                query_div = 0;
                Send_Generic(id, CMD_GET_CURRENT, NULL, 0);
            }

            float actual_current = dummy_joints[id].current;
            float abs_current = (actual_current > 0) ? actual_current : -actual_current;

            if (abs_current >= config->stop_threshold_current)
            {
                loop_count++;
                if (loop_count > config->timeout_loops) // 堵转确认
                {
                    if (loop_count > 2 * config->timeout_loops)
                        loop_count = 2 * config->timeout_loops;

                    float dir_sign = (config->target_grab_angle > config->release_angle) ? 1.0f : -1.0f;
                    float hold_angle = dummy_joints[id].reduction_angle + (dir_sign * 3.0f);

                    Joint_Motor_Set_Pos_With_SpeedLimit(id, hold_angle, config->speed_limit);

                    grab_state = GRIPPER_STATE_LOCKED;
                }
            }
            else
            {
                if (loop_count > 50U)
                    loop_count -= 50U;
                else
                    loop_count = 0U;
            }
        }
        else if (grab_state == GRIPPER_STATE_LOCKED)
        {
            // 【防松开修复 2】：防止电机通信超时保护。
            // 很多驱动器若2秒收不到任何报文就会自动停机掉力。在这里加一个慢速循环保持唤醒它。
            static uint8_t keep_alive_div = 0;
            if (++keep_alive_div >= 5) // 假设Task 10~20ms跑一次，这里约200~400ms发一次唤醒包
            {
                keep_alive_div = 0;
                Send_Generic(id, CMD_GET_CURRENT, NULL, 0); // 用查询指令当作心跳包喂狗
            }
            float dir_sign = (config->target_grab_angle > config->release_angle) ? 1.0f : -1.0f;
            float hold_angle = dummy_joints[id].reduction_angle + (dir_sign * 3.0f);

            Joint_Motor_Set_Pos_With_SpeedLimit(id, hold_angle, config->speed_limit);
        }
    }
}