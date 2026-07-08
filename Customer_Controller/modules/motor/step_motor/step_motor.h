#ifndef _Joint_MOTOR_H
#define _Joint_MOTOR_H

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "daemon.h" // 复用守护进程
#include "motor_def.h"

/* 六轴机械臂 + 1级夹爪 */
#define ARM_JOINT_COUNT 7

/* Dummy 协议指令集 */
// 无存储类指令 (掉电不保存)
#define CMD_ENABLE 0x01        // 使能/失能电机 (Data: uint32 1=On, 0=Off)
#define CMD_CALIBRATE 0x02     // 触发编码器校准
#define CMD_SET_CURRENT 0x03   // 力矩/电流控制 (Data: float Current)
#define CMD_SET_VELOCITY 0x04  // 纯速度控制 (Data: float Turns/s)
#define CMD_SET_POSITION 0x05  // 纯位置控制 (Data: float Turns, Byte[4]=ACK)
#define CMD_SET_POS_TIME 0x06  // 位置+时间控制 (指定时间到达)
#define CMD_POS_VEL_LIMIT 0x07 // 位置+速度限制 (梯形轨迹规划) [关键指令]

// 存储类指令 (Config)
#define CMD_SET_NODE_ID 0x11     // 修改电机ID
#define CMD_SET_CUR_LIMIT 0x12   // 设置电流限制
#define CMD_SET_VEL_LIMIT 0x13   // 设置速度限制
#define CMD_SET_ACC 0x14         // 设置加速度
#define CMD_APPLY_HOME 0x15      // 设置当前位置为零点
#define CMD_SET_AUTO_ENABLE 0x16 // 上电自动使能
#define CMD_SET_DCE_KP 0x17      // 设置位置环 KP
#define CMD_SET_DCE_KV 0x18      // 设置速度环 KV
#define CMD_SET_DCE_KI 0x19      // 设置积分环 KI
#define CMD_SET_DCE_KD 0x1A      // 设置微分环 KD
#define CMD_SET_STALL_PROT 0x1B  // 堵转保护开关

// 查询类指令 (Master问 -> Slave答)
#define CMD_GET_CURRENT 0x21  // 回复当前电流
#define CMD_GET_VELOCITY 0x22 // 回复当前速度
#define CMD_GET_POSITION 0x23 // 回复当前位置
#define CMD_GET_OFFSET 0x24   // 回复编码器偏置
#define CMD_GET_TEMP 0x25     // 回复驱动板温度

// 系统指令
#define CMD_SAVE_CONFIG 0x7E // 擦除/保存配置
#define CMD_REBOOT 0x7F      // 重启电机

#define MOTOR_OFFLINE_TIMEOUT_MS 1000

typedef enum
{
    MODE_STOP = 0,
    MODE_COMMAND_POSITION,
    MODE_COMMAND_VELOCITY,
    MODE_COMMAND_CURRENT,
    MODE_COMMAND_Trajectory,
    MODE_PWM_POSITION,
    MODE_PWM_VELOCITY,
    MODE_PWM_CURRENT,
    MODE_Joint_DIR,
} Mode_t;

typedef enum
{
    STATE_STOP = 0,
    STATE_FINISH,
    STATE_RUNNING,
    STATE_OVERLOAD,
    STATE_STALL,
    STATE_NO_CALIB
} State_t;

typedef struct{
    float reduction_ratio;  // 减速比
    bool  reverse_flag;     // 反转标志 (true=反向)
    float joint_min_angle;
    float joint_max_angle;
} StepMotor_Config_t;

/* 关节状态结构体 */
typedef struct
{

    StepMotor_Config_t config;
    uint8_t id;             // 1-6
    bool is_online;         // 在线状态
    float last_online_time; // 上次收到消息的时间戳 (ms)
    float turns;            // 单位：圈 (Turns)
    float reduction_angle;  // 单位：度 (Angle)
    float velocity;         // 单位：圈/秒
    float current;          // 单位：A
    float temperature;      // 单位：℃
    int32_t raw_offset;     // 编码器偏置

    uint8_t is_finished; // 运动是否完成 (from 0x23 byte[4])
} Joint_t;

/* 步进电机实例 */
typedef struct
{
    Joint_t joint; // 电机关节状态
    /* 系统部分 */
    CANInstance *motor_can_instance; // 电机CAN实例
    DaemonInstance *daemon;          // 守护进程实例
} JointMotorInstance;

/* 夹爪参数配置结构体 */
typedef struct {
    uint8_t id;                        // 夹爪电机ID
    float target_grab_angle;           // 自动抓取时闭合极限位置
    float stop_threshold_current;      // 力控电流阈值绝对值
    float release_angle;               // 松开时的复位角度
    float speed_limit;                 // 夹爪运动速度
    uint32_t timeout_loops;            // 超时保护循环次数
} Gripper_Config_t;

/* --- API --- */
void Joint_Motor_Init_Legacy(void); // Renaming or Removing old declaration
Joint_t *Joint_Motor_Get_Joint(uint8_t id);

// 控制指令

/**
 * @brief 使能电机
 */
void Joint_Motor_Enable(uint8_t id, bool enable);

/**
 * @brief 位置控制
 */
void Joint_Motor_Set_Pos(uint8_t id, float turns, bool need_ack);

/**
 * @brief 速度控制
 */
void Joint_Motor_Set_Speed(uint8_t id, float speed_turns);

/**
 * @brief 电流(力矩)控制
 */
void Joint_Motor_Set_Current(uint8_t id, float current);

/**
 * @brief [核心] 位置控制 + 速度限制 (梯形加减速规划)
 * @details 对应 Cmd 0x07, 电机将总是回复 ACK (Cmd 0x23)
 * @param angle_degree 目标位置 (度)
 * @param speed_limit_degree 最大巡航速度 (度/秒)
 */
void Joint_Motor_Set_Pos_With_SpeedLimit(uint8_t id, float angle_degree, float speed_limit_degree);

/**
 * @brief [核心] 位置控制 + 时间限制 (指定时间到达)
 * @details 对应 Cmd 0x06, 电机将总是回复 ACK (Cmd 0x23)
 * @param angle_degree 目标位置 (度)
 * @param time_limit 指定到达时间 (为浮点数，代表秒)
 */
void Joint_Motor_Set_Pos_With_Time(uint8_t id, float angle_degree, float time_limit);

// 查询指令
void Joint_Motor_Init(StepMotor_Config_t *configs); // Keep the new one
void Joint_Motor_Query_State(uint8_t id);
float Joint_Motor_Get_Angle(uint8_t id);
uint8_t Joint_Motor_Get_State(uint8_t id);
// 杂项
void Joint_Motor_Set_ID(uint8_t current_id, uint8_t new_id);
void Joint_Motor_Reboot(uint8_t id);
void Joint_Motor_Set_Zero(uint8_t id);
void CANTransmit_Test(void);
void Joint_All_Motor_Set(float joint1_angle, float joint2_angle, float joint3_angle, float joint4_angle, float joint5_angle, float joint6_angle);

#define GRIPPER_STATE_IDLE      0 // 空闲
#define GRIPPER_STATE_WAIT_LOAD 1 // 等待受力
#define GRIPPER_STATE_LOCKED    2 // 已锁定
// 夹爪控制 API
void Joint_Motor_Gripper_Task(Gripper_Config_t *config, uint8_t gripper_mode);

#endif /* __Joint_MOTOR_H */