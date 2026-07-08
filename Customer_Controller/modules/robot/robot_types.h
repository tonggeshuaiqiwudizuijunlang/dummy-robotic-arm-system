#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

/* app层定�?*/

typedef enum
{
    ARM_ZERO_FORCE = 0,
    ARM_FREE_MODE,
    ARM_PC_MODE,        // 上位�?蓝牙关节角模�?    ARM_CARTESIAN_MODE  // 末端位姿 IK 模式
} arm_mode_e;

typedef enum
{
    GRIPPER_RELEASE = 0,    // 松开
    GRIPPER_AUTO_GRAB = 1,  // 自动抓取
    GRIPPER_MANUAL_GRAB     // 手动抓取
} gripper_mode_e;

typedef enum
{
    BIG_ARM_CTRL = 0,
    SMALL_ARM_CTRL
} arm_ctrl_mode_e;

typedef enum
{
    ARM_STATUS_IDLE = 0,    // 空闲
    ARM_STATUS_MOVING,      // 正在移动�?    ARM_STATUS_ARRIVED,     // 已到达目标点 (误差范围�?
    ARM_STATUS_ERROR,       // 发生错误 (如逆解无解、碰撞保�?
    ARM_CARTESIAN_MODE
} arm_state_e;

typedef struct
{
    float reduction_angle;
    float velocity;
    float current;
    float temperature;
    uint8_t is_online;
    uint8_t is_finished;
} Dummy_Joint_Feedback_t;

typedef struct
{
    /* --- 机械臂模�?(Arm Mode) --- */
    arm_mode_e arm_mode;
    arm_ctrl_mode_e arm_ctrl_mode;
    gripper_mode_e gripper_mode;
    arm_state_e  arm_state;
    float    arm_max_speed; // 机械臂最大速度 (单位: mm/s)
    float    arm_arrived_time;

    /* --- 目标角度 --- */
    float joint1_angle;
    float joint2_angle;
    float joint3_angle;
    float joint4_angle;
    float joint5_angle;
    float joint6_angle;

    /* --- 末端目标位姿，位�?mm，姿�?deg --- */
    float target_x;
    float target_y;
    float target_z;
    float target_roll;
    float target_pitch;
    float target_yaw;

    float gripper_current;

} Dummy_Ctrl_Cmd_s;

typedef struct
{
    /* --- 机械臂模�?(Arm Mode) --- */
    arm_mode_e current_mode;
    gripper_mode_e gripper_mode;
    arm_ctrl_mode_e arm_ctrl_mode;
    arm_state_e  arm_state;

    /* --- 当前控制器解算出的末端位姿，位置 mm，姿�?deg --- */
    float    cur_x;
    float    cur_y;
    float    cur_z;

    float    cur_roll;
    float    cur_pitch;
    float    cur_yaw;


    float    controller_motor_angle[3]; // deg: q1 RS, q2 DJI, q3 DJI
    float    imu_euler[3];              // deg: roll, pitch, yaw
    /* --- 关节目标信息 --- *
    * joint_motor[0~5] 对应 Axis 1-6
    * joint_motor[6]   保留给兼容层 */
    Dummy_Joint_Feedback_t joint_motor[7];

} Dummy_Upload_Data_s;

#endif // ROBOT_TYPES_H
