#ifndef DUMMY_KINEMATIC_V2_H
#define DUMMY_KINEMATIC_V2_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "motor_def.h"
// ---------------- 定义数据结构 ----------------

typedef struct {
    float angle[6]; // 单位：度 (Degrees)
    uint8_t state[6]; // 每个关节的状态 (0: 未到达目标点 1: 到达目标点)
} Joint_Angle_State_t;

// 6轴关节角度结构体
typedef struct {
    float a[6]; // 单位：度 (Degrees)
} Joint6D_t;

// 6D 位姿结构体
typedef struct {
    float X, Y, Z;       // 位置 mm
    float A, B, C;       // ZYX 欧拉角: A=Roll(X), B=Pitch(Y), C=Yaw(Z), 单位: 度
    float R[9];          // 旋转矩阵缓存 (行主序)
    bool hasR;           // 标记是否已有旋转矩阵
} Pose6D_t;

// 逆解结果集 (最多8组解)
typedef struct {
    Joint6D_t config[8];   // 8组可能的关节角度
    char solFlag[8][3];    // 解标记: [0]基座 [1]大小臂 [2]腕部; 1有效, 0无效, -1奇异但可参考
} IKSolves_t;

typedef struct {
    // 关节限制 (单位: 度)
    // 很多机械臂有关节限位，不仅是物理限位，算法也需要知道
    float joint_min_limit[6];
    float joint_max_limit[6];
    
    // DH 参数 / 连杆长度 (单位: mm)
    float L_BASE;
    float D_BASE;
    float L_ARM;
    float L_FOREARM;
    float D_ELBOW;
    float L_WRIST;
} ArmConfig_t;

// 机械臂配置参数 (DH 参数)
typedef struct {
    ArmConfig_t config; // 包含所有配置信息

    // 内部缓存变量，避免重复计算
    float DH_matrix[6][4];
    float L1_base[3];
    float L2_arm[3];
    float L3_elbow[3];
    float L6_wrist[3];
    
    // 预计算常量
    float l_se_2;
    float l_se;
    float l_ew_2;
    float l_ew;
    float atan_e; // [新增] 必须添加此成员
} DOF6Kinematic_Handle_t;

// ---------------- 函数声明 ----------------

/**
 * @brief 初始化运动学解算器 (类似 C++ 构造函数)
 * @param handle 句柄指针
 * @param config 配置结构体指针 (传入后被复制到 handle 内部)
 */
void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config);

/**
 * @brief 正运动学解算 (Joints -> Pose)
 */
bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle, 
                       const Joint6D_t *inputJoints, 
                       Pose6D_t *outputPose);

/**
 * @brief 逆运动学解算 (Pose -> Joints)
 * @param lastJoints 上一时刻的关节角度 (用于从8组解中选最近的一组)
 */
bool Kinematic_SolveIK(const DOF6Kinematic_Handle_t *handle, 
                       const Pose6D_t *inputPose, 
                       const Joint6D_t *lastJoints, 
                       IKSolves_t *outputSolves);

/**
 * @brief 辅助函数：从8组解中挑选最优解
 * @return 最优解在 outputSolves->config[] 中的索引，返回 -1 表示无解
 */
int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle, const IKSolves_t *solves, const Joint6D_t *current_joints);
// int Kinematic_Select_Best_Sol(const IKSolves_t *solves, const Joint6D_t *current_joints);

void Joint_Motor_Get_All_Angles_And_State(Joint_Angle_State_t *joint_angle);

#endif // DUMMY_KINEMATIC_V2_H
