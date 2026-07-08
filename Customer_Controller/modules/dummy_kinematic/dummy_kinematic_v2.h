#ifndef DUMMY_KINEMATIC_V2_H
#define DUMMY_KINEMATIC_V2_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef KINEMATIC_DEG_TO_RAD
#define KINEMATIC_DEG_TO_RAD (0.01745329251994329577f)
#endif

#ifndef KINEMATIC_RAD_TO_DEG
#define KINEMATIC_RAD_TO_DEG (57.295779513082320876f)
#endif

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
    float A, B, C;       // 欧拉角 Roll, Pitch, Yaw，单位 deg
    float R[9];          // 旋转矩阵缓存，Rz(Yaw) * Ry(Pitch) * Rx(Roll)
    bool hasR;           // 标记是否已有旋转矩阵
} Pose6D_t;

// 逆解结果集 (最多8组解)
typedef struct {
    Joint6D_t config[8];   // 机械臂IK关节角，单位 deg
    char solFlag[8][3];    // 每组解的有效性标记 (1:有效, 0:空, -1:无效)
} IKSolves_t;

typedef struct
{
    // 原始电机/IMU读数。q1来自RS位置(rad)，q2/q3由DJI total_angle转为rad。
    float q1_rad;
    float q2_rad;
    float q3_rad;
    float imu_roll_deg;
    float imu_pitch_deg;
    float imu_yaw_deg;
} ControllerInput_t;

typedef struct
{
    float q1_initial_deg;
    float q2_initial_deg;
    float q3_initial_deg;
    float q1_offset_deg;
    float q2_offset_deg;
    float q3_offset_deg;

    float q1_max_rad;
    float q2_max_deg;
    float q3_max_deg;
    float q1_limit_torque;

    float delta_r_m;
    float delta_R_m;
    float delta_L_m;
    float delta_La_m;

    float xyz_delta_error_m[3];
    float xyz_arm_error_m[3];
    float xyz_scale[3];
    float pitch_limit_deg;
} ControllerKinematicConfig_t;

typedef struct {
    double d[6];
    double a[6];
    double alpha[6];
} KinematicDH_t;

typedef struct {
    float min_deg;
    float max_deg;
    bool enable;
} AngleLimit_t;

typedef struct {
    float map[6][6];
    float bias_deg[6];
} AffineAngleMap_t;

typedef struct {
    uint8_t lhs_motor_index;
    uint8_t rhs_motor_index;
    float min_delta_deg;
    float max_delta_deg;
    bool enable;
} MotorCouplingLimit_t;

typedef struct {
    KinematicDH_t dh;
    float pose_to_dh_scale;     // Pose6D mm -> DH长度单位，原工程为 m，所以通常为0.001
    float dh_to_pose_scale;     // DH长度单位 -> Pose6D mm，原工程为 m，所以通常为1000

    float ik_reference_init_deg[6];
    AngleLimit_t joint_limit[6];
    AngleLimit_t motor_limit[6];

    AffineAngleMap_t joint_to_motor;
    AffineAngleMap_t motor_to_joint;

    MotorCouplingLimit_t coupling_limits[4];
    uint8_t coupling_limit_count;

    float select_weight[6];
    float max_single_joint_jump_rad;
    float max_total_cost;
    bool enable_q3_beyond_180_patch;

    ControllerKinematicConfig_t controller_config;
} ArmConfig_t;

// 控制器运动学配置句柄
typedef struct {
    ArmConfig_t config;
    double current_ik_rad[6];
    double last_answer_rad[6];
    int flag_beyond_180;
    bool initialized;
} DOF6Kinematic_Handle_t;

// ---------------- 函数声明 ----------------

void Kinematic_Get_Default_Config(ArmConfig_t *config);

/**
 * @brief 初始化控制器运动学解算器
 */
void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config);

/**
 * @brief 正解包装。输入为IK/DH关节角(deg)，输出Pose6D(mm)。
 */
bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle,
                       const Joint6D_t *inputJoints,
                       Pose6D_t *outputPose);

/**
 * @brief 原工程Arm_Moveit逆解包装。Pose -> 最多8组IK关节角(deg)。
 */
bool Kinematic_SolveIK(const DOF6Kinematic_Handle_t *handle,
                       const Pose6D_t *inputPose,
                       const Joint6D_t *lastJoints,
                       IKSolves_t *outputSolves);

/**
 * @brief 从结果集中挑选最连续的一组解
 */
int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle,
                              const IKSolves_t *solves,
                              const Joint6D_t *current_joints);

float Kinematic_Clamp(float value, float min, float max);
float Kinematic_Scale_Controller_Pitch(const DOF6Kinematic_Handle_t *handle, float pitch_deg);

void Kinematic_Build_Rotation_ZYX(float yaw_deg,
                                  float pitch_deg,
                                  float roll_deg,
                                  float R[9]);

void Kinematic_Build_Pose(float x_mm,
                          float y_mm,
                          float z_mm,
                          float roll_deg,
                          float pitch_deg,
                          float yaw_deg,
                          Pose6D_t *pose);

bool Kinematic_ControllerFK(const DOF6Kinematic_Handle_t *handle,
                            const ControllerInput_t *input,
                            Pose6D_t *pose,
                            float torque[3],
                            float xyz_m[3]);

bool Kinematic_ControllerDeltaFK(const DOF6Kinematic_Handle_t *handle,
                                 const float theta_rad[3],
                                 float xyz_m[3]);

bool Kinematic_ControllerGravityCompensation(const DOF6Kinematic_Handle_t *handle,
                                             const float theta_rad[3],
                                             float torque[3],
                                             float xyz_m[3]);

void Kinematic_IKAngle_To_Motor(const DOF6Kinematic_Handle_t *handle,
                                const Joint6D_t *ik_joint_deg,
                                Joint6D_t *motor_joint_deg);

void Kinematic_Motor_To_IKAngle(const DOF6Kinematic_Handle_t *handle,
                                const Joint6D_t *motor_joint_deg,
                                Joint6D_t *ik_joint_deg);

void Kinematic_Limit_Motor_Angle(const DOF6Kinematic_Handle_t *handle,
                                 Joint6D_t *motor_joint_deg);

bool Kinematic_SolveIK_To_Motor(DOF6Kinematic_Handle_t *handle,
                                const Pose6D_t *target_pose,
                                Joint6D_t *last_valid_ik_joint_deg,
                                Joint6D_t *motor_joint_deg,
                                int *best_index);

#endif // DUMMY_KINEMATIC_V2_H
