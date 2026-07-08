#include "dummy_motormatic.h"
#include "dummy_cmd.h"

#define DUMMY_MOTOR_PLANNER_DEFAULT_SPEED_DEG 30.0f
#define DUMMY_MOTOR_PLANNER_MIN_TIME          0.05f
#define DUMMY_MOTOR_PLANNER_MIN_DELTA_DEG     0.01f
#define DUMMY_MOTOR_PLANNER_MIN_SPEED_DEG     0.1f

static DOF6Kinematic_Handle_t kinematic_handle; // 运动学处理句柄

static Pose6D_t current_pose;             // 当前位姿监测
static Joint6D_t target_joints_angle;     // 目标关节角度
static Joint6D_t current_joints_feedback; // 当前关节角度反馈 (从电机读取)
static IKSolves_t ik_results;             // 逆解结果
static int best_sol_index = 1;            // 最优解索引
static bool is_pose_initialized = false;  // 位姿是否初始化

static Publisher_t *dummy_motormatic_pub;   // 控制消息发布者
static Subscriber_t *dummy_motormatic_sub;  // 反馈信息订阅者
static Dummy_Ctrl_Cmd_s dummy_cmd_data;     // 控制命令缓存
static Dummy_Upload_Data_s dummy_feed_data; // 反馈数据缓存
static Pose6D_t target_pose;                // 期望目标位姿

static Joint_Angle_State_t joint_angle;
static Gripper_Config_t gripper_config;     // 夹爪配置结构体

DOF6Kinematic_Handle_t *Dummy_Motormatic_GetKinematicHandle(void)
{
    return &kinematic_handle;
}

void Dummy_Joint_Update(void)
{
    Joint_Motor_Check_Connection();
    Joint_Motor_Get_All_Angles_And_State(&joint_angle);
    for (uint8_t i = 0; i < 6; i++)
    {
        // Joint_Motor_Query_State(i + 1); // 电机ID从1开始
        current_joints_feedback.a[i] = joint_angle.angle[i];
        dummy_feed_data.joint_motor[i].reduction_angle = joint_angle.angle[i];
        dummy_feed_data.joint_motor[i].is_finished = joint_angle.state[i];
    }

    Kinematic_SolveFK(&kinematic_handle, &current_joints_feedback, &current_pose);
    dummy_feed_data.current_joints_feedback = current_joints_feedback;
    dummy_feed_data.current_pose = current_pose;

    dummy_feed_data.current_mode = dummy_cmd_data.arm_mode;
    dummy_feed_data.arm_ctrl_mode = dummy_cmd_data.arm_ctrl_mode;
    dummy_feed_data.gripper_mode = dummy_cmd_data.gripper_mode;
}

void Dummy_Motormatic_Init(void)
{
    // 1. 填充配置结构体 (这里可以根据实际电机情况修改)
    ArmConfig_t my_arm_config = {
        // --- 几何尺寸 (mm) ---
        .L_BASE = 109.0f,
        .D_BASE = 35.0f,
        .L_ARM = 146.0f,
        .L_FOREARM = 115.0f,
        .D_ELBOW = 52.0f,
        .L_WRIST = 72.0f,
        // --- 关节软限位 (度) ---
        .joint_min_limit = {-175.0f, -25.0f, 0.0f, -180.0f, -120.0f, -360.0f},
        .joint_max_limit = {175.0f, 150.0f, 180.0f, 180.0f, 120.0f, 360.0f}};
    Kinematic_Init(&kinematic_handle, &my_arm_config);

    // 1. 电机配置 (减速比 + 方向 + 关节限位)
    StepMotor_Config_t motor_configs[7] = {
        {50.0f, false, -175.0f, 175.0f},  // J1
        {50.0f, true,  -25.0f, 150.0f},  // J2
        {50.0f, true,    0.0f, 180.0f},  // J3
        {50.0f, true, -180.0f, 180.0f},  // J4
        {50.0f, true, -120.0f, 120.0f},  // J5
        {50.0f, false, -360.0f, 360.0f}, // J6
        {8.0f, true, -720.0f, 720.0f}     // 夹爪
    };
    // 初始化电机驱动 (传入配置)
    Joint_Motor_Init(motor_configs);

    // --- 夹爪专门配置参数 ---
    gripper_config.id = 7;
    gripper_config.target_grab_angle = 500.0f;
    gripper_config.stop_threshold_current = DUMMY_GRIPPER_STOP_THRESHOLD_CURRENT;
    gripper_config.release_angle = 0.0f;
    gripper_config.speed_limit = 50.0f;
    gripper_config.timeout_loops = 500;
#if DUMMY_GRIPPER_CALIB_ENABLE
    Joint_Motor_Gripper_Calib_Reset();
#endif

    dummy_motormatic_pub = PubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
    dummy_motormatic_sub = SubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));

    is_pose_initialized = true;
}

static float Angle_Delta_Abs(float target, float current)
{
    float delta = target - current;
    if (delta < 0.0f)
        delta = -delta;
    return delta;
}

static void Dummy_Motormatic_MoveJ_Sync(const Joint6D_t *target, const uint8_t *active_axis)
{
    float max_delta = 0.0f;
    float max_speed = dummy_cmd_data.arm_max_speed;
    float move_time;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (!active_axis[i])
            continue;

        float delta = Angle_Delta_Abs(target->a[i], current_joints_feedback.a[i]);
        if (delta > max_delta)
            max_delta = delta;
    }

    if (max_delta < DUMMY_MOTOR_PLANNER_MIN_DELTA_DEG)
        return;

    if (max_speed < DUMMY_MOTOR_PLANNER_MIN_SPEED_DEG)
        max_speed = DUMMY_MOTOR_PLANNER_DEFAULT_SPEED_DEG;

    move_time = max_delta / max_speed;
    if (dummy_cmd_data.arm_arrived_time > move_time)
        move_time = dummy_cmd_data.arm_arrived_time;
    if (move_time < DUMMY_MOTOR_PLANNER_MIN_TIME)
        move_time = DUMMY_MOTOR_PLANNER_MIN_TIME;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (!active_axis[i])
            continue;

        float delta = Angle_Delta_Abs(target->a[i], current_joints_feedback.a[i]);
        if (delta < DUMMY_MOTOR_PLANNER_MIN_DELTA_DEG)
            continue;

        float speed = delta / move_time;
        if (speed < DUMMY_MOTOR_PLANNER_MIN_SPEED_DEG)
            speed = DUMMY_MOTOR_PLANNER_MIN_SPEED_DEG;

        Joint_Motor_Set_Pos_With_SpeedLimit(i + 1, target->a[i], speed);
    }
}

void Dummy_Motormatic_Task(void)
{
    if (!is_pose_initialized)
        return;
    SubGetMessage(dummy_motormatic_sub, &dummy_cmd_data);

    Dummy_Joint_Update();

    if (dummy_cmd_data.arm_mode == ARM_ZERO_FORCE)
    {
        for (uint8_t i = 0; i < 7; i++)
        {
            Joint_Motor_Set_Current(i + 1, 0.0f);
        }
        Dummy_Joint_Update();
        PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
        return;
    }

    bool gripper_calib_active = false;
#if DUMMY_GRIPPER_CALIB_ENABLE
    gripper_calib_active = !Joint_Motor_Gripper_Calib_Task(&gripper_config);
#if DUMMY_GRIPPER_CALIB_BLOCK_ARM_ENABLE
    if (gripper_calib_active)
    {
        PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
        return;
    }
#endif
#endif

    if (dummy_cmd_data.arm_mode == ARM_FREE_MODE)
    {
        Joint6D_t target = {{
            dummy_cmd_data.joint1_angle,
            dummy_cmd_data.joint2_angle,
            dummy_cmd_data.joint3_angle,
            dummy_cmd_data.joint4_angle,
            dummy_cmd_data.joint5_angle,
            dummy_cmd_data.joint6_angle,
        }};

        if (dummy_cmd_data.arm_ctrl_mode == BIG_ARM_CTRL)
        {
            const uint8_t active_axis[6] = {1, 1, 1, 1, 0, 0};
            Dummy_Motormatic_MoveJ_Sync(&target, active_axis);
        }
        else if (dummy_cmd_data.arm_ctrl_mode == SMALL_ARM_CTRL)
        {
            const uint8_t active_axis[6] = {0, 0, 1, 1, 1, 1};
            Dummy_Motormatic_MoveJ_Sync(&target, active_axis);
        }
    }
    else if ((dummy_cmd_data.arm_mode == ARM_PC_MODE) ||
             (dummy_cmd_data.arm_mode == ARM_CUSTOM_MODE))
    {
        const uint8_t active_axis[6] = {1, 1, 1, 1, 1, 1};
        Joint6D_t target = {{
            dummy_cmd_data.joint1_angle,
            dummy_cmd_data.joint2_angle,
            dummy_cmd_data.joint3_angle,
            dummy_cmd_data.joint4_angle,
            dummy_cmd_data.joint5_angle,
            dummy_cmd_data.joint6_angle,
        }};
        Dummy_Motormatic_MoveJ_Sync(&target, active_axis);
    }

    if (!gripper_calib_active)
    {
        if (dummy_cmd_data.gripper_mode == GRIPPER_MANUAL_GRAB)
        {
            float gripper_angle = Joint_Motor_Gripper_Map_Knob(dummy_cmd_data.gripper_current, 0.0f, REMOTE_KNOB_RANGE);
            Joint_Motor_Set_Pos_With_SpeedLimit(7, gripper_angle, gripper_config.speed_limit);
        }
        else
        {
            // 定期执行夹爪防错状态机，传入结构体配置以及模式枚举
            Joint_Motor_Gripper_Task(&gripper_config, (uint8_t)dummy_cmd_data.gripper_mode);
        }
    }
    PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
}
