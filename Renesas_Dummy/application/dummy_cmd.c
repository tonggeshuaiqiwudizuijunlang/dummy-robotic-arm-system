#include "dummy_cmd.h"
#include "message_center.h"
#include "flysky.h"
#include "bluetooth.h"
#include "serial_debug.h"
#include "vision.h"
#include "dummy_motormatic.h"



static FS_ctrl_t *fs_data;
static Publisher_t *dummy_cmd_pub;           // 控制消息发布者
static Subscriber_t *dummy_feed_sub;         // 反馈信息订阅者
static Dummy_Ctrl_Cmd_s dummy_cmd_send;      // 控制命令缓存
static Dummy_Upload_Data_s dummy_fetch_data; // 反馈数据缓存

static Transmit_Data_s vision_tx_data;  // 发送给上位机的数据缓存
static Received_Data_s *vision_rx_data; // 从上位机接收的数据缓存
static DOF6Kinematic_Handle_t *cmd_kinematic_handle;

#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
static Pose6D_t remote_target_pose;
static bool remote_pose_initialized = false;
#endif

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static SerialDebug_Instance_s *serial_debug_instance;
#endif

RX_BT_Pose_Data_s *bt_rx_data;
TX_BT_Data_s bt_tx_data;

volatile DummyCmd_Debug_s dummy_cmd_debug = {
    .bt_last_status = DUMMY_CMD_BT_STATUS_IDLE,
};


static void FS_Remote_Control(void);
static void Vision_Set_FeedData(void);
static void DummyCmd_Set_Controller_Joints(arm_mode_e arm_mode, const Joint6D_t *joints);
static void DummyCmd_Motor_To_Controller_Joints(const Joint6D_t *motor_joints, Joint6D_t *controller_joints);
static uint8_t DummyCmd_Get_Motion_Finished_Feedback(void);
#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE || (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
static float Limit_Float(float value, float min, float max);
static float Normalize_Pose_Angle(float angle);
#endif
#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
static void FS_Remote_Pose_Test_Control(void);
#endif
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static void DummyCmd_Serial_Debug_Send(void);
#if DUMMY_CMD_FKIK_TEST_ENABLE
static void DummyCmd_FKIK_Test_Send(void);
#endif
#endif
static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd);
static void Bt_Set_FeedData(void);
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
static bool Bt_Pose_IK_Control(void);
static bool Bt_Pose_Try_Solve(const Pose6D_t *target_pose);
#endif

void DummyCmd_Init(void)
{
    DWT_Init(200);
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
    bt_rx_data = BT_Init(&bt_uart_ctrl, &bt_uart_cfg);
#elif (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
    SerialDebug_Init_Config_s serial_debug_config;
    serial_debug_config.usart_config.p_uart_ctrl = &bt_uart_ctrl;
    serial_debug_config.usart_config.p_uart_cfg = &bt_uart_cfg;
    serial_debug_config.pid_callback = NULL;
    serial_debug_instance = SerialDebug_Init(&serial_debug_config);
#endif
    vision_rx_data = Vision_Init(&pc_uart_ctrl, &pc_uart_cfg);
    cmd_kinematic_handle = Dummy_Motormatic_GetKinematicHandle();
    fs_data = FSControlInit(&sbus_ctrl, &sbus_cfg);
    dummy_cmd_pub = PubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));
    dummy_feed_sub = SubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
}

void DummyCmd_Task(void)
{
    SubGetMessage(dummy_feed_sub, &dummy_fetch_data);
    FS_Remote_Control();
    Vision_Set_FeedData();
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
#if DUMMY_CMD_FKIK_TEST_ENABLE
    DummyCmd_FKIK_Test_Send();
#else
    DummyCmd_Serial_Debug_Send();
#endif
#endif

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
    Bt_Set_FeedData();
#endif
    PubPushMessage(dummy_cmd_pub, (void *)&dummy_cmd_send);
}

static void FS_Remote_Control(void)
{
    float gripper_knob = fs_data[TEMP].knob_l;

    dummy_cmd_debug.last_switch_l2 = fs_data[TEMP].switch_l2;
    dummy_cmd_debug.last_switch_r1 = fs_data[TEMP].switch_r1;
    dummy_cmd_debug.last_switch_r2 = fs_data[TEMP].switch_r2;

    // 优先级1：R2为急停开关，只要拨到DOWN就直接进入零力模式，其他控制全部不再处理
    if (fs_data[TEMP].switch_r2 == FS_SW_DOWN)
    {
        dummy_cmd_send.arm_mode = ARM_ZERO_FORCE;
        return;
    }
    // R2拨到UP时才允许正常控制，先恢复夹爪的基础控制命令
    else
    {
        // 右旋钮：前半段保持普通夹爪控制，超过一半则启动自动夹爪程序
        if (fs_data[TEMP].knob_r > REMOTE_KNOB_HALF)
        {
            dummy_cmd_send.gripper_mode = GRIPPER_AUTO_GRAB;
        }
        else
        {
            dummy_cmd_send.gripper_mode = GRIPPER_MANUAL_GRAB;
            // 左旋钮：传递原始旋钮值，由电机任务按校准后的夹爪边界映射为角度
            dummy_cmd_send.gripper_current = gripper_knob;
        }
        // 优先级2：R1为强制点位开关，用于快速回到安全预设点
#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
        if (fs_data[TEMP].switch_l2 != FS_SW_UP)
            remote_pose_initialized = false;
#endif
        // R1_DOWN：强行回到0点；R1_MID：强行回到“7”字形点；R1_UP：正常控制
        // 这里使用ARM_PC_MODE下发6个关节目标，避免大小臂模式只控制部分关节
        if (fs_data[TEMP].switch_r1 == FS_SW_DOWN)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = ARM_HOME_JOINT1;
            dummy_cmd_send.joint2_angle = ARM_HOME_JOINT2;
            dummy_cmd_send.joint3_angle = ARM_HOME_JOINT3;
            dummy_cmd_send.joint4_angle = ARM_HOME_JOINT4;
            dummy_cmd_send.joint5_angle = ARM_HOME_JOINT5;
            dummy_cmd_send.joint6_angle = ARM_HOME_JOINT6;
            return;
        }
        else if (fs_data[TEMP].switch_r1 == FS_SW_MID)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = ARM_SEVEN_JOINT1;
            dummy_cmd_send.joint2_angle = ARM_SEVEN_JOINT2;
            dummy_cmd_send.joint3_angle = ARM_SEVEN_JOINT3;
            dummy_cmd_send.joint4_angle = ARM_SEVEN_JOINT4;
            dummy_cmd_send.joint5_angle = ARM_SEVEN_JOINT5;
            dummy_cmd_send.joint6_angle = ARM_SEVEN_JOINT6;
            return;
        }
        // 优先级3：L2为控制方式选择开关
        // L2_DOWN：PC上位机控制，直接使用视觉/上位机给出的目标角度
        if (fs_data[TEMP].switch_l2 == FS_SW_DOWN)
        {
            Joint6D_t vision_joints = {{
                vision_rx_data->joint1,
                vision_rx_data->joint2,
                vision_rx_data->joint3,
                vision_rx_data->joint4,
                vision_rx_data->joint5,
                vision_rx_data->joint6,
            }};
            DummyCmd_Set_Controller_Joints(ARM_PC_MODE, &vision_joints);

        }
        // L2_MID: Bluetooth pose control, solve 6 joint targets with IK.
        else if (fs_data[TEMP].switch_l2 == FS_SW_MID)
        {
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
            dummy_cmd_debug.bt_branch_count++;
            if (!Bt_Pose_IK_Control())
            {
                dummy_cmd_send.arm_mode = ARM_FREE_MODE;
            }
#else
            dummy_cmd_send.arm_mode = ARM_FREE_MODE;
#endif

        }
        // L2_UP：遥控器控制，L1用于切换大臂/小臂控制
        else if (fs_data[TEMP].switch_l2 == FS_SW_UP)
        {
#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
            FS_Remote_Pose_Test_Control();
#else
            dummy_cmd_send.arm_mode = ARM_FREE_MODE;

            // L1_DOWN：小臂控制，四个摇杆以积分方式控制joint3~joint6
            if (fs_data[TEMP].switch_l1 == FS_SW_DOWN)
            {
                dummy_cmd_send.arm_ctrl_mode = SMALL_ARM_CTRL;
                dummy_cmd_send.joint3_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l1);
                dummy_cmd_send.joint4_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l_);
                dummy_cmd_send.joint5_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r1);
                dummy_cmd_send.joint6_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r_);
            }
            // L1_UP：大臂控制，四个摇杆以积分方式控制joint1~joint4
            else
            {
                dummy_cmd_send.arm_ctrl_mode = BIG_ARM_CTRL;
                dummy_cmd_send.joint1_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r_);
                dummy_cmd_send.joint2_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r1);
                dummy_cmd_send.joint3_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l1);
                dummy_cmd_send.joint4_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l_);
            }
#endif
        }
    }
    limit_joint_angles(&dummy_cmd_send);
}

static void DummyCmd_Set_Controller_Joints(arm_mode_e arm_mode, const Joint6D_t *joints)
{
    if (joints == NULL)
        return;

    dummy_cmd_send.arm_mode = arm_mode;
    dummy_cmd_send.joint1_angle = -joints->a[0];
    dummy_cmd_send.joint2_angle = -joints->a[1] + 75.0f;
    dummy_cmd_send.joint3_angle = -joints->a[2] + 90.0f;
    dummy_cmd_send.joint4_angle = joints->a[3];
    dummy_cmd_send.joint5_angle = -joints->a[4];
    dummy_cmd_send.joint6_angle = joints->a[5];
}

static void DummyCmd_Motor_To_Controller_Joints(const Joint6D_t *motor_joints, Joint6D_t *controller_joints)
{
    if ((motor_joints == NULL) || (controller_joints == NULL))
        return;

    controller_joints->a[0] = -motor_joints->a[0];
    controller_joints->a[1] = -motor_joints->a[1] + 75.0f;
    controller_joints->a[2] = -motor_joints->a[2] + 90.0f;
    controller_joints->a[3] = motor_joints->a[3];
    controller_joints->a[4] = -motor_joints->a[4];
    controller_joints->a[5] = motor_joints->a[5];
}

static uint8_t DummyCmd_Get_Motion_Finished_Feedback(void)
{
#if DUMMY_CMD_NONBLOCKING_FINISH_FEEDBACK_ENABLE
    return 1u;
#else
    for (uint8_t i = 0; i < 6; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0)
            return 0u;
    }

    return 1u;
#endif
}

static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd)
{
    // 关节角度限幅 (与 Dummy_Motormatic_Init 中 ArmConfig_t 和 motor_configs 保持一致)
    // joint_min_limit: {-175, -25,   0, -180, -120, -360}
    // joint_max_limit: { 175, 150, 180,  180,  120,  360}

    if (cmd->joint1_angle < -175.0f)
        cmd->joint1_angle = -175.0f;
    if (cmd->joint1_angle > 175.0f)
        cmd->joint1_angle = 175.0f;

    if (cmd->joint2_angle < -25.0f)
        cmd->joint2_angle = -25.0f;
    if (cmd->joint2_angle > 150.0f)
        cmd->joint2_angle = 150.0f;

    if (cmd->joint3_angle < 0.0f)
        cmd->joint3_angle = 0.0f;
    if (cmd->joint3_angle > 180.0f)
        cmd->joint3_angle = 180.0f;

    if (cmd->joint4_angle < -180.0f)
        cmd->joint4_angle = -180.0f;
    if (cmd->joint4_angle > 180.0f)
        cmd->joint4_angle = 180.0f;

    if (cmd->joint5_angle < -120.0f)
        cmd->joint5_angle = -120.0f;
    if (cmd->joint5_angle > 120.0f)
        cmd->joint5_angle = 120.0f;

    if (cmd->joint6_angle < -360.0f)
        cmd->joint6_angle = -360.0f;
    if (cmd->joint6_angle > 360.0f)
        cmd->joint6_angle = 360.0f;
}

#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE || (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
static float Limit_Float(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static float Normalize_Pose_Angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}
#endif

#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
static void FS_Remote_Pose_Test_Control(void)
{
    if (cmd_kinematic_handle == NULL)
        return;

    if (!remote_pose_initialized)
    {
        if (!dummy_fetch_data.current_pose.hasR)
            return;

        remote_target_pose = dummy_fetch_data.current_pose;
        remote_target_pose.hasR = false;
        remote_pose_initialized = true;
    }

    if (fs_data[TEMP].switch_l1 == FS_SW_DOWN)
    {
        remote_target_pose.X += REMOTE_POSE_POS_GAIN * fs_data[TEMP].rocker_r1;
        remote_target_pose.Y += REMOTE_POSE_POS_GAIN * fs_data[TEMP].rocker_r_;
        remote_target_pose.Z += REMOTE_POSE_POS_GAIN * fs_data[TEMP].rocker_l1;
    }
    else
    {
        remote_target_pose.A += REMOTE_POSE_ATT_GAIN * fs_data[TEMP].rocker_l1;
        remote_target_pose.B += REMOTE_POSE_ATT_GAIN * fs_data[TEMP].rocker_l_;
        remote_target_pose.C += REMOTE_POSE_ATT_GAIN * fs_data[TEMP].rocker_r_;
    }
    remote_target_pose.hasR = false;

    remote_target_pose.X = Limit_Float(remote_target_pose.X, -250.0f, 300.0f);
    remote_target_pose.Y = Limit_Float(remote_target_pose.Y, -300.0f, 300.0f);
    remote_target_pose.Z = Limit_Float(remote_target_pose.Z, -50.0f, 450.0f);
    remote_target_pose.A = Limit_Float(remote_target_pose.A, -180.0f, 180.0f);
    remote_target_pose.B = Limit_Float(remote_target_pose.B, -180.0f, 180.0f);
    remote_target_pose.C = Limit_Float(remote_target_pose.C, -180.0f, 180.0f);

    IKSolves_t solves;
    Joint6D_t current_joints = dummy_fetch_data.current_joints_feedback;
    current_joints.a[1] = current_joints.a[1] - 75.0f;
    current_joints.a[2] = current_joints.a[2] - 90.0f;
    if (Kinematic_SolveIK(cmd_kinematic_handle, &remote_target_pose, &current_joints, &solves))
    {
        int best = Kinematic_Select_Best_Sol(cmd_kinematic_handle, &solves, &current_joints);
        if (best >= 0)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = solves.config[best].a[0];
            dummy_cmd_send.joint2_angle = solves.config[best].a[1];
            dummy_cmd_send.joint3_angle = solves.config[best].a[2];
            dummy_cmd_send.joint4_angle = solves.config[best].a[3];
            dummy_cmd_send.joint5_angle = solves.config[best].a[4];
            dummy_cmd_send.joint6_angle = solves.config[best].a[5];
        }
    }
}
#endif

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
static bool Bt_Pose_IK_Control(void)
{
    if ((cmd_kinematic_handle == NULL) || (bt_rx_data == NULL))
    {
        dummy_cmd_debug.bt_last_status = DUMMY_CMD_BT_STATUS_NO_HANDLE;
        return false;
    }

    dummy_cmd_debug.bt_last_header = bt_rx_data->header;
    dummy_cmd_debug.bt_last_tailer = bt_rx_data->tailer;

    if ((bt_rx_data->header != 0x5A) || (bt_rx_data->tailer != 0xFFFB))
    {
        dummy_cmd_debug.bt_last_status = DUMMY_CMD_BT_STATUS_NO_FRAME;
        return false;
    }

    Pose6D_t target_pose = {
        .X = bt_rx_data->x * BT_POSE_POS_SCALE,
        .Y = bt_rx_data->y * BT_POSE_POS_SCALE,
        .Z = bt_rx_data->z * BT_POSE_POS_SCALE,
        .A = bt_rx_data->roll,
        .B = bt_rx_data->pitch,
        .C = bt_rx_data->yaw,
        .hasR = false,
    };
    target_pose.X = Limit_Float(target_pose.X, DUMMY_CMD_WORKSPACE_X_MIN, DUMMY_CMD_WORKSPACE_X_MAX);
    target_pose.Y = Limit_Float(target_pose.Y, DUMMY_CMD_WORKSPACE_Y_MIN, DUMMY_CMD_WORKSPACE_Y_MAX);
    target_pose.Z = Limit_Float(target_pose.Z, DUMMY_CMD_WORKSPACE_Z_MIN, DUMMY_CMD_WORKSPACE_Z_MAX);
    target_pose.A = Limit_Float(Normalize_Pose_Angle(target_pose.A), DUMMY_CMD_POSE_ATT_MIN, DUMMY_CMD_POSE_ATT_MAX);
    target_pose.B = Limit_Float(Normalize_Pose_Angle(target_pose.B), DUMMY_CMD_POSE_ATT_MIN, DUMMY_CMD_POSE_ATT_MAX);
    target_pose.C = Limit_Float(Normalize_Pose_Angle(target_pose.C), DUMMY_CMD_POSE_ATT_MIN, DUMMY_CMD_POSE_ATT_MAX);
    dummy_cmd_debug.bt_target_x = target_pose.X;
    dummy_cmd_debug.bt_target_y = target_pose.Y;
    dummy_cmd_debug.bt_target_z = target_pose.Z;

    if (Bt_Pose_Try_Solve(&target_pose))
    {
        dummy_cmd_debug.bt_last_status = DUMMY_CMD_BT_STATUS_OK;
        dummy_cmd_debug.bt_ik_ok_count++;
        return true;
    }

#if BT_POSE_ATT_FALLBACK_ENABLE
    if (dummy_fetch_data.current_pose.hasR)
    {
        target_pose.A = dummy_fetch_data.current_pose.A;
        target_pose.B = dummy_fetch_data.current_pose.B;
        target_pose.C = dummy_fetch_data.current_pose.C;
    }
    else
    {
        target_pose.A = 0.0f;
        target_pose.B = 0.0f;
        target_pose.C = 0.0f;
    }
    target_pose.hasR = false;
    if (Bt_Pose_Try_Solve(&target_pose))
    {
        dummy_cmd_debug.bt_last_status = DUMMY_CMD_BT_STATUS_OK;
        dummy_cmd_debug.bt_ik_ok_count++;
        return true;
    }
#endif

    dummy_cmd_debug.bt_last_status = DUMMY_CMD_BT_STATUS_IK_FAIL;
    dummy_cmd_debug.bt_ik_fail_count++;
    return false;
}

static bool Bt_Pose_Try_Solve(const Pose6D_t *target_pose)
{
    IKSolves_t solves;
    Joint6D_t current_joints = dummy_fetch_data.current_joints_feedback;

    DummyCmd_Motor_To_Controller_Joints(&dummy_fetch_data.current_joints_feedback, &current_joints);

    if (Kinematic_SolveIK(cmd_kinematic_handle, target_pose, &current_joints, &solves))
    {
        int best = Kinematic_Select_Best_Sol(cmd_kinematic_handle, &solves, &current_joints);
        if (best >= 0)
        {
            DummyCmd_Set_Controller_Joints(ARM_CUSTOM_MODE, &solves.config[best]);
            return true;
        }
    }

    return false;
}
#endif

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static void DummyCmd_Serial_Debug_Send(void)
{
    static uint16_t send_div_count = 0;

    send_div_count++;
    if (send_div_count < SERIAL_DEBUG_SEND_DIV)
        return;
    send_div_count = 0;

    float channels[SERIAL_DEBUG_JUSTFLOAT_CHANNELS] = {
        dummy_fetch_data.current_joints_feedback.a[0],
        dummy_fetch_data.current_joints_feedback.a[1],
        dummy_fetch_data.current_joints_feedback.a[2],
        dummy_fetch_data.current_joints_feedback.a[3],
        dummy_fetch_data.current_joints_feedback.a[4],
        dummy_fetch_data.current_joints_feedback.a[5],
        dummy_fetch_data.current_pose.X,
        dummy_fetch_data.current_pose.Y,
        dummy_fetch_data.current_pose.Z,
        dummy_fetch_data.current_pose.A,
        dummy_fetch_data.current_pose.B,
        dummy_fetch_data.current_pose.C,
#if DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
        remote_target_pose.X,
        remote_target_pose.Y,
        remote_target_pose.Z,
        remote_target_pose.A,
        remote_target_pose.B,
        remote_target_pose.C,
#else
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
#endif
    };

    SerialDebug_Send_JustFloat(serial_debug_instance, channels, SERIAL_DEBUG_JUSTFLOAT_CHANNELS);
}
#if DUMMY_CMD_FKIK_TEST_ENABLE
static void DummyCmd_FKIK_Test_Send(void)
{
    static uint16_t send_div_count = 0;

    send_div_count++;
    if (send_div_count < DUMMY_CMD_FKIK_TEST_SEND_DIV)
        return;
    send_div_count = 0;

    if (cmd_kinematic_handle == NULL)
        return;

    Joint6D_t test_joints = {{30.0f, 20.0f, 40.0f, 10.0f, 20.0f, 30.0f}};
    Pose6D_t test_pose;
    IKSolves_t test_solves;
    Joint6D_t output_joints = test_joints;

    if (!Kinematic_SolveFK(cmd_kinematic_handle, &test_joints, &test_pose))
        return;

    if (Kinematic_SolveIK(cmd_kinematic_handle, &test_pose, &test_joints, &test_solves))
    {
        int best_index = Kinematic_Select_Best_Sol(cmd_kinematic_handle, &test_solves, &test_joints);
        if (best_index >= 0)
            output_joints = test_solves.config[best_index];
    }

    float channels[SERIAL_DEBUG_JUSTFLOAT_CHANNELS] = {
        output_joints.a[0],
        output_joints.a[1],
        output_joints.a[2],
        output_joints.a[3],
        output_joints.a[4],
        output_joints.a[5],
        test_pose.X,
        test_pose.Y,
        test_pose.Z,
        test_pose.A,
        test_pose.B,
        test_pose.C,
    };

    SerialDebug_Send_JustFloat(serial_debug_instance, channels, SERIAL_DEBUG_JUSTFLOAT_CHANNELS);
}
#endif
#endif

static void Vision_Set_FeedData(void)
{
    vision_tx_data.header = 0x5A;
    vision_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    vision_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    vision_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    vision_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    vision_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    vision_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    // 根据要求，当所有电机完成时设置为11，否则为0
    vision_tx_data.is_finished = DummyCmd_Get_Motion_Finished_Feedback();
    vision_tx_data.tailer = 0XFFFB;
    Vision_Send_Data((uint8_t *)&vision_tx_data, sizeof(Transmit_Data_s));
}

static void Bt_Set_FeedData(void)
{
    bt_tx_data.header = 0xAA;
    bt_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    bt_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    bt_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    bt_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    bt_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    bt_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    // 根据要求，当所有电机完成时设置为11，否则为0
    bt_tx_data.is_finished = DummyCmd_Get_Motion_Finished_Feedback();
    bt_tx_data.tailer = 0XFFFB;
    BT_SendData((uint8_t *)&bt_tx_data, sizeof(TX_BT_Data_s));
}
