#include "dummy_motormatic.h"
#include "dji_motor.h"
#include "rs_motor.h"
#include "DM_IMU.h"
#include "message_center.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static DOF6Kinematic_Handle_t kinematic_handle;
static Pose6D_t current_pose;
static Pose6D_t target_pose;
static Joint6D_t last_valid_joint_cmd = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
static Joint6D_t last_valid_ik_joint_deg = {{0.0f, 75.0f, 105.0f, 0.0f, 0.0f, 0.0f}};
static bool is_pose_initialized = false;

static Publisher_t *dummy_motormatic_pub;
static Subscriber_t *dummy_motormatic_sub;
static Dummy_Ctrl_Cmd_s dummy_cmd_data;
static Dummy_Upload_Data_s dummy_feed_data;

static RSMotorInstance *q1_rs_motor = NULL;
static DJIMotorInstance *q2_dji_motor = NULL;
static DJIMotorInstance *q3_dji_motor = NULL;
static DM_IMU_Instance_s *imu_delta = NULL;

static float controller_torque[3] = {0.0f};
static float controller_xyz_m[3] = {0.0f};

static void Controller_Motor_Enable(bool enable)
{
    if (q1_rs_motor != NULL)
    {
        if (enable)
            RSMotorEnable(q1_rs_motor);
        else
            RSMotorStop(q1_rs_motor);
    }
    if (q2_dji_motor != NULL)
    {
        if (enable)
            DJIMotorEnable(q2_dji_motor);
        else
            DJIMotorStop(q2_dji_motor);
    }
    if (q3_dji_motor != NULL)
    {
        if (enable)
            DJIMotorEnable(q3_dji_motor);
        else
            DJIMotorStop(q3_dji_motor);
    }
}

static void Controller_Motor_Register(void)
{
    IMU_CAN_Init_Config_s imu_delta_config = {
        .can_config = {
            .p_ctrl = &motor_ctrl_can_ctrl,
            .p_cfg = &motor_ctrl_can_cfg,
            .tx_id = 0x22,
            .rx_id = 0x22,
        },
    };

    Motor_Init_Config_s q1_rs_config = {
        .can_init_config = {
            .p_ctrl = &motor_ctrl_can_ctrl,
            .p_cfg = &motor_ctrl_can_cfg,
            .tx_id = 0x01,
            .rx_id = 0xfd,
        },
        .motor_type = RS05,
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
    };

    Motor_Init_Config_s q2_6020_config = {
        .can_init_config = {
            .p_ctrl = &motor_ctrl_can_ctrl,
            .p_cfg = &motor_ctrl_can_cfg,
            .tx_id = 0x04,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 15.0f,
                .Ki = 0.0f,
                .Kd = 2.0f,
                .MaxOut = 5500.0f,
                .DeadBand = 1.0f,
            },
            .speed_PID = {
                .Kp = 15.0f,
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit || PID_Derivative_On_Measurement || PID_Trapezoid_Intergral,
                .IntegralLimit = 10000.0f,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };

    Motor_Init_Config_s q3_6020_config = {
        .can_init_config = {
            .p_ctrl = &motor_ctrl_can_ctrl,
            .p_cfg = &motor_ctrl_can_cfg,
            .tx_id = 0x02,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20.0f,
                .Ki = 0.0f,
                .Kd = 2.0f,
                .MaxOut = 5500.0f,
                .DeadBand = 1.0f,
            },
            .speed_PID = {
                .Kp = 15.0f,
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };

    q1_rs_motor = RSMotorInit(&q1_rs_config);
    q3_dji_motor = DJIMotorInit(&q3_6020_config);
    q2_dji_motor = DJIMotorInit(&q2_6020_config);

    if (q1_rs_motor != NULL)
        RSMotorCaliEncoder(q1_rs_motor);
    if (q2_dji_motor != NULL)
        DJiMotorSetMode(q2_dji_motor, CURRENT_MODE);
    if (q3_dji_motor != NULL)
        DJiMotorSetMode(q3_dji_motor, CURRENT_MODE);

    imu_delta = DM_IMU_Init(&imu_delta_config);
    if (imu_delta != NULL)
    {
        IMU_Mode_Set(imu_delta, DM_IMU_DataType_euler);
        IMU_Data_Request(imu_delta);
    }
}

static bool Dummy_Motormatic_Read_Controller_Input(ControllerInput_t *input)
{
    if (input == NULL || q1_rs_motor == NULL || q2_dji_motor == NULL || q3_dji_motor == NULL)
        return false;

    input->q1_rad = q1_rs_motor->measure.position;
    input->q2_rad = q2_dji_motor->measure.total_angle * KINEMATIC_DEG_TO_RAD;
    input->q3_rad = q3_dji_motor->measure.total_angle * KINEMATIC_DEG_TO_RAD;

    input->imu_roll_deg = 0.0f;
    input->imu_pitch_deg = 0.0f;
    input->imu_yaw_deg = 0.0f;
    if (imu_delta != NULL)
    {
        input->imu_roll_deg = imu_delta->IMU_Driver_Data.roll;
        input->imu_pitch_deg = imu_delta->IMU_Driver_Data.pitch;
        input->imu_yaw_deg = imu_delta->IMU_Driver_Data.yaw;
    }

    return true;
}

static void Dummy_Motormatic_Apply_Controller_Torque(const float torque[3])
{
    if (torque == NULL)
        return;

    if (q1_rs_motor != NULL)
        RSMotorSetRef(q1_rs_motor, torque[0] * 0.4f);
    if (q2_dji_motor != NULL)
        DJIMotorSetRef(q2_dji_motor, -torque[1] * 5461.333f * 1000.0f * 1.08f / 741.0f);
    if (q3_dji_motor != NULL)
        DJIMotorSetRef(q3_dji_motor, torque[2] * 5461.333f * 1000.0f * 1.1f / 741.0f);
}

static void Fill_Controller_Feedback(void)
{
    dummy_feed_data.controller_motor_angle[0] = (q1_rs_motor != NULL) ?
                                                    q1_rs_motor->measure.position * KINEMATIC_RAD_TO_DEG :
                                                    0.0f;
    dummy_feed_data.controller_motor_angle[1] = (q2_dji_motor != NULL) ?
                                                    q2_dji_motor->measure.total_angle :
                                                    0.0f;
    dummy_feed_data.controller_motor_angle[2] = (q3_dji_motor != NULL) ?
                                                    q3_dji_motor->measure.total_angle :
                                                    0.0f;

    dummy_feed_data.imu_euler[0] = (imu_delta != NULL) ? imu_delta->IMU_Driver_Data.roll : 0.0f;
    dummy_feed_data.imu_euler[1] = (imu_delta != NULL) ? imu_delta->IMU_Driver_Data.pitch : 0.0f;
    dummy_feed_data.imu_euler[2] = (imu_delta != NULL) ? imu_delta->IMU_Driver_Data.yaw : 0.0f;

    dummy_feed_data.joint_motor[0].is_online = (q1_rs_motor != NULL && q1_rs_motor->daemon != NULL) ?
                                                   DaemonIsOnline(q1_rs_motor->daemon) :
                                                   0U;
    dummy_feed_data.joint_motor[1].is_online = DJIMotorIsOnline(q2_dji_motor);
    dummy_feed_data.joint_motor[2].is_online = DJIMotorIsOnline(q3_dji_motor);
}

static void Fill_Common_Feedback(arm_state_e state)
{
    dummy_feed_data.current_mode = dummy_cmd_data.arm_mode;
    dummy_feed_data.arm_ctrl_mode = dummy_cmd_data.arm_ctrl_mode;
    dummy_feed_data.gripper_mode = dummy_cmd_data.gripper_mode;
    dummy_feed_data.arm_state = state;

    dummy_feed_data.cur_x = current_pose.X;
    dummy_feed_data.cur_y = current_pose.Y;
    dummy_feed_data.cur_z = current_pose.Z;
    dummy_feed_data.cur_roll = current_pose.A;
    dummy_feed_data.cur_pitch = current_pose.B;
    dummy_feed_data.cur_yaw = current_pose.C;

    Fill_Controller_Feedback();
}

static void Fill_Joint_Feedback(const Joint6D_t *joint_cmd, uint8_t is_finished)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        dummy_feed_data.joint_motor[i].reduction_angle = joint_cmd->a[i];
        dummy_feed_data.joint_motor[i].is_finished = is_finished;
        dummy_feed_data.joint_motor[i].is_online = 1U;
        dummy_feed_data.joint_motor[i].velocity = 0.0f;
        dummy_feed_data.joint_motor[i].current = 0.0f;
        dummy_feed_data.joint_motor[i].temperature = 0.0f;
    }

    dummy_feed_data.joint_motor[6].reduction_angle = 0.0f;
    dummy_feed_data.joint_motor[6].is_finished = 1U;
    dummy_feed_data.joint_motor[6].is_online = 0U;
    Fill_Controller_Feedback();
}

static void Fill_Pose_Only_Feedback(arm_state_e state)
{
    Fill_Common_Feedback(state);

    for (uint8_t i = 0; i < 7U; i++)
    {
        /* In pose-output modes the arm body should consume cur_x/cur_y/cur_z
           and cur_roll/cur_pitch/cur_yaw, then run its own IK. */
        dummy_feed_data.joint_motor[i].velocity = 0.0f;
        dummy_feed_data.joint_motor[i].current = 0.0f;
        dummy_feed_data.joint_motor[i].temperature = 0.0f;
        dummy_feed_data.joint_motor[i].is_online = 0U;
        dummy_feed_data.joint_motor[i].is_finished = 1U;
    }

    Fill_Controller_Feedback();
}

static void Fill_Direct_Joint_Output(void)
{
    last_valid_joint_cmd.a[0] = dummy_cmd_data.joint1_angle;
    last_valid_joint_cmd.a[1] = dummy_cmd_data.joint2_angle;
    last_valid_joint_cmd.a[2] = dummy_cmd_data.joint3_angle;
    last_valid_joint_cmd.a[3] = dummy_cmd_data.joint4_angle;
    last_valid_joint_cmd.a[4] = dummy_cmd_data.joint5_angle;
    last_valid_joint_cmd.a[5] = dummy_cmd_data.joint6_angle;

    Fill_Joint_Feedback(&last_valid_joint_cmd, 1U);
    Fill_Common_Feedback(ARM_STATUS_MOVING);
}

static void Dummy_Motormatic_Build_Kinematic_Config(ArmConfig_t *config)
{
    memset(config, 0, sizeof(ArmConfig_t));

    config->dh.a[0] = 0.0;
    config->dh.a[1] = 0.0;
    config->dh.a[2] = 0.250;
    config->dh.a[3] = -0.04347;
    config->dh.a[4] = 0.0;
    config->dh.a[5] = 0.0;

    config->dh.alpha[0] = 0.0;
    config->dh.alpha[1] = -M_PI / 2.0;
    config->dh.alpha[2] = 0.0;
    config->dh.alpha[3] = -M_PI / 2.0;
    config->dh.alpha[4] = M_PI / 2.0;
    config->dh.alpha[5] = -M_PI / 2.0;

    config->dh.d[0] = 0.0;
    config->dh.d[1] = 0.0;
    config->dh.d[2] = 0.0;
    config->dh.d[3] = 0.36349;
    config->dh.d[4] = 0.0;
    config->dh.d[5] = 0.06;

    config->pose_to_dh_scale = 0.001f;
    config->dh_to_pose_scale = 1000.0f;

    config->ik_reference_init_deg[0] = 0.0f;
    config->ik_reference_init_deg[1] = 75.0f;
    config->ik_reference_init_deg[2] = 105.0f;
    config->ik_reference_init_deg[3] = 0.0f;
    config->ik_reference_init_deg[4] = 0.0f;
    config->ik_reference_init_deg[5] = 0.0f;

    config->joint_to_motor.bias_deg[1] = 75.0f;
    config->joint_to_motor.bias_deg[2] = 180.0f;
    config->joint_to_motor.map[0][0] = 1.0f;
    config->joint_to_motor.map[1][1] = -1.0f;
    config->joint_to_motor.map[2][1] = -1.0f;
    config->joint_to_motor.map[2][2] = -1.0f;
    config->joint_to_motor.map[3][3] = 1.0f;
    config->joint_to_motor.map[4][4] = 1.0f;
    config->joint_to_motor.map[5][5] = 1.0f;

    config->motor_to_joint.bias_deg[1] = 75.0f;
    config->motor_to_joint.bias_deg[2] = 105.0f;
    config->motor_to_joint.map[0][0] = 1.0f;
    config->motor_to_joint.map[1][1] = -1.0f;
    config->motor_to_joint.map[2][1] = 1.0f;
    config->motor_to_joint.map[2][2] = -1.0f;
    config->motor_to_joint.map[3][3] = 1.0f;
    config->motor_to_joint.map[4][4] = 1.0f;
    config->motor_to_joint.map[5][5] = 1.0f;

    config->motor_limit[1].enable = true;
    config->motor_limit[1].min_deg = 0.0f;
    config->motor_limit[1].max_deg = 140.0f;
    config->motor_limit[2].enable = true;
    config->motor_limit[2].min_deg = -110.0f;
    config->motor_limit[2].max_deg = 40.0f;
    config->motor_limit[4].enable = true;
    config->motor_limit[4].min_deg = -130.0f;
    config->motor_limit[4].max_deg = 130.0f;

    // joint5 与 motor5 为 1:1 映射，可直接参与 IK 候选解筛选；q2/q3 优先由 motor/coupling 约束筛选。
    config->joint_limit[4].enable = true;
    config->joint_limit[4].min_deg = -130.0f;
    config->joint_limit[4].max_deg = 130.0f;

    config->coupling_limit_count = 1U;
    config->coupling_limits[0].enable = true;
    config->coupling_limits[0].lhs_motor_index = 2U;
    config->coupling_limits[0].rhs_motor_index = 1U;
    config->coupling_limits[0].min_delta_deg = -135.0f;
    config->coupling_limits[0].max_delta_deg = 0.0f;
    // motor_limit 与 coupling_limit 已在 IK 选解阶段作为硬约束，最终 clamp 只作为安全兜底。

    config->select_weight[0] = 7.0f;
    config->select_weight[1] = 6.0f;
    config->select_weight[2] = 5.0f;
    config->select_weight[3] = 4.0f;
    config->select_weight[4] = 3.0f;
    config->select_weight[5] = 2.0f;
    config->max_single_joint_jump_rad = 6.0f;
    config->max_total_cost = 20.0f;
    config->enable_q3_beyond_180_patch = true;

    config->controller_config.q1_initial_deg = 0.0f;
    config->controller_config.q2_initial_deg = 91.84504f;
    config->controller_config.q3_initial_deg = -88.727f;
    config->controller_config.q1_offset_deg = 27.0f;
    config->controller_config.q2_offset_deg = 28.85f;
    config->controller_config.q3_offset_deg = 18.99f;
    config->controller_config.q1_max_rad = 2.22398f;
    config->controller_config.q2_max_deg = 200.0f;
    config->controller_config.q3_max_deg = -190.029f;
    config->controller_config.q1_limit_torque = 5.0f;
    config->controller_config.delta_R_m = 0.090f;
    config->controller_config.delta_r_m = 0.035f;
    config->controller_config.delta_L_m = 0.120f;
    config->controller_config.delta_La_m = 0.270f;
    config->controller_config.xyz_delta_error_m[0] = -0.15f;
    config->controller_config.xyz_delta_error_m[1] = -0.01f;
    config->controller_config.xyz_delta_error_m[2] = -0.35f;
    config->controller_config.xyz_arm_error_m[0] = -0.122f;
    config->controller_config.xyz_arm_error_m[1] = 0.0f;
    config->controller_config.xyz_arm_error_m[2] = 0.108f;
    config->controller_config.xyz_scale[0] = 1.7f;
    config->controller_config.xyz_scale[1] = 1.5f;
    config->controller_config.xyz_scale[2] = 1.5f;
    config->controller_config.pitch_limit_deg = 60.0f;
}

void Dummy_Motormatic_Init(void)
{
    ArmConfig_t kinematic_config;

    memset(&dummy_cmd_data, 0, sizeof(dummy_cmd_data));
    memset(&dummy_feed_data, 0, sizeof(dummy_feed_data));
    memset(&current_pose, 0, sizeof(current_pose));
    memset(&target_pose, 0, sizeof(target_pose));
    dummy_cmd_data.arm_mode = ARM_FREE_MODE;
    dummy_cmd_data.arm_ctrl_mode = BIG_ARM_CTRL;
    dummy_cmd_data.gripper_mode = GRIPPER_RELEASE;

    Dummy_Motormatic_Build_Kinematic_Config(&kinematic_config);
    Kinematic_Init(&kinematic_handle, &kinematic_config);
    Kinematic_IKAngle_To_Motor(&kinematic_handle, &last_valid_ik_joint_deg, &last_valid_joint_cmd);
    Kinematic_Limit_Motor_Angle(&kinematic_handle, &last_valid_joint_cmd);

    Controller_Motor_Register();

    Fill_Joint_Feedback(&last_valid_joint_cmd, 1U);
    Fill_Common_Feedback(ARM_STATUS_IDLE);

    dummy_motormatic_pub = PubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
    dummy_motormatic_sub = SubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));

    is_pose_initialized = true;
}

void Dummy_Motormatic_Task(void)
{
    static uint8_t imu_zero_delay = 0U;

    if (!is_pose_initialized)
        return;

    SubGetMessage(dummy_motormatic_sub, &dummy_cmd_data);

    if (imu_delta != NULL && imu_zero_delay < 11U)
    {
        imu_zero_delay++;
        if (imu_zero_delay == 10U)
            IMU_Euler_Set_Zeros(imu_delta);
    }

    if (dummy_cmd_data.arm_mode == ARM_ZERO_FORCE)
    {
        Controller_Motor_Enable(false);
        Fill_Joint_Feedback(&last_valid_joint_cmd, 1U);
        Fill_Common_Feedback(ARM_STATUS_IDLE);
    }
    else if (dummy_cmd_data.arm_mode == ARM_FREE_MODE)
    {
        ControllerInput_t input;

        Controller_Motor_Enable(true);
        if (Dummy_Motormatic_Read_Controller_Input(&input) &&
            Kinematic_ControllerFK(&kinematic_handle, &input, &target_pose, controller_torque, controller_xyz_m))
        {
            current_pose = target_pose;
            Dummy_Motormatic_Apply_Controller_Torque(controller_torque);
            Fill_Pose_Only_Feedback(ARM_STATUS_MOVING);
        }
        else
        {
            Fill_Pose_Only_Feedback(ARM_STATUS_ERROR);
        }
    }
    else if (dummy_cmd_data.arm_mode == ARM_CARTESIAN_MODE)
    {
        Controller_Motor_Enable(false);
        Kinematic_Build_Pose(dummy_cmd_data.target_x,
                             dummy_cmd_data.target_y,
                             dummy_cmd_data.target_z,
                             dummy_cmd_data.target_roll,
                             dummy_cmd_data.target_pitch,
                             dummy_cmd_data.target_yaw,
                             &target_pose);
        current_pose = target_pose;
        Fill_Pose_Only_Feedback(ARM_STATUS_MOVING);
    }
    else if (dummy_cmd_data.arm_mode == ARM_PC_MODE)
    {
        Controller_Motor_Enable(false);
        Fill_Direct_Joint_Output();
    }

    if (imu_delta != NULL)
        IMU_Data_Request(imu_delta);

    PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
}
