#include "bsp_flash.h"
#include "bsp_usart.h"
#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "XLmotor.h"
#include "dmmotor.h"
#include "RSmotor/rs_motor.h"
#include "buzzer.h"
#include "user_lib.h"
#include "DM_IMU.h"
#include "delta_cal/delta_cal.h"
#include "Arm_Moveit/arm_moveit.h"
#include "stdlib.h"
//^ 并且所有的角度均为角度值，只有在DM最后输入给电机的时候才会转变为弧度制
//^ q1是上边，
//^ q2是左边， 向外为正
//^ q3是右边 向外为负

#define Q1_INITIAL_ANGLE 0.00f
#define Q2_INITIAL_ANGLE 91.84504f
#define Q3_INITIAL_ANGLE -88.727f

#define Q1_MAX_ANGLE 2.22398f
#define Q2_MAX_ANGLE 200.f
#define Q3_MAX_ANGLE -190.029f

#define Q1_INITAL_OFFEST_ANGLE 27.f
#define Q2_INITAL_OFFEST_ANGLE 28.85f
#define Q3_INITAL_OFFEST_ANGLE 18.99f

#define Q1_LIMIT_POS_TOR 5.f

#define Q2_OFFEST 15.f // 15
#define Q3_OFFSET 180.f
#define Q4_MAX 180.f
#define Q4_MIN -180.f
#define Q5_MIN -130
#define Q5_MAX 130
#define Q6_MIN -180
#define Q6_MAX 180

#define ALPHA_MAX 165.f // 165 这个地方调小一点可以防止极限位置碰撞
#define ALPHA_MIN 30.f  // 15
//^ 我需要记录其最低点的xyz，以及保证这个最低点是一个恒定的位置
//^
static attitude_t *gimbal_IMU_data; // 云台IMU数据
// new
static RSMotorInstance *q1;
static DJIMotorInstance *q2, *q3;
static DM_IMU_Instance_s *imu_delta;
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv_flash;   // 来自flash的控制信息
static BMI088Instance *bmi088;                    // 云台IMU
static BuzzzerInstance *gimbal_buzzer;
static JointAngles q_answer = {0};
static InverseKinematicsSolutions q_value = {0};
static JointAngles q_current = {0, 75 * DEGREE_2_RAD, 105 * DEGREE_2_RAD, 0, 0, 0};
static JointAngles q_current_debug = {0, 75 * DEGREE_2_RAD, 105 * DEGREE_2_RAD, 0, 0, 0};
static float xyz[4] = {0};
static float xyz_debug[3] = {0};

static float theta[4] = {0};
static float tor[4] = {0};
static void Gravity_compensation_fixed_fit_appliction(void);
static void scale_pitch(DM_IMU_Instance_s *imu);
static double *Robot_ik(Vector3 *_end_pos,
                        Matrix3x3 *_end_rotation,
                        DHParameters *_dh_params);
static void IK_Angle_Translate(JointAngles *_joint_angles, double *q_send);
void GimbalLimit(double *q);
void GimbalInit()
{
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    IMU_CAN_Init_Config_s imu_delta_config = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x22,
            .rx_id = 0x22, // IMU的发送id
        }};

    //* ///////////////////////////begin
    Motor_Init_Config_s q1_rs_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0xfd},
        .motor_type = RS05,
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
    };
    // DJI6020
    Motor_Init_Config_s q2_6020_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x04,
            .rx_id = 0x04,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 15, // 10
                .Ki = 0,
                .Kd = 2,
                .MaxOut = 5500,
                .DeadBand = 1,
            },
            .speed_PID = {
                .Kp = 15, // 10
                .Ki = 1,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,              // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP, //^ 增加了一个电流环
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    Motor_Init_Config_s q3_6020_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x02,
            .rx_id = 0x02,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 10
                .Ki = 0,
                .Kd = 2,
                .MaxOut = 5500,
                .DeadBand = 1,
            },
            .speed_PID = {
                .Kp = 15, // 10
                .Ki = 1,  // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_HIGH, // 设置警报等级 同一状态下 高等级的响应
        .loudness = 0.4,                 // 设置响度
        .octave = OCTAVE_1,              // 设置音阶
    };

    //* ///////////////////////////end
    gimbal_buzzer = BuzzerRegister(&buzzer_config);
    AlarmSetStatus(gimbal_buzzer, ALARM_ON);
    q1 = RSMotorInit(&q1_rs_config);
    q2 = DJIMotorInit(&q2_6020_config);
    q3 = DJIMotorInit(&q3_6020_config);
    imu_delta = DM_IMU_Init(&imu_delta_config);
    IMU_Mode_Set(imu_delta, DM_IMU_DataType_euler);
    IMU_Data_Request(imu_delta);
    DWT_Delay(1);
    IMU_Data_Request(imu_delta);
    RSMotorCaliEncoder(q1);
    DJiMotorSetMode(q2, CURRENT_MODE);
    DJiMotorSetMode(q3, CURRENT_MODE);
    // 云台消息发布者和订阅者
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
static float rotation[4][4] = {0};
const static DHParameters dh_params = {
    .a = {0, 0, 0.250, -0.04347, 0, 0},
    .alpha = {0, -pi / 2, 0, -pi / 2, pi / 2, -pi / 2},
    .d = {0, 0, 0, 0.36349, 0, 0.06},
};
const float xyz_delta_error[4] = {0, -0.15, -0.01, -0.35};
const float xyz_arm_error[4] = {0, -0.122, 0, 0.108};
/* 机器人机械臂控制核心任务 */
void GimbalTask()
{

    double *motor_q;
    static uint8_t num = 0;

    static Vector3 __pos;
    static Matrix3x3 __rotation;
    static float offset_x_test = 0;
    static float offset_y_test = 0;
    static float offset_z_test = 0;
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    offset_x_test = gimbal_cmd_recv.offset_x;
    offset_y_test = gimbal_cmd_recv.offset_y;
    offset_z_test = gimbal_cmd_recv.offset_z;
    
    if (num < 10)
    {
        num++;
    }
    else if (num == 10)
    {
        IMU_Euler_Set_Zeros(imu_delta);
        AlarmSetStatus(gimbal_buzzer, ALARM_OFF);
        num = 11;
    }

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        RSMotorStop(q1);
        DJIMotorStop(q2);
        DJIMotorStop(q3);
        break;
    case GIMBAL_FREE_MODE: // 自由模式
        RSMotorEnable(q1);
        DJIMotorEnable(q2);
        DJIMotorEnable(q3);
        // todo :需要修改
        scale_pitch(imu_delta);

        Rotation_ZYX_Matrix(rotation,
                            imu_delta->IMU_Driver_Data.yaw,
                            -imu_delta->IMU_Driver_Data.pitch + 90.f,
                            imu_delta->IMU_Driver_Data.roll + 180.f);
        Gravity_compensation_fixed_fit_appliction();
        RSMotorSetRef(q1, tor[1] * 0.4f);
        DJIMotorSetRef(q2, -tor[2] * 5461.333f * 1000.f * 1.08f / 741.f);
        DJIMotorSetRef(q3, tor[3] * 5461.333f * 1000.f * 1.1f / 741.f);
        gimbal_cmd_recv.q1_tor = xyz[1];
        gimbal_cmd_recv.q2_tor = xyz[2];
        gimbal_cmd_recv.q3_tor = xyz[3];
        __pos.x = -(xyz[3] - xyz_delta_error[3]);
        __pos.y = xyz[2] - xyz_delta_error[2];
        __pos.z = xyz[1] - xyz_delta_error[1];
        xyz_debug[0] = __pos.x;
        xyz_debug[1] = __pos.y;
        xyz_debug[2] = __pos.z;
        __pos.x += xyz_arm_error[1];
        __pos.y += xyz_arm_error[2];
        __pos.z += xyz_arm_error[3];

        __pos.x *= 1.7;
        __pos.y *= 1.5;
        __pos.z *= 1.5;
        __pos.y *= -1; // 偏移应该在缩放之前应用，保证物理坐标系中的偏移正确性
        __pos.x -= offset_x_test;
        __pos.y += offset_y_test;
        __pos.z += offset_z_test;
        Rotation_4x4_To_3x3(rotation, &__rotation);
        motor_q = Robot_ik(&__pos, &__rotation, &dh_params);
        gimbal_feedback_data.q1 = (float)motor_q[0];
        gimbal_feedback_data.q2 = (float)motor_q[1];
        gimbal_feedback_data.q3 = (float)motor_q[2];
        gimbal_feedback_data.q4 = (float)motor_q[3];
        gimbal_feedback_data.q5 = (float)motor_q[4];
        gimbal_feedback_data.q6 = (float)motor_q[5];
        break;
    default:
        break;
    }
  
        // 推送消息
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    IMU_Data_Request(imu_delta);
}

static void Gravity_compensation_fixed_fit_appliction(void)
{

    theta[1] = (-q1->measure.position) - Q1_INITAL_OFFEST_ANGLE * DEGREE_2_RAD;
    theta[2] = (q2->measure.total_angle - Q2_INITIAL_ANGLE - Q2_INITAL_OFFEST_ANGLE) * DEGREE_2_RAD;
    theta[3] = (-(q3->measure.total_angle - Q3_INITIAL_ANGLE) - Q3_INITAL_OFFEST_ANGLE) * DEGREE_2_RAD;
    Gravity_compensation(theta, tor, xyz);
    if (theta[1] > Q1_MAX_ANGLE - Q1_INITAL_OFFEST_ANGLE * DEGREE_2_RAD)
        tor[1] = Q1_LIMIT_POS_TOR;
    if (theta[2] > (Q2_MAX_ANGLE - Q2_INITIAL_ANGLE - Q2_INITAL_OFFEST_ANGLE) * DEGREE_2_RAD)
        tor[2] *= 0.3f;
    if (theta[3] > (-(Q3_MAX_ANGLE - Q3_INITIAL_ANGLE) - Q3_INITAL_OFFEST_ANGLE) * DEGREE_2_RAD)
        tor[3] *= 0.3f;
}
#define PITCH_ANGLE_LIMIT (60.f)
/**
 * @brief 防止万向锁
 * @note 将imu的pitch角度缩放从而防止imu硬件上的万向锁
 */
static void scale_pitch(DM_IMU_Instance_s *imu)
{
    if (imu->IMU_Driver_Data.pitch > PITCH_ANGLE_LIMIT)
    {
        AlarmSetStatus(gimbal_buzzer, ALARM_ON);
        imu->IMU_Driver_Data.pitch = PITCH_ANGLE_LIMIT;
    }
    else if (imu->IMU_Driver_Data.pitch < -PITCH_ANGLE_LIMIT)
    {
        imu->IMU_Driver_Data.pitch = -PITCH_ANGLE_LIMIT;
        AlarmSetStatus(gimbal_buzzer, ALARM_ON);
    }
    else
    {
        AlarmSetStatus(gimbal_buzzer, ALARM_OFF);
    }
    imu->IMU_Driver_Data.pitch = imu->IMU_Driver_Data.pitch / PITCH_ANGLE_LIMIT * 90.f;
}
static double *Robot_ik(Vector3 *_end_pos, Matrix3x3 *_end_rotation, DHParameters *_dh_params)
{
    static int best_index = 0;
    static double q_robot_motor_send[6] = {0};
    best_index = -1;
    memset(&q_value, 0, sizeof(q_value));
    q_value = inverse_kinematics_from_rotation_matrix(_end_pos, _end_rotation, _dh_params);
    best_index = select_best_solution(&q_value, &q_current);

    if (best_index == -1 || best_index == 255)
    {
        q_answer.theta[0] = q_current.theta[0];
        q_answer.theta[1] = q_current.theta[1];
        q_answer.theta[2] = q_current.theta[2];
        q_answer.theta[3] = q_current.theta[3];
        q_answer.theta[4] = q_current.theta[4];
        q_answer.theta[5] = q_current.theta[5];
        //* 角度转换
        IK_Angle_Translate(&q_answer, q_robot_motor_send);
        // //* 角度限制防止越界
        GimbalLimit(q_robot_motor_send);
        return q_robot_motor_send;
    }
    fit_angle_solution(&q_value.angles[best_index], &q_current);
    q_current.theta[0] = q_answer.theta[0] = q_value.angles[best_index].theta[0];
    q_current.theta[1] = q_answer.theta[1] = q_value.angles[best_index].theta[1];
    q_current.theta[2] = q_answer.theta[2] = q_value.angles[best_index].theta[2];
    q_current.theta[3] = q_answer.theta[3] = q_value.angles[best_index].theta[3];
    q_current.theta[4] = q_answer.theta[4] = q_value.angles[best_index].theta[4];
    q_current.theta[5] = q_answer.theta[5] = q_value.angles[best_index].theta[5];
    q_current_debug.theta[1] = q_current.theta[1] * RAD_2_DEGREE;
    q_current_debug.theta[2] = q_current.theta[2] * RAD_2_DEGREE;

    //* 角度转换
    IK_Angle_Translate(&q_answer, q_robot_motor_send);
    // //* 角度限制防止越界
    GimbalLimit(q_robot_motor_send);
    return q_robot_motor_send;
}
/**
 * @brief 自定义控制器角度和电机角度关联函数
 * @param  _gimbal_cmd_send
 * @param  _custom_cms_rev
 * @note   这里的角度单位全部是度
 * @note 传入的全是弧度，变换角度之后全为度
 * @attention 我这里都还没考虑超过180的限位情况
 * @note 具体怎么转换的，详情参见gc解释😊
 */
static void IK_Angle_Translate(JointAngles *_joint_angles, double *q_send)
{
    static double q[6] = {0};
    memset(q, 0, sizeof(q));

    //^ translate
    // q1
    q[0] = _joint_angles->theta[0] * RAD_2_DEGREE;

    // q2
    q[1] = -_joint_angles->theta[1] * RAD_2_DEGREE + 75.f;

    // q3
    q[2] = -105.f - q[1] + _joint_angles->theta[2] * RAD_2_DEGREE;
    q[2] *= -1; // 取反，在我所需要的空间中q3是反向的
    q[3] = _joint_angles->theta[3] * RAD_2_DEGREE;
    q[4] = _joint_angles->theta[4] * RAD_2_DEGREE;
    q[5] = _joint_angles->theta[5] * RAD_2_DEGREE;
    memcpy(q_send, q, sizeof(q));
}
/**
 * @brief 前两轴限位
 * @note 这里的限位单位全部都是度
 * @note q2 q3 做了一个相对角度偏移
 */
void GimbalLimit(double *q)
{
    // 参数设置
    static float q2_q3_sum_max = ALPHA_MAX - (Q3_OFFSET - Q2_OFFEST); // 0
    static float q2_q3_sum_min = ALPHA_MIN - (Q3_OFFSET - Q2_OFFEST); //-135
    // 后面加正运动学，计算出末端的姿态和图传的距离

    // 这里是q2 q3的角度
    // q2
    if (q[1] > 140.0f) // 极限是0~150
        q[1] = 140.0f;
    if (q[1] < 0.0f)
        q[1] = 0.0f;

    if (q[2] < -110.0) // 这个极限还不知道
        q[2] = -110.0;
    if (q[2] > 40)
        q[2] = 40;

    if (q[2] - q[1] > q2_q3_sum_max)
        q[2] = q2_q3_sum_max + q[1];
    if (q[2] - q[1] < q2_q3_sum_min)
        q[2] = q2_q3_sum_min + q[1];
    if (q[4] <= Q5_MIN)
        q[4] = Q5_MIN;
    else if (q[4] >= Q5_MAX)
        q[4] = Q5_MAX;
}

static Vector3 *sensitivity_controller__delta_output(Vector3 *_pos)
{
    static float deadzone[3] = {0},
                 expo[3] = {0.8, 0.8, 0.8},
                 sensitivity[3] = {1, 1, 1};
    static float __pos[3] = {0};
    __pos[0] = _pos->x;
    __pos[1] = _pos->y;
    __pos[2] = _pos->z;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (abs(__pos[i]) < deadzone[i])
        {
            __pos[i] = 0;
            continue;
        }
        __pos[i] = __pos[i] * (1 + expo[i] * (abs(__pos[i]) - 1));
        __pos[i] = __pos[i] * sensitivity[i];
    }
    _pos->x = __pos[0];
    _pos->y = __pos[1];
    _pos->z = __pos[2];
    return _pos;
}