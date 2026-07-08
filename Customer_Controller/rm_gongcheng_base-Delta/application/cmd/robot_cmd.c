// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "vision.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "bmi088.h"
#include "referee_protocol.h"
#include "referee_task.h"

#include "fly/flysky.h"
#include "delta_cal/delta_cal.h"
#include "custom/custom_controller.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// algorithm
#include "user_lib.h"

#define AVERGEFILTERLEN 40 // 均值滤波长度，用于wz的滤波

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define armor_f (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define armor_l (1118 * ECD_ANGLE_COEF_DJI)
#define armor_r (5120 * ECD_ANGLE_COEF_DJI)
#define armor_b (3140 * ECD_ANGLE_COEF_DJI)

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static FS_ctrl_t *fs_data;
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static Custom_data_t custom_data;
static Re_Custom_data_t *re_custom_data;

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

static referee_info_t *referee_info;

static uint8_t cali_count = 0;

//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
//* 函数声明////////////////////////////////* 函数声明//////////////////////////////

Video_transmission_link_struct *Video_link;
static void Angle_Limit_demo_test(Gimbal_Ctrl_Cmd_s *_gimbal_cmd_send);
static void Delta_Controller(Gimbal_Ctrl_Cmd_s *_gimbal_cmd_send);

//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
//* 函数声明////////////////////////////////* 函数声明//////////////////////////////
void RobotCMDInit()
{
    // fs_data = FSControlInit(&huart3);
    // rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    //* 自定义控制器数据
    
    Video_link = Tran_Link_Init(&huart1);
    re_custom_data = Recustom_get();
    //* 机械臂cmd发布者 ————这里就是注册遥控器，键盘的topic
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    //* 机械臂反馈订阅者
    //* 主要是两个imu，
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    //* 吸盘cmd发布者
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    //* 吸盘反馈订阅者
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

    // referee_data = UITaskInit(&huart1, &ui_data); // 裁判系统初始化,会同时初始化UI
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle * RAD_2_DEGREE; // 从云台获取的当前yaw电机单圈角度
    angle /= 2.0;                                                          // 减速比二比一，所以传回底盘实际的yaw值要除二//用于计算前进方向
#if YAW_ECD_GREATER_THAN_4096                                              // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    // if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    // else if (angle > YAW_ALIGN_ANGLE - 180.0f)
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    // else
    //     chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;

#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    static uint8_t num = 0; // 设定不同电机的任务频率
    //* 启动重力补偿的
    if (fs_data[TEMP].switch_r2 == FS_SW_UP)
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    else if (fs_data[TEMP].switch_r2 == FS_SW_DOWN)
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
}

/**
 * @brief 输入为图传链路时模式和控制量设置
 *
 */

static void ImageRoadSet()
{
    static uint16_t num = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    custom_data.theta1 = gimbal_fetch_data.q1;
    custom_data.theta2 = gimbal_fetch_data.q2;
    custom_data.theta3 = gimbal_fetch_data.q3;
    custom_data.theta4 = gimbal_fetch_data.q4;
    custom_data.theta5 = gimbal_fetch_data.q5;
    custom_data.theta6 = gimbal_fetch_data.q6;
    Tran_Link_customdata_float_to_hex(Video_link, custom_data);
    //* 接收机器人回传数据，移动世界基坐标系
    gimbal_cmd_send.offset_x = re_custom_data->offset_x;
    gimbal_cmd_send.offset_y = re_custom_data->offset_y;
    gimbal_cmd_send.offset_z = re_custom_data->offset_z;
    gimbal_cmd_send.get_error_flag = re_custom_data->re_custom_flag;
}
/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    // if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    // if (switch_is_up(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) // 右侧开关为[下],急停
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     // shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     // shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     // shoot_cmd_send.load_mode = LOAD_STOP;
    //     LOGERROR("[CMD] emergency stop!");
    // }
    // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        robot_state = ROBOT_READY;
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    // referee_info = Get_referee_info();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();


    ImageRoadSet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
