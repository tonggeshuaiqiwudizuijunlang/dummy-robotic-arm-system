#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "sukermotor.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static Suker_Instance *suker_final_motor, *suker_left_motor, *suker_right_motor;
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;

void ShootInit()
{
    // 末端吸盘
    Motor_Init_Config_s final_config = 
    {
        .can_init_config = 
        {
            .can_handle = &hcan1,
            .tx_id =0x05,
            .rx_id =0x05,
        },
        .controller_setting_init_config = 
        {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = Sukermotor,
    };

    // 左存矿吸盘
    Motor_Init_Config_s left_config = 
    {
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id =0x15,
            .rx_id =0x15,
        },
        .controller_setting_init_config = 
        {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = Sukermotor,
    };

    // 右存矿吸盘
    Motor_Init_Config_s right_config = 
    {
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id =0x16,
            .rx_id =0x16,
        },
        .controller_setting_init_config = 
        {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = Sukermotor,
    };

    suker_final_motor = Suker_Motor_Init(&final_config);
    suker_left_motor = Suker_Motor_Init(&left_config);
    suker_right_motor = Suker_Motor_Init(&right_config);
    
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
}