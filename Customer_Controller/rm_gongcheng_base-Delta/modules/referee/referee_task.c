/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据

static char char_temp[10] = {0};  // 字符+终止符


uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    //RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_takemine_line[10]; // 取矿辅助线
// static Graph_Data_t UI_shoot_line[10]; // 射击准线
// static Graph_Data_t UI_Energy[3];      // 电容能量条
static Graph_Data_t UI_gesture_line[4]; // 工程整车简易姿态
static Graph_Data_t UI_gesture_line_move[1];
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
// static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
static uint32_t takemine_line_location[10] = {540};
static uint32_t gesture_line_location[10] = {300, 450, 500, 550};


void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制取矿基准线
    UILineDraw(&UI_takemine_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1025, 450, 1215, 450);
    UILineDraw(&UI_takemine_line[1], "sl2", UI_Graph_ADD, 8, UI_Color_Yellow, 3, 1215, 500, 1225, 600);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_takemine_line[0], UI_takemine_line[1]);

    // 绘制整车简易姿态
    // UILineDraw(&UI_gesture_line[0], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1300, gesture_line_location[3], 1450, gesture_line_location[3]);
    // UILineDraw(&UI_gesture_line[1], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1450.0-cos(PI/6), 1450-75, 1450, gesture_line_location[3]);
    // UILineDraw(&UI_gesture_line[2], "sl4", UI_Graph_ADD, 7, UI_Color_White, 3, 1375, 375, 1110, 450);
    UILineDraw(&UI_gesture_line[0], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1300+200, gesture_line_location[1]+100, 1450+200, gesture_line_location[1]+100);
    UILineDraw(&UI_gesture_line[1], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1300+200, gesture_line_location[0]+100, 1450+200, gesture_line_location[0]+100);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_gesture_line[0], UI_gesture_line[1]);
    UILineDraw(&UI_gesture_line[2], "sl7", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1300+200, gesture_line_location[0]+100, 1300+200, gesture_line_location[1]+100);
    UILineDraw(&UI_gesture_line[3], "sl8", UI_Graph_ADD, 7, UI_Color_Yellow, 3, 1450+200, gesture_line_location[0]+100, 1450+200, gesture_line_location[1]+100);
    // // UIGraphRefresh(&referee_recv_info->referee_id, 7, UI_gesture_line[0], UI_gesture_line[1], UI_gesture_line[2], UI_gesture_line[3], UI_gesture_line[4], UI_gesture_line[5], UI_gesture_line[6]);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_gesture_line[2], UI_gesture_line[3]);

    UILineDraw(&UI_gesture_line_move[0], "sl14", UI_Graph_ADD, 9, UI_Color_Green, 3, 1375, 375, 1110, 450);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_gesture_line_move[0]);

    // // 绘制车辆状态标志指示
    // UICharDraw(&UI_State_sta[3], "ss4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "suker-f:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    // UICharDraw(&UI_State_sta[4], "ss5", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "suker-l:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);
    // UICharDraw(&UI_State_sta[5], "ss6", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "suker-r:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);

    // // 绘制车辆状态标志，动态
    // // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新

    UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 1500, 700, "gimbal_mode:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 1500, 650, "trail_mode:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd3", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 1500, 600, "trail_mode_num:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);

    UICharDraw(&UI_State_dyn[3], "sd4", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 50, 700, "suker-f:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd5", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 50, 650, "suker-l:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    UICharDraw(&UI_State_dyn[5], "sd6", UI_Graph_ADD, 8, UI_Color_White, 28, 5, 50, 600, "suker-r:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);

}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->suker_final_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->suker_final_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->suker_final_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->suker_final_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Main, 28, 5, 1500, 700, "zeroforce");
            break;
        case GIMBAL_FREE_MODE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Main, 28, 5, 1500, 700, "free_mode");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case GIMBAL_CUS_MODE:
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Main, 28, 5, 1500, 700, "cus_mode");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    // 整车简易姿态
    if (_Interactive_data->Referee_Interactive_Flag.gestrue_flag == 1)
    {
        // UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "armmove");
        // UILineDraw(&UI_gesture_line_move[0], "s14", UI_Graph_Change, 9, UI_Color_Green, 3, 1375, 375, (1375.0+75.0*cos(_Interactive_data->gesture_data.yaw*360.0/PI)), (375.0+75.0*sin(_Interactive_data->gesture_data.yaw * 360.0/PI)));
        // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_gesture_line_move[0]);
        UILineDraw(&UI_gesture_line_move[0], "sl14", UI_Graph_Change, 9, UI_Color_Green, 3, 1375+200, 375+100, (1375.0+75.0*sin(_Interactive_data->gesture_data.yaw))+200, (375.0+75.0*cos(_Interactive_data->gesture_data.yaw))+100);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_gesture_line_move[0]);
    
        // UILineDraw(&UI_gesture_line[0], "s12", UI_Graph_Change, 8, UI_Color_White, 3, (uint32_t)(1300.0+150.0*cos(_Interactive_data->gesture_data.pitch1)), (uint32_t)(500.0+150.0*sin(_Interactive_data->gesture_data.pitch1)), 
        // (uint32_t)(1300.0+150.0*(cos(_Interactive_data->gesture_data.pitch1)+cos(_Interactive_data->gesture_data.pitch1+_Interactive_data->gesture_data.pitch2 * 360.0 / PI))), (uint32_t)(500.0+150.0*(sin(_Interactive_data->gesture_data.pitch1)+sin(_Interactive_data->gesture_data.pitch1+_Interactive_data->gesture_data.pitch2))));
        // UIGraphRefresh(&referee_recv_info->referee_id, UI_gesture_line[1]);

        // UILineDraw(&UI_gesture_line[1], "s13", UI_Graph_Change, 8, UI_Color_White, 3, 1300, 500, (uint32_t)(1300.0+150.0*cos(_Interactive_data->gesture_data.pitch1)), (uint32_t)(500.0+150.0*sin(_Interactive_data->gesture_data.pitch1)));
        // UIGraphRefresh(&referee_recv_info->referee_id, UI_gesture_line[1]);

        // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.gestrue_flag = 0;
    }
    
    // suker
    if (_Interactive_data->Referee_Interactive_Flag.left_flag == 1)
    {
        UICharDraw(&UI_State_dyn[4], "sd5", UI_Graph_Change, 8, UI_Color_Orange, 28, 5, 50, 650, _Interactive_data->suker_restore_mode_l == SHOOT_ON ? "open" : "close");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.right_flag = 0;
    }    

    if (_Interactive_data->Referee_Interactive_Flag.final_flag == 1)
    {
        UICharDraw(&UI_State_dyn[3], "sd4", UI_Graph_Change, 8, UI_Color_Orange, 28, 5, 50, 700, _Interactive_data->suker_final_mode == SHOOT_ON ? "open" : "close");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.final_flag = 0;
    }

    if(_Interactive_data->Referee_Interactive_Flag.trail_flag == 1)
    {
        UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 28, 5, 1500, 650, _Interactive_data->trail_mode == TRAIL_PLAY ? "play" : "pause");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.trail_flag = 0;
    }

    // if(_Interactive_data->Referee_Interactive_Flag.trail_mode_flag == 1)
    // {
    //     sprintf(char_temp, "%c", (char)_Interactive_data->trail_mode_num + 48);

    //     UICharDraw(&UI_State_dyn[2], "sd3", UI_Graph_Change, 8, UI_Color_Orange, 28, 5, 1500, 600, char_temp);
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    //     _Interactive_data->Referee_Interactive_Flag.trail_mode_flag = 0;
    // }

    // if (_Interactive_data->Referee_Interactive_Flag.left_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[5], "sd6", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 50, 600, _Interactive_data->suker_restore_mode_r == SHOOT_ON ? "open" : "close");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    //     _Interactive_data->Referee_Interactive_Flag.left_flag = 0;
    // }
}


/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if(_Interactive_data->suker_final_mode != _Interactive_data->last_suker_final_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.final_flag = 1;
        _Interactive_data->last_suker_final_mode = _Interactive_data->suker_final_mode;
    }

    if (_Interactive_data->suker_restore_mode_l != _Interactive_data->last_suker_restore_mode_l)
    {
        _Interactive_data->Referee_Interactive_Flag.left_flag = 1;
        _Interactive_data->last_suker_restore_mode_l = _Interactive_data->suker_restore_mode_l;
    }

    if (_Interactive_data->trail_mode != _Interactive_data->last_trail_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.trail_flag = 1;
        _Interactive_data->last_trail_mode = _Interactive_data->trail_mode;
    }

    if (_Interactive_data->trail_mode_num != _Interactive_data->last_trail_mode_num)
    {
        _Interactive_data->Referee_Interactive_Flag.trail_mode_flag = 1;
        _Interactive_data->last_trail_mode_num = _Interactive_data->trail_mode_num;
    }

    // if (_Interactive_data->suker_restore_mode_r != _Interactive_data->last_suker_restore_mode_r)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.right_flag = 1;
    //     _Interactive_data->last_suker_restore_mode_l = _Interactive_data->suker_restore_mode_r;
    // }

    if((_Interactive_data->gesture_data.pitch1 != _Interactive_data->last_gesture_data.pitch1) || (_Interactive_data->gesture_data.pitch2 != _Interactive_data->last_gesture_data.pitch2) || (_Interactive_data->gesture_data.yaw != _Interactive_data->last_gesture_data.yaw))
    {
        _Interactive_data->Referee_Interactive_Flag.gestrue_flag = 1;
        _Interactive_data->last_gesture_data.pitch1 =  _Interactive_data->gesture_data.pitch1;
        _Interactive_data->last_gesture_data.pitch2 =  _Interactive_data->gesture_data.pitch2;
        _Interactive_data->last_gesture_data.yaw =  _Interactive_data->gesture_data.yaw;
   
    }
}
