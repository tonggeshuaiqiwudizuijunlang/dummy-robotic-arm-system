#ifndef __NEW_IMAGEROAD_H
#define __NEW_IMAGEROAD_H

#include "rm_referee.h"
#include "robot_def.h"
#include "remote_control.h"
#include "imageRoad.h"

#define IM_RC_SW_LEFT ((uint16_t)0)  // 开关向上时的值
#define IM_RC_SW_MID ((uint16_t)1)   // 开关中间时的值
#define IM_RC_SW_RIGHT ((uint16_t)2) // 开关向下时的值

// 三个判断开关状态的宏
#define Switch_Is_Left(t) (t == IM_RC_SW_LEFT)
#define Switch_Is_Mid(t) (t == IM_RC_SW_MID)
#define Switch_Is_Right(t) (t == IM_RC_SW_RIGHT)

#pragma pack(1)
typedef struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    uint16_t key;

    uint16_t crc;

} remote_data_t;
#pragma pack()
typedef struct
{
    int64_t rocker_l_;
    int64_t rocker_l1;
    int64_t rocker_r_;
    int64_t rocker_r1;

    uint8_t mode_sw : 2;      // 挡位切换开关
    uint8_t pause : 1;        // 暂停按键
    uint8_t left_buttom : 1;  // 左自定义按键
    uint8_t right_buttom : 1; // 右自定义按键
    uint64_t wheel : 11;      // 左拨轮
    uint8_t trigger : 1;      // 右扳机

    // int16_t mouse_x;
    // int16_t mouse_y;
    // int16_t mouse_z;    //是滚轮的值！
    // uint8_t left_button_down;
    // uint8_t right_button_down;
    // uint8_t mouse_middle;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍
    uint8_t key_count[3][16];
} New_ImageRoadRC;

remote_data_t *New_ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle);
uint8_t Get_img_lostflag(void);
uint8_t Get_cus_lostflag(void);
void Get_Key_New(New_ImageRoadRC *KEY);
custom_robot_data_t *Get_custom_controller_data();

#endif
