#ifndef __CUSTOM_CONTROLLER_H
#define __CUSTOM_CONTROLLER_H
#include "bsp_usart.h"
typedef struct
{
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;
    uint8_t custom_flag;
    uint16_t data[5];
} Custom_data_t;
// 机器人回传数据暂定,后续可以更改
#pragma pack(1)
typedef struct
{
    float offset_x;
    float offset_y;
    float offset_z;         // 12
    uint8_t re_custom_flag; // 机器人回传flag -- "re"词根表示相反 =)/1
    uint8_t temp[17];
} Re_Custom_data_t;
#pragma pack()
/* 帧头 */
typedef struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;

} frame_header;

/* 机器人交互数据，发送方触发发送，频率上限为 10Hz 0x0301 */
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];

} robot_interaction_data_t;

/* 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz 0x0302 */
typedef struct
{
    uint8_t data[30];

} custom_robot_data_t;

/* 键鼠遥控数据，固定 30Hz 频率发送 0x0304 */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;

} remote_control_t;

/* 自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz 0x0306 */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;

} Custom_client_data_t;

/* 图传链路 */
typedef struct
{
    // 图传串口
    uint8_t data_seq;                       // 数据序列号
    uint8_t data_send_buf[50];              // 发送数据缓存
    uint8_t data_recb_buf[256];              // 接收数据缓存
    frame_header Frame_header;              // 帧头
    uint16_t cmd_id;                        // 命令ID
    custom_robot_data_t CustomControl_Data; // 自定义控制器与机器人交互数据 0x0302
    remote_control_t remote_control;        // 键鼠遥控数据 0x0304
    USARTInstance *uart_ins;
} Video_transmission_link_struct;

extern Video_transmission_link_struct Custom_Controller;
extern Video_transmission_link_struct Custom_Controller_;

Video_transmission_link_struct *Tran_Link_Init(UART_HandleTypeDef *HUART);
void Tran_Link_customdata_float_to_hex(Video_transmission_link_struct *custom_controller,
                                       const Custom_data_t data);
void Tran_Link_recustomdata_float_to_hex(Video_transmission_link_struct *custom_controller,
                                         Re_Custom_data_t *data);

void Custom_Task(void const *argument);
void Custom_Task_Init(void);
Re_Custom_data_t *Recustom_get(void);
#endif // !__CUSTOM_CONTROLLER_H