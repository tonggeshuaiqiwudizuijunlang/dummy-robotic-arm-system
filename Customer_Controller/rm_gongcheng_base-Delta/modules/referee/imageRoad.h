/**
 * @file imageRoad.h
 * @author Jiaxing He (1756735553@qq.com)
 * @version 0.1
 * @date 2024-10-20
 *
 */
#ifndef imageRoad_H
#define imageRoad_H

#include "remote_control.h"
#include "stdint.h"
/* ID: 0x0304  Byte:  12    图传链路 */
typedef  struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
}remote_control_t;

/* ID: 0x0302  Byte:  12   自定义控制器数据 *///30？？
/** 
 * @brief 自定义控制器的结构体
 */
typedef struct
{
	float theta1;
	float theta2;
	float theta3;
	float theta4;
	float theta5;
	float theta6;
	uint8_t flag;
	uint8_t data[5];
} custom_robot_data_t;


typedef struct
{
    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍
    uint8_t key_count[3][16];
} ImageRoadRC;


remote_control_t *ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle);
void Get_Key(ImageRoadRC* KEY);
uint8_t Get_imgroad_lostflag(void);

#endif
