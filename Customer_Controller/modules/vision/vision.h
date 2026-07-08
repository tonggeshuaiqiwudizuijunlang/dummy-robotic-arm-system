#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include "bsp_uart.h"

// 保持1字节对齐，防止编译器自动插入填充字节导致结构体大小不匹配
#pragma pack(1)
typedef struct {
    uint8_t header; // 固定为0xAA

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;

    // uint8_t gripper_state; // 0:关闭 1:打开
    uint16_t tailer; // 固定为0xFFFB   
} Received_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
    uint8_t is_finished; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB   
} Transmit_Data_s;

#pragma pack()

Received_Data_s* Vision_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);

// 获取解析完毕的上位机指令数据
Received_Data_s* Vision_Get_TargetJoints(void);
uint8_t Vision_Is_Online(void);

// 提供给外界向电脑（上位机）发送数据的接口
void Vision_Send_Data(uint8_t* tx_data, uint16_t len);

#endif // VISION_H