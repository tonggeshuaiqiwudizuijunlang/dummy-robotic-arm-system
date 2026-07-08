#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include "bsp_uart.h"

#define BT_TX_MODE_JOINT       0u
#define BT_TX_MODE_POSE        1u
#define BT_TX_MODE_DEBUG_POSE  BT_TX_MODE_POSE

#ifndef BT_TX_PACKET_MODE
#define BT_TX_PACKET_MODE BT_TX_MODE_POSE
#endif

// 保持1字节对齐，防止编译器自动插入填充字节导致结构体大小不匹配
#pragma pack(1)
typedef struct {
    uint8_t header; // 固定为0xAA

    float joint1; // 目标关节1角度，单位 deg
    float joint2; // 目标关节2角度，单位 deg
    float joint3; // 目标关节3角度，单位 deg
    float joint4; // 目标关节4角度，单位 deg
    float joint5; // 目标关节5角度，单位 deg
    float joint6; // 目标关节6角度，单位 deg

    uint8_t gripper_state; // 0:关闭 1:打开

    uint16_t tailer; // 固定为0xFFFB
} RX_BT_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;

    uint8_t is_finished; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB
} TX_BT_Joint_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float x;     // mm
    float y;     // mm
    float z;     // mm
    float roll;  // deg, Roll around X
    float pitch; // deg, Pitch around Y
    float yaw;   // deg, Yaw around Z

    uint8_t auto_gripper; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB
} TX_BT_Pose_Data_s;

typedef TX_BT_Pose_Data_s TX_BT_Debug_Pose_Data_s;

#if (BT_TX_PACKET_MODE == BT_TX_MODE_POSE)
typedef TX_BT_Pose_Data_s TX_BT_Data_s;
#else
typedef TX_BT_Joint_Data_s TX_BT_Data_s;
#endif
#pragma pack()

RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
bool BT_SendData(uint8_t* tx_data, uint16_t len);

#endif // BLUETOOTH_H
