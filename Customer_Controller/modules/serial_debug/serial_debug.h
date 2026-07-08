#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "bsp_uart.h"
#include <stdint.h>

#define SERIAL_DEBUG_TX_BUFF_SIZE      256
#define SERIAL_DEBUG_PID_FRAME_SIZE    17
#define SERIAL_DEBUG_MAX_CHANNEL       18
#define SERIAL_DEBUG_JUSTFLOAT_CHANNELS SERIAL_DEBUG_MAX_CHANNEL

#define SERIAL_DEBUG_PID_FRAME_HEAD_1  0xAAU
#define SERIAL_DEBUG_PID_FRAME_HEAD_2  0x55U
#define SERIAL_DEBUG_PID_FRAME_TAIL_1  0x0DU
#define SERIAL_DEBUG_PID_FRAME_TAIL_2  0x0AU

/* PID 调参帧解析状�?*/
typedef enum
{
    SERIAL_DEBUG_PID_FRAME_OK = 0,
    SERIAL_DEBUG_PID_FRAME_BAD = 1,
} SerialDebug_PID_Frame_Status_e;

typedef void (*SerialDebug_PID_Callback)(uint8_t loop_id, float kp, float ki, float kd, uint8_t status);

typedef struct
{
    USARTInstance *usart;
    uint8_t tx_buff[SERIAL_DEBUG_TX_BUFF_SIZE];
    SerialDebug_PID_Callback pid_callback;
} SerialDebug_Instance_s;

typedef struct
{
    USART_Init_Config_s usart_config;
    SerialDebug_PID_Callback pid_callback;
} SerialDebug_Init_Config_s;

/**
 * @brief 初始化串口调�?波形观察模块
 */
SerialDebug_Instance_s *SerialDebug_Init(SerialDebug_Init_Config_s *init_config);

/**
 * @brief 发送多通道浮点数据到上位机（匹�?VOFA+ JustFloat 协议�? */
void SerialDebug_Send_JustFloat(SerialDebug_Instance_s *instance, float *data, uint8_t num);

#endif
