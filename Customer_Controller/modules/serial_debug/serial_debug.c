#include "serial_debug.h"
#include <stdlib.h>
#include <string.h>

#define MAX_SERIAL_DEBUG_INSTANCES 2

static SerialDebug_Instance_s *serial_debug_instances[MAX_SERIAL_DEBUG_INSTANCES] = {NULL};
static uint8_t serial_debug_idx = 0;

static float SerialDebug_DecodeFloat(const uint8_t *buf)
{
    float value;
    memcpy(&value, buf, sizeof(float));
    return value;
}

/**
 * @brief 串口接收回调函数，解析固定长度 PID 调参帧
 */
static void SerialDebug_RxCallback(void)
{
    for (uint8_t i = 0; i < serial_debug_idx; i++)
    {
        SerialDebug_Instance_s *instance = serial_debug_instances[i];
        uint8_t *frame;

        if (instance == NULL || instance->usart == NULL || instance->pid_callback == NULL)
            continue;

        frame = instance->usart->recv_buff;
        if (frame[0] != SERIAL_DEBUG_PID_FRAME_HEAD_1 || frame[1] != SERIAL_DEBUG_PID_FRAME_HEAD_2 ||
            frame[15] != SERIAL_DEBUG_PID_FRAME_TAIL_1 || frame[16] != SERIAL_DEBUG_PID_FRAME_TAIL_2)
        {
            instance->pid_callback(0, 0.0f, 0.0f, 0.0f, SERIAL_DEBUG_PID_FRAME_BAD);
            continue;
        }

        instance->pid_callback(frame[2],
                               SerialDebug_DecodeFloat(&frame[3]),
                               SerialDebug_DecodeFloat(&frame[7]),
                               SerialDebug_DecodeFloat(&frame[11]),
                               SERIAL_DEBUG_PID_FRAME_OK);
    }
}

/**
 * @brief 初始化串口调试/波形观察模块
 * @param init_config 初始化配置结构体
 * @return 实例指针
 */
SerialDebug_Instance_s *SerialDebug_Init(SerialDebug_Init_Config_s *init_config)
{
    if (serial_debug_idx >= MAX_SERIAL_DEBUG_INSTANCES || init_config == NULL)
    {
        return NULL;
    }

    SerialDebug_Instance_s *instance = (SerialDebug_Instance_s *)malloc(sizeof(SerialDebug_Instance_s));
    if (instance == NULL)
    {
        return NULL;
    }
    memset(instance, 0, sizeof(SerialDebug_Instance_s));

    init_config->usart_config.module_callback = SerialDebug_RxCallback;
    init_config->usart_config.recv_buff_size = SERIAL_DEBUG_PID_FRAME_SIZE;

    instance->pid_callback = init_config->pid_callback;
    instance->usart = USARTRegister(&init_config->usart_config);

    serial_debug_instances[serial_debug_idx++] = instance;

    return instance;
}

/**
 * @brief 按 VOFA+ JustFloat 协议格式发送浮点数数组用于上位机波形显示
 * @param instance 串口调试模块实例
 * @param data 要发送的浮点数数组指针
 * @param num 浮点数数量
 */
void SerialDebug_Send_JustFloat(SerialDebug_Instance_s *instance, float *data, uint8_t num)
{
    if (instance == NULL || instance->usart == NULL || data == NULL)
    {
        return;
    }

    if (num > SERIAL_DEBUG_MAX_CHANNEL)
    {
        num = SERIAL_DEBUG_MAX_CHANNEL;
    }

    uint16_t total_size = num * sizeof(float) + 4; // float 数据 + 4字节帧尾
    if (total_size > SERIAL_DEBUG_TX_BUFF_SIZE)
    {
        return;
    }

    // 拷贝 float 数据
    memcpy(instance->tx_buff, data, num * sizeof(float));

    // 添加 JustFloat 帧尾：0x00 0x00 0x80 0x7F
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(instance->tx_buff + num * sizeof(float), tail, 4);

    if (instance->usart->tx_busy)
    {
        return;
    }

    instance->usart->tx_busy = true;
    fsp_err_t err = R_SCI_UART_Write(instance->usart->p_ctrl, instance->tx_buff, total_size);
    if (err != FSP_SUCCESS)
    {
        instance->usart->tx_busy = false;
    }
}
