#include "bluetooth.h"
#include <string.h>

#define BT_FRAME_SIZE sizeof(RX_BT_Data_s)
#define BT_RAW_FIFO_SIZE    256

// 软FIFO缓存
static uint8_t s_bt_raw_fifo[BT_RAW_FIFO_SIZE];
static uint16_t s_bt_head = 0;
static uint16_t s_bt_tail = 0;

static USARTInstance *bt_uart_instance = NULL;
static RX_BT_Data_s bt_received_data;

/**
 * @brief 提取解析蓝牙环形缓冲中的结构体数据 (软FIFO机制)
 */
static void BT_RxCallback(void)
{
    if (!bt_uart_instance) return;

    // 将 FSP 接收到的缓存块放入软 FIFO
    uint8_t *new_data = bt_uart_instance->recv_buff; 
    
    for (int i = 0; i < bt_uart_instance->recv_buff_size; i++)
    {
        s_bt_raw_fifo[s_bt_head] = new_data[i];
        s_bt_head = (s_bt_head + 1) % BT_RAW_FIFO_SIZE;
    }
    // 寻找完整的数据帧
    uint16_t valid_len = (s_bt_head + BT_RAW_FIFO_SIZE - s_bt_tail) % BT_RAW_FIFO_SIZE;
    while (valid_len >= BT_FRAME_SIZE)
    {
        RX_BT_Data_s temp_frame;
        uint8_t *p_temp = (uint8_t *)&temp_frame;
        
        // 由于有对齐约束，并且假定C与协议严格一致，此处将环形缓冲区内容映射出来核对
        for (uint16_t i = 0; i < BT_FRAME_SIZE; i++)
        {
            p_temp[i] = s_bt_raw_fifo[(s_bt_tail + i) % BT_RAW_FIFO_SIZE];
        }

        // 仅匹配了头和尾的情况才将整帧认为有效并拷贝应用
        if (temp_frame.header == 0xAA && temp_frame.tailer == 0xFFFB)
        {
            // 校验通过，直接内存拷贝更新全局目标状态
            memcpy(&bt_received_data, &temp_frame, sizeof(RX_BT_Data_s));
            s_bt_tail = (s_bt_tail + BT_FRAME_SIZE) % BT_RAW_FIFO_SIZE;
            valid_len -= BT_FRAME_SIZE;
        }
        else
        {
            // 头尾不匹配，滑动一个字节继续寻找
            s_bt_tail = (s_bt_tail + 1) % BT_RAW_FIFO_SIZE;
            valid_len--;
        }
    }
}

/**
 * @brief HC05 蓝牙模块初始化
 * @param p_ctrl UART控制块指针
 * @param p_cfg  UART配置结构体指针
 * @return 接收数据结构体的全局地址指针
 */
RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    USART_Init_Config_s config;
    config.recv_buff_size = sizeof(RX_BT_Data_s);
    config.p_uart_ctrl = p_ctrl;
    config.p_uart_cfg  = p_cfg;
    config.module_callback = BT_RxCallback;

    bt_uart_instance = USARTRegister(&config);

    // 为防出现垃圾数据
    memset(&bt_received_data, 0, sizeof(RX_BT_Data_s));

    return &bt_received_data;
}

/**
 * @brief HC05 串口发送数据 (直接推流)
 */
bool BT_SendData(uint8_t* tx_data, uint16_t len)
{
    if (bt_uart_instance == NULL || tx_data == NULL || len == 0 || bt_uart_instance->tx_busy) return false;

    // 将结构体作为二进制字节流推入 BSP 层发送
    bt_uart_instance->tx_busy = true;
    fsp_err_t err = R_SCI_UART_Write(bt_uart_instance->p_ctrl, tx_data, len);
    if (err != FSP_SUCCESS)
    {
        bt_uart_instance->tx_busy = false;
    }

    return (err == FSP_SUCCESS);
}