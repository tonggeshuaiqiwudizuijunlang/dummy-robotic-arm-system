#include "bluetooth.h"
#include <string.h>

#define BT_FRAME_SIZE sizeof(RX_BT_Pose_Data_s)
#define BT_RAW_FIFO_SIZE    256
#define BT_UART_RECV_SIZE   1U
#define BT_RX_HEADER        0x5A
#define BT_RX_TAILER        0xFFFB

// 软FIFO缓存
static uint8_t s_bt_raw_fifo[BT_RAW_FIFO_SIZE];
static uint16_t s_bt_head = 0;
static uint16_t s_bt_tail = 0;

static USARTInstance *bt_uart_instance = NULL;
static RX_BT_Pose_Data_s bt_received_data;

volatile BT_Debug_s bt_debug = {0};

static void BT_RawFifo_Push(uint8_t data)
{
    uint16_t next_head = (s_bt_head + 1U) % BT_RAW_FIFO_SIZE;

    if (next_head == s_bt_tail)
    {
        bt_debug.rx_fifo_overflow_count++;
        s_bt_tail = (s_bt_tail + 1U) % BT_RAW_FIFO_SIZE;
    }

    s_bt_raw_fifo[s_bt_head] = data;
    s_bt_head = next_head;
}

/**
 * @brief 提取解析蓝牙环形缓冲中的结构体数据 (软FIFO机制)
 */
#if 0
static void BT_RxCallback(void)
{
    bt_debug.callback_count++;

    if (!bt_uart_instance) return;

    // 将 FSP 接收到的缓存块放入软 FIFO
    uint8_t *new_data = bt_uart_instance->recv_buff;
    uint16_t recv_len = bt_uart_instance->recv_len;

    if (recv_len == 0U)
        recv_len = bt_uart_instance->recv_buff_size;

    bt_debug.last_recv_len = recv_len;
    bt_debug.rx_byte_count += recv_len;
    
    for (uint16_t i = 0; i < recv_len; i++)
    {
        bt_debug.last_byte = new_data[i];
        bt_debug.raw_tail[bt_debug.raw_index] = new_data[i];
        bt_debug.raw_index = (uint8_t)((bt_debug.raw_index + 1U) % BT_DEBUG_RAW_WINDOW);
        BT_RawFifo_Push(new_data[i]);
    }
    // 寻找完整的数据帧
    uint16_t valid_len = (s_bt_head + BT_RAW_FIFO_SIZE - s_bt_tail) % BT_RAW_FIFO_SIZE;
    bt_debug.fifo_valid_len = valid_len;
    while (valid_len >= BT_FRAME_SIZE)
    {
        RX_BT_Pose_Data_s temp_frame;
        uint8_t *p_temp = (uint8_t *)&temp_frame;
        
        // 由于有对齐约束，并且假定C与协议严格一致，此处将环形缓冲区内容映射出来核对
        for (uint16_t i = 0; i < BT_FRAME_SIZE; i++)
        {
            p_temp[i] = s_bt_raw_fifo[(s_bt_tail + i) % BT_RAW_FIFO_SIZE];
        }

        // 仅匹配了头和尾的情况才将整帧认为有效并拷贝应用
        bt_debug.last_candidate_header = temp_frame.header;
        bt_debug.last_candidate_tailer = temp_frame.tailer;
        if (temp_frame.header == BT_RX_HEADER && temp_frame.tailer == BT_RX_TAILER)
        {
            // 校验通过，直接内存拷贝更新全局目标状态
            memcpy(&bt_received_data, &temp_frame, sizeof(RX_BT_Pose_Data_s));
            bt_debug.rx_frame_count++;
            s_bt_tail = (s_bt_tail + BT_FRAME_SIZE) % BT_RAW_FIFO_SIZE;
            valid_len -= BT_FRAME_SIZE;
        }
        else
        {
            // 头尾不匹配，滑动一个字节继续寻找
            bt_debug.rx_resync_count++;
            s_bt_tail = (s_bt_tail + 1) % BT_RAW_FIFO_SIZE;
            valid_len--;
            bt_debug.fifo_valid_len = valid_len;
        }
    }
}

/**
 * @brief HC05 蓝牙模块初始化
 * @param p_ctrl UART控制块指针
 * @param p_cfg  UART配置结构体指针
 * @return 接收数据结构体的全局地址指针
 */
#endif

static void BT_RxCallback(void)
{
    bt_debug.callback_count++;

    if (!bt_uart_instance)
        return;

    uint8_t *new_data = bt_uart_instance->recv_buff;
    uint16_t recv_len = bt_uart_instance->recv_len;

    if (recv_len == 0U)
        recv_len = bt_uart_instance->recv_buff_size;

    bt_debug.last_recv_len = recv_len;
    bt_debug.rx_byte_count += recv_len;

    for (uint16_t i = 0; i < recv_len; i++)
    {
        bt_debug.last_byte = new_data[i];
        bt_debug.raw_tail[bt_debug.raw_index] = new_data[i];
        bt_debug.raw_index = (uint8_t)((bt_debug.raw_index + 1U) % BT_DEBUG_RAW_WINDOW);
        BT_RawFifo_Push(new_data[i]);
    }

    uint16_t valid_len = (s_bt_head + BT_RAW_FIFO_SIZE - s_bt_tail) % BT_RAW_FIFO_SIZE;
    bt_debug.fifo_valid_len = valid_len;
    while (valid_len >= BT_FRAME_SIZE)
    {
        RX_BT_Pose_Data_s temp_frame;
        uint8_t *p_temp = (uint8_t *)&temp_frame;

        for (uint16_t i = 0; i < BT_FRAME_SIZE; i++)
        {
            p_temp[i] = s_bt_raw_fifo[(s_bt_tail + i) % BT_RAW_FIFO_SIZE];
        }

        bt_debug.last_candidate_header = temp_frame.header;
        bt_debug.last_candidate_tailer = temp_frame.tailer;
        if (temp_frame.header == BT_RX_HEADER && temp_frame.tailer == BT_RX_TAILER)
        {
            memcpy(&bt_received_data, &temp_frame, sizeof(RX_BT_Pose_Data_s));
            bt_debug.rx_frame_count++;
            s_bt_tail = (s_bt_tail + BT_FRAME_SIZE) % BT_RAW_FIFO_SIZE;
            valid_len -= BT_FRAME_SIZE;
            bt_debug.fifo_valid_len = valid_len;
        }
        else
        {
            bt_debug.rx_resync_count++;
            s_bt_tail = (s_bt_tail + 1U) % BT_RAW_FIFO_SIZE;
            valid_len--;
            bt_debug.fifo_valid_len = valid_len;
        }
    }
}

RX_BT_Pose_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    bt_debug.init_count++;
    bt_debug.frame_size = BT_FRAME_SIZE;
    bt_debug.uart_channel = (uint8_t)p_cfg->channel;

    USART_Init_Config_s config;
    config.recv_buff_size = BT_UART_RECV_SIZE;
    config.use_rx_char = 1U;
    config.p_uart_ctrl = p_ctrl;
    config.p_uart_cfg  = p_cfg;
    config.module_callback = BT_RxCallback;

    bt_uart_instance = USARTRegister(&config);
    bt_debug.register_ok = (bt_uart_instance != NULL) ? 1U : 0U;

    // 为防出现垃圾数据
    memset(&bt_received_data, 0, sizeof(RX_BT_Pose_Data_s));
    bt_debug.frame_size = BT_FRAME_SIZE;

    return &bt_received_data;
}

/**
 * @brief HC05 串口发送数据 (直接推流)
 */
#if 0
bool BT_SendData(uint8_t* tx_data, uint16_t len)
{
    if (bt_uart_instance == NULL || tx_data == NULL || len == 0) return false;

    // 将结构体作为二进制字节流推入 BSP 层发送
    fsp_err_t err = R_SCI_UART_Write(bt_uart_instance->p_ctrl, tx_data, len);
    
    return (err == FSP_SUCCESS);
}
#endif

bool BT_SendData(uint8_t* tx_data, uint16_t len)
{
    if ((bt_uart_instance == NULL) || (tx_data == NULL) || (len == 0U))
        return false;

    fsp_err_t err = R_SCI_UART_Write(bt_uart_instance->p_ctrl, tx_data, len);
    return (err == FSP_SUCCESS);
}
