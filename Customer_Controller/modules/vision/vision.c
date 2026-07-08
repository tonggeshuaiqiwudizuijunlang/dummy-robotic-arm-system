#include "vision.h"
#include "daemon.h"
#include <string.h>

/* 通信常量定义 */
#define PROTOCOL_HEADER  0xAA
#define PROTOCOL_TAILER  0xFFFB
#define PROTOCOL_LEN     sizeof(Received_Data_s)

/* 缓冲区大小 */
#define RX_RAW_FIFO_SIZE 256
static uint8_t host_raw_fifo[RX_RAW_FIFO_SIZE];
static uint16_t host_fifo_head = 0;
static uint16_t host_fifo_tail = 0;

static USARTInstance *host_usart_instance = NULL;
static DaemonInstance *host_daemon_instance = NULL;

/* 内部状态维护（仅维护接收数据） */
static Received_Data_s host_target_joints;
static uint8_t host_is_online = 0;

/**
 * @brief 接收中断回调函数
 */
static void HostRxCallback(void)
{
    if (!host_usart_instance) return;

    // 将 FSP 接收到的缓存块放入软 FIFO
    uint8_t *new_data = host_usart_instance->recv_buff; 
    
    for (int i = 0; i < host_usart_instance->recv_buff_size; i++)
    {
        host_raw_fifo[host_fifo_head] = new_data[i];
        host_fifo_head = (host_fifo_head + 1) % RX_RAW_FIFO_SIZE;
    }
    // 寻找完整的数据帧
    uint16_t valid_len = (host_fifo_head + RX_RAW_FIFO_SIZE - host_fifo_tail) % RX_RAW_FIFO_SIZE;
    while (valid_len >= PROTOCOL_LEN)
    {
        Received_Data_s temp_frame;
        uint8_t *p_temp = (uint8_t *)&temp_frame;
        
        // 由于有对齐约束，并且假定C与协议严格一致，此处将环形缓冲区内容映射出来核对
        for (uint16_t i = 0; i < PROTOCOL_LEN; i++)
        {
            p_temp[i] = host_raw_fifo[(host_fifo_tail + i) % RX_RAW_FIFO_SIZE];
        }

        // 仅匹配了头和尾的情况才将整帧认为有效并拷贝应用
        if (temp_frame.header == PROTOCOL_HEADER && temp_frame.tailer == PROTOCOL_TAILER)
        {
            // 校验通过，直接内存拷贝更新全局目标状态
            memcpy(&host_target_joints, &temp_frame, sizeof(Received_Data_s));
            host_fifo_tail = (host_fifo_tail + PROTOCOL_LEN) % RX_RAW_FIFO_SIZE;
            valid_len -= PROTOCOL_LEN;
            
            host_is_online = 1;
            if (host_daemon_instance)
            {
                DaemonReload(host_daemon_instance);
            }
        }
        else
        {
            // 头尾不匹配，滑动一个字节继续寻找
            host_fifo_tail = (host_fifo_tail + 1) % RX_RAW_FIFO_SIZE;
            valid_len--;
        }
    }
}

/**
 * @brief 离线回调
 */
static void HostLostCallback(void *id)
{
    (void)id;
    host_is_online = 0;
    // 强制重启串口接收状态机防死锁
    USART_Force_Restart(host_usart_instance);
}

Received_Data_s* Vision_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    // 配置使用那个串口, 改为你现有的 pc_uart (如终端报错提示，应当是 pc_uart_ctrl)
    USART_Init_Config_s usart_conf = {
        .recv_buff_size = sizeof(Received_Data_s), // 每次 DTC 收一块数据
        .p_uart_ctrl = p_ctrl,
        .p_uart_cfg = p_cfg,
        .module_callback = HostRxCallback
    };

    host_usart_instance = USARTRegister(&usart_conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 50,  // 50个心跳周期离线
        .callback = HostLostCallback,
        .owner_id = NULL,
    };
    host_daemon_instance = DaemonRegister(&daemon_conf);

    return &host_target_joints;
}

Received_Data_s* Vision_Get_TargetJoints(void)
{
    return &host_target_joints;
}

uint8_t Vision_Is_Online(void)
{
    return host_is_online && DaemonIsOnline(host_daemon_instance);
}

void Vision_Send_Data(uint8_t* tx_data, uint16_t len)
{
    if(host_usart_instance && !host_usart_instance->tx_busy && tx_data != NULL && len > 0U)
    {
        host_usart_instance->tx_busy = true;
        if (R_SCI_UART_Write(host_usart_instance->p_ctrl, tx_data, len) != FSP_SUCCESS)
            host_usart_instance->tx_busy = false;
    }
}
