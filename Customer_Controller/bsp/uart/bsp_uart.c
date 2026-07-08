#include "bsp_uart.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx = 0;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* 修改点1：不再需要 temp_rx_byte，而是需要一个足够大的 buffer */
/* 在 bsp_uart.h 里把 buffer 大小改为 25 或更大 */

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (init_config == NULL || idx >= DEVICE_USART_CNT) return NULL;

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    if (instance == NULL) return NULL;
    memset(instance, 0, sizeof(USARTInstance));

    instance->p_ctrl = init_config->p_uart_ctrl;
    instance->p_cfg = init_config->p_uart_cfg;
    instance->module_callback = init_config->module_callback;
    instance->recv_buff_size = init_config->recv_buff_size; // 传入数据包长度
    usart_instance[idx++] = instance;

    /* Open */
    fsp_err_t err = R_SCI_UART_Open(instance->p_ctrl, instance->p_cfg);
    if (FSP_SUCCESS != err)
    {
        free(instance);
        idx--; 
        return NULL;
    }

    /* 启动 DTC 接收 */
    err = R_SCI_UART_Read(instance->p_ctrl, instance->recv_buff, instance->recv_buff_size);
    if (FSP_SUCCESS != err)
    {
        R_SCI_UART_Close(instance->p_ctrl);
        free(instance);
        idx--;
        return NULL;
    }

    return instance;
}

void UART_Global_Callback(uart_callback_args_t *p_args)
{
    USARTInstance *target = NULL;
    /* 查找实例 */
    for (int i = 0; i < idx; i++)
    {
        if (usart_instance[i]->p_cfg->channel == p_args->channel)
        {
            target = usart_instance[i];
            break;
        }
    }
    if (!target) return;
    /* 处理错误 (DTC模式下偶尔也会有错误) */
    if (p_args->event == UART_EVENT_ERR_OVERFLOW ||
        p_args->event == UART_EVENT_ERR_FRAMING || 
        p_args->event == UART_EVENT_ERR_PARITY || 
        p_args->event == UART_EVENT_BREAK_DETECT)
    {
        /* 任何错误都重启接收，防止死锁 */
        USART_Force_Restart(target);
        return;
    }
    /* 本次接收完成 */
    if (p_args->event == UART_EVENT_RX_COMPLETE)
    {
        if (target->module_callback) target->module_callback();
        R_SCI_UART_Read(target->p_ctrl, target->recv_buff, target->recv_buff_size);
    }
    /* 处理单字节接收事件 (不使用 DTC 时触发此事件) */
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        /* 获取数据 */
        uint8_t data = (uint8_t)p_args->data;
        
        /* 存入环形缓冲区 */
        uint16_t next_head = (target->head + 1) % UART_RING_BUFFER_SIZE;
        if (next_head != target->tail) 
        {
            target->rx_buffer[target->head] = data;
        }
        target->head = next_head;        
        /* 可选：每收到一个字节都回调，或者在上层定时轮询 */
        if (target->module_callback) target->module_callback(); 
    }
    else if (p_args->event == UART_EVENT_TX_COMPLETE)
    {
        target->tx_busy = false;
    }

}

/* 新增 API：强制重启接收 */
void USART_Force_Restart(USARTInstance *_instance)
{
    if (!_instance) return;

    _instance->head = 0U;
    _instance->tail = 0U;
    R_SCI_UART_Abort(_instance->p_ctrl, UART_DIR_RX);
    (void)R_SCI_UART_Read(_instance->p_ctrl, _instance->recv_buff, _instance->recv_buff_size);
}

