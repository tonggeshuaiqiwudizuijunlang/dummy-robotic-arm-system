#ifndef BSP_USART_H
#define BSP_USART_H

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define DEVICE_USART_CNT 4
#define USART_RXBUFF_LIMIT 256
#define UART_RING_BUFFER_SIZE 512
typedef void (*usart_module_callback)(void);

/* 
 * 移除了 UART_Handle_t 
 * 直接将 FSP 原生指针放入实例中
 */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT];
    uint8_t recv_buff_size;
    /* FSP 原生句柄 */
    uart_ctrl_t * p_ctrl;
    uart_cfg_t const * p_cfg;

    usart_module_callback module_callback;
    
    volatile bool tx_busy; // 发送忙标志
    /* 环形缓冲区管理 */
    uint8_t rx_buffer[UART_RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} USARTInstance;

/* 初始化配置结构体也同步修改 */
typedef struct
{
    /* 直接传入 FSP 生成的全局变量指针 */
    uint8_t recv_buff_size;
    uart_ctrl_t * p_uart_ctrl;     
    uart_cfg_t const * p_uart_cfg; 
    usart_module_callback module_callback;
} USART_Init_Config_s;

/* API */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);
void USART_Force_Restart(USARTInstance *_instance);

#endif
