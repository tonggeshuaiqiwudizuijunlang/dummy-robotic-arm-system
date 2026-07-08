#ifndef BSP_USB_H
#define BSP_USB_H

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

/** 
 * @brief USB 数据接收/发送回调函数指针类型定义
 * @param buffer: 数据指针
 * @param len: 数据长度
 */
typedef void (*USBCallback)(uint8_t *buffer, uint32_t len);

/**
 * @brief USB 初始化配置结构体
 */
typedef struct
{
    usb_ctrl_t *p_ctrl;        // FSP底层 USB 控制块句柄 (如 &g_basic0_ctrl)
    usb_cfg_t const *p_cfg;    // FSP底层 USB 配置块句柄 (如 &g_basic0_cfg)
    uint16_t recv_buff_size;   // 缓存区大小(建议2048等较大数值)
    USBCallback rx_cbk;        // 接收完成回调
    USBCallback tx_cbk;        // 发送完成回调
} USB_Init_Config_s;

/**
 * @brief USB 实例对象结构体
 */
typedef struct
{
    usb_ctrl_t *p_ctrl;
    usb_cfg_t const *p_cfg;
    uint8_t *rx_buffer;
    uint16_t rx_buff_size;
    
    USBCallback rx_cbk;
    USBCallback tx_cbk;

    volatile bool is_configured; // USB 是否已连接并枚举成功
    volatile bool tx_busy;       // 发送忙碌标志
} USBInstance;

/**
 * @brief 注册并初始化 USB 虚拟串口
 * @param init_config 配置信息
 * @return USBInstance* 返回申请的实例指针
 */
USBInstance *USBRegister(USB_Init_Config_s *init_config);

/**
 * @brief 通过 USB 发送数据
 * @param _instance USB实例句柄
 * @param buffer 待发送数据
 * @param len 数据长度
 * @return uint8_t 1:发送成功 0:发送失败/忙碌
 */
uint8_t USBTransmit(USBInstance *_instance, uint8_t *buffer, uint32_t len);

/**
 * @brief 获取 USB 连接态
 * @param _instance 实例句柄
 * @return true: 已连接; false: 未连接
 */
bool USBIsConfigured(USBInstance *_instance);

#endif // BSP_USB_H