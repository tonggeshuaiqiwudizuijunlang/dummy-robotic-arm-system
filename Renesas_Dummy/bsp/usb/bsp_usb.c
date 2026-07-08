#include "bsp_usb.h"
#include <string.h>

/* USB 是单端口设备, 此处使用单例句柄以便 FSP 底层回调函数能找到实例对象 */
static USBInstance usb_instance_obj;
static USBInstance *usb_instance = NULL;

/* 静态分配一块较大的内存用于USB接收，避免使用标准 malloc 而导致的堆内存不足问题 */
#define USB_RX_MAX_SIZE 2048
static uint8_t usb_static_rx_buf[USB_RX_MAX_SIZE];

USBInstance *USBRegister(USB_Init_Config_s *init_config)
{
    if (usb_instance != NULL) 
        return usb_instance; // 防止重复注册

    // 检查 FSP 句柄合法性
    if (!init_config || !init_config->p_ctrl || !init_config->p_cfg)
        return NULL;

    // 弃用包含隐患的 malloc，改为纯静态分配
    USBInstance *instance = &usb_instance_obj;
    memset(instance, 0, sizeof(USBInstance));

    instance->p_ctrl = init_config->p_ctrl;
    instance->p_cfg  = init_config->p_cfg;
    instance->rx_cbk = init_config->rx_cbk;
    instance->tx_cbk = init_config->tx_cbk;
    
    // 限制最大缓存不要越界
    instance->rx_buff_size = (init_config->recv_buff_size <= USB_RX_MAX_SIZE) ? init_config->recv_buff_size : USB_RX_MAX_SIZE;
    instance->rx_buffer = usb_static_rx_buf;

    instance->is_configured = false;
    instance->tx_busy = false;

    // 初始化 FSP 硬件底层
    fsp_err_t err = R_USB_Open(instance->p_ctrl, instance->p_cfg);
    if (FSP_SUCCESS != err)
    {
        return NULL;
    }

    usb_instance = instance;
    return instance;
}

uint8_t USBTransmit(USBInstance *_instance, uint8_t *buffer, uint32_t len)
{
    if (!_instance || !_instance->is_configured || _instance->tx_busy)
        return 0; // 返回 0 表示发送失败或硬件正忙碌
    _instance->tx_busy = true;
    // FSP 发送 API 需附带 USB_CLASS_PCDC 标识
    fsp_err_t err = R_USB_Write(_instance->p_ctrl, buffer, len, USB_CLASS_PCDC);
    if (err != FSP_SUCCESS)
    {
        _instance->tx_busy = false; // 出错立刻解除忙碌标志
        return 0;
    }
    return 1; // 成功投递
}

bool USBIsConfigured(USBInstance *_instance)
{
    return _instance ? _instance->is_configured : false;
}

/**
 * @brief FSP 底层界面中填写的 USB CallBack 函数
 *        由系统中断自动触发处理 USB 生命周期与数据收发
 */
void usb_cdc_callback(usb_event_info_t *p_api_event, usb_hdl_t cur_task, usb_onoff_t usb_state)
{
    (void)cur_task;
    (void)usb_state;
    if (!usb_instance) return;

    switch (p_api_event->event)
    {
        case USB_STATUS_CONFIGURED: // 1. 主机(如电脑)接上，成功枚举完成了
            usb_instance->is_configured = true;
            // 不同于 UART, USB 必须要在枚举成功后才能开启读取。
            // 提交读取请求：硬件会在此阻塞等待电脑发来不定长数据。
            R_USB_Read(usb_instance->p_ctrl, usb_instance->rx_buffer, usb_instance->rx_buff_size, USB_CLASS_PCDC);
            break;
        case USB_STATUS_DETACH:     // USB 物理拔出
        case USB_STATUS_SUSPEND:    // 或者电脑进入休眠挂起
            usb_instance->is_configured = false;
            break;
        case USB_STATUS_READ_COMPLETE: // 2. 接收到了不定长的网络数据包！就像 STM32 的 ReceiveToIdle 
            // 将数据传达给用户的回掉函数
            if (usb_instance->rx_cbk) {
                // p_api_event->data_size 自动帮我们统计了硬件实际上收到了多少个字节（自动实现不定长判断!）
                usb_instance->rx_cbk(usb_instance->rx_buffer, p_api_event->data_size);
            }
            // 每次接收完不要忘了重新续订监听，防止死锁
            if (usb_instance->is_configured) {
                R_USB_Read(usb_instance->p_ctrl, usb_instance->rx_buffer, usb_instance->rx_buff_size, USB_CLASS_PCDC);
            }
            break;
        case USB_STATUS_WRITE_COMPLETE: // 3. 发送完成回调
            usb_instance->tx_busy = false;
            if (usb_instance->tx_cbk) {
                usb_instance->tx_cbk(NULL, 0); // 通知上层数据发送完成可用
            }
            break;
        default:
            break;
    }
}
