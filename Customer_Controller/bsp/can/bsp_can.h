#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include "hal_data.h"
#include "bsp_dwt.h"
#include "r_canfd.h" // 必须包含 CANFD 底层头文件以访问扩展配置


/* can instance typedef, every module registered to CAN should have this variable */
typedef struct CANInstance
{
    /* 硬件句柄 */
    can_ctrl_t * p_ctrl;
    can_cfg_t const * p_cfg;

    /* 接收缓冲区
     * 回调触发时，这里面存放了刚收到的数据
     */
    uint8_t rx_buff[8];
    uint8_t rx_len;         // 实际收到长度
    uint32_t current_rx_id; // 刚收到的 ID (含数据帧 ID + Command)
    uint32_t rx_id;         // 期望接收 ID，0 表示不过滤
    uint32_t tx_id;         // 默认发送 ID
    void *id;               // 上层实例指针

    /* 发送缓冲区 */
    uint8_t tx_buff[8];
    can_frame_t tx_frame;  // FSP 底层发送帧对象

    /* 用户回调函数 */
    void (*can_module_callback)(struct CANInstance *);

} CANInstance;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    can_ctrl_t * p_ctrl;
    can_cfg_t const * p_cfg;
    uint32_t tx_id;
    uint32_t rx_id;
    void *id;
    /* 接收到任何数据都会调用此回调 */
    void (*can_module_callback)(CANInstance *);
} CAN_Init_Config_s;

// CAN初始化
CANInstance *BSP_CAN_Init(CAN_Init_Config_s *config);
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief 发送 CAN 消息
 * @param _instance CAN 实例
 * @param std_id   标准 ID
 * @param data    数据指针
 * @param len     数据长度
 * @return 1:成功 0:失败
 */
uint8_t CANTransmit(CANInstance *_instance, uint32_t std_id, uint8_t *data, uint8_t len);

#endif /* __BSP_CAN_H */
