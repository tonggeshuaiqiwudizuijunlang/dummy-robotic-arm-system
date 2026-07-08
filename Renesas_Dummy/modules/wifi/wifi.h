#ifndef BLUETOOTH_H
#define BLUETOOTH_H


#include "bsp_uart.h"
#include "bsp_gpio.h"




/* --- 蓝牙状态机枚举 --- */
typedef enum {
    BT_STEP_IDLE,           // 空闲状态
    BT_STEP_HARD_RESET,     // 硬件复位
    BT_STEP_WAIT_READY,     // 等待启动 Ready
    BT_STEP_SEND_AT,        // 发送 AT 测试
    BT_STEP_SEND_ECHO,      // 关闭回显 ATE0
    BT_STEP_BLE_INIT,       // 蓝牙初始化 (配置为从机 Server)
    BT_STEP_SET_NAME,       // 设置蓝牙名称
    BT_STEP_ADV_START,      // 开启蓝牙广播
    BT_STEP_WAIT_CONN,      // 等待手机或其他主机连接 (+BLECONN)
    BT_STEP_ENTER_SPP,      // 开启透传模式
    BT_STEP_DATA_TRANS,     // 透传数据收发阶段
    BT_STEP_ERROR           // 出错状态
} bt_step_t;




/* --- API 接口 --- */
RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
void BT_Task_Entry(void);                           // 放在操作系统的任务 while(1) 中轮询
bool BT_SendData(uint8_t* tx_data, uint16_t len);   // 发送透传数据
bool BT_IsConnected(void);                          // 检查是否在透传状态
void test(void);
#endif /* BLUETOOTH_H */