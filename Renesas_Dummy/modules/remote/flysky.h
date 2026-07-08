/**
 * @file flysky.h
 * @brief FlySky SBUS mode: inverted, 100000 baud, 8 data bits, 2 stop bits, even parity.
 * @author Univs_he
 * @date 2025-12-15
 */

#ifndef FLYSKY_H
#define FLYSKY_H

#include <stdint.h>
#include "string.h"
#include "bsp_uart.h"
#include "stdlib.h"
#include "daemon.h"

#define FS_SBUS

#define FS_SBUS_USER_CHANNELS      10u
#define FS_FRAME_SIZE              25u

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

/* ----------------------- RC Switch Definition----------------------------- */
#define FS_DATA_UP   ((uint16_t)240)  // 开关向上时的值
#define FS_DATA_MID  ((uint16_t)1024) // 开关中间时的值
#define FS_DATA_DOWN ((uint16_t)1807) // 开关向下时的值

#define FS_DATA_MAX ((uint16_t)1807)
#define FS_DATA_MIN ((uint16_t)240)
#define FS_ROCKER_MAX ((int16_t)900)

#define knob_data_0 ((uint16_t)0)
#define knob_data_1 ((uint16_t)522)
#define knob_data_2 ((uint16_t)1048)
#define knob_data_3 ((uint16_t)1573)

/* ----------------------- RC Switch Definition----------------------------- */
#define FS_SW_UP   ((uint16_t)1) // 开关向上时的值
#define FS_SW_MID  ((uint16_t)3) // 开关中间时的值
#define FS_SW_DOWN ((uint16_t)2) // 开关向下时的值

#define fs_data_dead_limit(data, dealine)             \
    {                                                 \
        if ((data) > (dealine) || (data) < -(dealine))\
        {                                             \
            (data) = (data);                          \
        }                                             \
        else                                          \
        {                                             \
            (data) = 0;                               \
        }                                             \
    }

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
    int16_t rocker_r_;   // 右水平，SBUS[-784~783]
    int16_t rocker_r1;   // 右竖直，SBUS[-784~783]
    int16_t rocker_l1;   // 左水平，SBUS[-784~783]
    int16_t rocker_l_;   // 左竖直，SBUS[-784~783]

    uint16_t switch_l1;  // 最左侧拨杆
    uint16_t switch_l2;  // 左侧第2个拨杆
    uint16_t switch_r1;  // 右侧第1个拨杆
    uint16_t switch_r2;  // 最右侧拨杆

    int16_t knob_l;      // 左边旋钮，SBUS[0~1567]
    int16_t knob_r;      // 右边旋钮，SBUS[0~1567]

    uint16_t channel[FS_SBUS_USER_CHANNELS]; // CH1~CH10原始值, 用于调试通道映射
    uint8_t online_flag; // 遥控器在线标志位

} FS_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器
 * @param p_ctrl FSP UART 控制块指针 (如 &g_uart0_ctrl)
 * @param p_cfg  FSP UART 配置块指针 (如 &g_uart0_cfg)
 */
FS_ctrl_t *FSControlInit(uart_ctrl_t * p_ctrl, uart_cfg_t const * p_cfg);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t FSControlIsOnline(void);

#endif
