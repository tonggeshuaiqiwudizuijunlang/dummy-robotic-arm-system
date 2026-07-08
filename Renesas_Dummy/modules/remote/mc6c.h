/**
 * @file flysky.h
 * @brief [SBUS模式(反相)，波特率100_000，数据位8位，停止位2位，偶检验]
 * @author Univs_he
 * @date 2025-12-15
 */

#ifndef MC6C_H
#define MC6C_H

#include <stdint.h>
#include "string.h"
#include "bsp_uart.h"
#include "stdlib.h"
#include "daemon.h"

#define MC_SBUS

#define MC_SBUS_USER_CHANNELS		6
/* User configuration */

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

/* ----------------------- RC Switch Definition----------------------------- */
#define MC_DATA_UP ((uint16_t)200)   // 开关向上时的值
#define MC_DATA_MID ((uint16_t)1000)  // 开关中间时的值
#define MC_DATA_DOWN ((uint16_t)1800) // 开关向下时的值

#define MC_DATA_MAX ((uint16_t)1800) // 最小值
#define MC_DATA_MIN ((uint16_t)200) // 最大值

/* ----------------------- RC Switch Definition----------------------------- */
#define MC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define MC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define MC_SW_DOWN ((uint16_t)2) // 开关向下时的值


#define mc_data_dead_limit(data, dealine)        			\
    {                                                    	\
        if((data) > (dealine) || (data) < -(dealine)) 		\
        {                                                	\
            (data) = (data);                          		\
        }                                                	\
        else                                             	\
        {                                                	\
            (data) = 0;                                		\
        }                                                	\
    }
	

#define mc_data_change(data)                                \
	{                                                    	\
		switch(data)										\
		{													\
			case MC_DATA_DOWN:							    \
				data = MC_SW_DOWN;							\
				break;										\
			case MC_DATA_MID:								\
				data = MC_SW_MID;							\
				break;										\
			case MC_DATA_UP:								\
				data = MC_SW_UP;							\
				break;										\
			default:										\
				break;										\
		}													\
	}

/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
    int16_t rocker_r_;      // 右水平，左前侧拨杆保持上拨，[-400~400]
    int16_t rocker_r1;      // 右竖直，左前侧拨杆保持上拨，[-400~400]
    int16_t rocker_l1;      // 左水平，左前侧拨杆保持上拨，[-800~800]
    int16_t rocker_l_;      // 左竖直，左前侧拨杆保持上拨，[-400~400]

    uint16_t switch_l;  // 最左侧拨杆
    uint16_t switch_r;  // 最右侧开关

    uint16_t none[2];  // 保留

} MC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器
 * @param p_ctrl FSP UART 控制块指针 (如 &g_uart0_ctrl)
 * @param p_cfg  FSP UART 配置块指针 (如 &g_uart0_cfg)
 */
MC_ctrl_t *MCControlInit(uart_ctrl_t * p_ctrl, uart_cfg_t const * p_cfg);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t MCControlIsOnline();

#endif
