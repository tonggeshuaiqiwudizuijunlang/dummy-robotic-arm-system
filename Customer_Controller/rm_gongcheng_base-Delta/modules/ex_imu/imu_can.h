/*
 * @Descripttion:
 * @version:
 * @Author: liu
 * @Date: 2025-3-22 21:47
 * @LastEditTime: 2025-3-22 21:47
 */

#ifndef IMU_CAN_H
#define IMU_CAN_H

#include "bsp_can.h"
#include "daemon.h"
#pragma pack(1)
typedef struct
{
    float roll; // X轴角度
    float pitch; // Y轴角度
    float yaw; // Z轴角度
} IMU_CAN_Msg_s;
#pragma pack()

/* IMU实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    IMU_CAN_Msg_s imu_msg; // IMU信息
    DaemonInstance* daemon;// 守护线程实例
    uint8_t* lost_flag; // 丢失标志
} IMUCANInstance;

/* IMU初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} IMU_CAN_Init_Config_s;

/**
 * @brief 初始化IMU
 * 
 * @param imu_config IMU初始化配置
 * @return IMUCANInstance* IMU实例指针
 */
IMUCANInstance *IMUCANInit(IMU_CAN_Init_Config_s *imu_config);

/**
 * @brief 发送IMU控制信息
 * 
 * @param instance IMU实例
 * @param data IMU控制信息
 */
void IMUCANSend(IMUCANInstance *instance, uint8_t *data);

#endif // !IMU_CAN_H
