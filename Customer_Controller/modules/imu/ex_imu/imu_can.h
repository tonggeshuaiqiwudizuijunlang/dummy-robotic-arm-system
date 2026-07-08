#ifndef IMU_CAN_H
#define IMU_CAN_H

#include "bsp_can.h"
#include "daemon.h"

#pragma pack(1)
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} IMU_CAN_Msg_s;
#pragma pack()

typedef struct
{
    CANInstance *can_ins;
    IMU_CAN_Msg_s imu_msg;
    DaemonInstance *daemon;
    uint8_t *lost_flag;
} IMUCANInstance;

typedef struct
{
    CAN_Init_Config_s can_config;
} IMU_CAN_Init_Config_s;

#endif // IMU_CAN_H
