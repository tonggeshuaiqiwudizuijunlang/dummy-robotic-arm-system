#ifndef __DM_IMU_H
#define __DM_IMU_H

#define DM_IUM_CNT 3
#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#include <stdint.h>
#include "imu_can.h"
#include "bsp_can.h"

enum Enum_DM_IMU_ControlModes
{
    DM_IMU_DataType_accel = 0x01,
    DM_IMU_DataType_omega = 0x02,
    DM_IMU_DataType_euler = 0x03,
    DM_IMU_DataType_quaternion = 0x04,
};

typedef struct
{
    float accel[3];
    float gyro[3];
    float qua[4];

    float roll;
    float pitch;
    float yaw;
    float yaw_zero;
    float pitch_zero;
    float roll_zero;
}Struct_DM_IMU_Data;

typedef struct
{    
    uint8_t lost_flag;
    DaemonInstance* daemon;// 守护线程实例
    CANInstance *can_ins; // CAN实例
    Struct_DM_IMU_Data IMU_Driver_Data;
    enum Enum_DM_IMU_ControlModes DM_IMU_ControlModes;
    
}DM_IMU_Instance_s;

uint8_t Get_IMU_lostflag(DM_IMU_Instance_s *DM_IMU);
void IMU_Mode_Set(DM_IMU_Instance_s *DM_IMU, enum Enum_DM_IMU_ControlModes DM_IMU_ControlModes);
DM_IMU_Instance_s *DM_IMU_Init(IMU_CAN_Init_Config_s *imu_config);
void IMU_Data_Request(DM_IMU_Instance_s *DM_IMU);
void IMU_Euler_Set_Zeros(DM_IMU_Instance_s *DM_IMU);
#endif
