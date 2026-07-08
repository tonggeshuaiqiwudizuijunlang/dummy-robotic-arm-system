#include "DM_IMU.h"
#include "daemon.h"
#include <stdlib.h>
#include <string.h>

static uint8_t idx;
static DM_IMU_Instance_s *IMU_Instance_buf[DM_IUM_CNT] = {NULL};
static uint8_t imulostflag;

static void IMUCANLostCallback(void *imu_ptr)
{
    imulostflag = 1;
    DM_IMU_Instance_s *imu = (DM_IMU_Instance_s *)imu_ptr;
    if (imu != NULL)
        imu->lost_flag = 1;
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void IMU_UpdateAccel(DM_IMU_Instance_s *DM_IMU, uint8_t *pData)
{
    uint16_t accel[3];

    accel[0] = pData[3] << 8 | pData[2];
    accel[1] = pData[5] << 8 | pData[4];
    accel[2] = pData[7] << 8 | pData[6];
    DM_IMU->IMU_Driver_Data.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    DM_IMU->IMU_Driver_Data.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    DM_IMU->IMU_Driver_Data.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

static void IMU_UpdateGyro(DM_IMU_Instance_s *DM_IMU, uint8_t *pData)
{
    uint16_t gyro[3];

    gyro[0] = pData[3] << 8 | pData[2];
    gyro[1] = pData[5] << 8 | pData[4];
    gyro[2] = pData[7] << 8 | pData[6];

    DM_IMU->IMU_Driver_Data.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    DM_IMU->IMU_Driver_Data.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    DM_IMU->IMU_Driver_Data.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

static void IMU_UpdateEuler(DM_IMU_Instance_s *DM_IMU, uint8_t *pData)
{
    int euler[3];

    euler[0] = pData[3] << 8 | pData[2];
    euler[1] = pData[5] << 8 | pData[4];
    euler[2] = pData[7] << 8 | pData[6];

    DM_IMU->IMU_Driver_Data.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16) - DM_IMU->IMU_Driver_Data.pitch_zero;
    DM_IMU->IMU_Driver_Data.yaw = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16) - DM_IMU->IMU_Driver_Data.yaw_zero;
    DM_IMU->IMU_Driver_Data.roll = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16) - DM_IMU->IMU_Driver_Data.roll_zero;
}

static void IMU_UpdateQuaternion(DM_IMU_Instance_s *DM_IMU, uint8_t *pData)
{
    int w = pData[1] << 6 | ((pData[2] & 0xF8) >> 2);
    int x = (pData[2] & 0x03) << 12 | (pData[3] << 4) | ((pData[4] & 0xF0) >> 4);
    int y = (pData[4] & 0x0F) << 10 | (pData[5] << 2) | (pData[6] & 0xC0) >> 6;
    int z = (pData[6] & 0x3F) << 8 | pData[7];

    DM_IMU->IMU_Driver_Data.qua[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->IMU_Driver_Data.qua[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->IMU_Driver_Data.qua[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    DM_IMU->IMU_Driver_Data.qua[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

static void DM_IMU_Decode(CANInstance *imu_can)
{
    uint8_t *rxbuff = imu_can->rx_buff;
    DM_IMU_Instance_s *imu = (DM_IMU_Instance_s *)imu_can->id;
    if (imu == NULL)
        return;

    if (imu->daemon != NULL)
        DaemonReload(imu->daemon);

    switch (rxbuff[0])
    {
    case DM_IMU_DataType_accel:
        IMU_UpdateAccel(imu, rxbuff);
        break;
    case DM_IMU_DataType_omega:
        IMU_UpdateGyro(imu, rxbuff);
        break;
    case DM_IMU_DataType_euler:
        IMU_UpdateEuler(imu, rxbuff);
        break;
    case DM_IMU_DataType_quaternion:
        IMU_UpdateQuaternion(imu, rxbuff);
        break;
    default:
        break;
    }

    imulostflag = 0;
    imu->lost_flag = 0;
}

void IMU_Euler_Set_Zeros(DM_IMU_Instance_s *DM_IMU)
{
    if (DM_IMU == NULL)
        return;
    DM_IMU->IMU_Driver_Data.yaw_zero = DM_IMU->IMU_Driver_Data.yaw;
    DM_IMU->IMU_Driver_Data.pitch_zero = DM_IMU->IMU_Driver_Data.pitch;
    DM_IMU->IMU_Driver_Data.roll_zero = DM_IMU->IMU_Driver_Data.roll;
}

void IMU_Mode_Set(DM_IMU_Instance_s *DM_IMU, enum Enum_DM_IMU_ControlModes DM_IMU_ControlModes)
{
    if (DM_IMU != NULL)
        DM_IMU->DM_IMU_ControlModes = DM_IMU_ControlModes;
}

uint8_t Get_IMU_lostflag(DM_IMU_Instance_s *DM_IMU)
{
    if (DM_IMU == NULL)
        return 1;
    return DM_IMU->lost_flag;
}

DM_IMU_Instance_s *DM_IMU_Init(IMU_CAN_Init_Config_s *imu_config)
{
    if (imu_config == NULL || idx >= DM_IUM_CNT)
        return NULL;

    DM_IMU_Instance_s *IMU_Instance = (DM_IMU_Instance_s *)malloc(sizeof(DM_IMU_Instance_s));
    if (IMU_Instance == NULL)
        return NULL;
    memset(IMU_Instance, 0, sizeof(DM_IMU_Instance_s));

    Daemon_Init_Config_s daemon_config = {
        .callback = IMUCANLostCallback,
        .owner_id = IMU_Instance,
        .reload_count = 200,
    };
    IMU_Instance->daemon = DaemonRegister(&daemon_config);

    imu_config->can_config.can_module_callback = DM_IMU_Decode;
    imu_config->can_config.id = IMU_Instance;
    IMU_Instance->can_ins = CANRegister(&imu_config->can_config);
    if (IMU_Instance->can_ins == NULL)
    {
        free(IMU_Instance);
        return NULL;
    }

    IMU_Instance->lost_flag = 1;
    IMU_Instance->IMU_Driver_Data.yaw_zero = 0.0f;
    IMU_Instance_buf[idx++] = IMU_Instance;

    return IMU_Instance;
}

void IMU_Data_Request(DM_IMU_Instance_s *DM_IMU)
{
    if (DM_IMU == NULL || DM_IMU->can_ins == NULL)
        return;

    uint8_t data[4] = {0};
    data[0] = (uint8_t)DM_IMU->can_ins->tx_id;
    data[1] = (uint8_t)(DM_IMU->can_ins->tx_id >> 8);
    data[3] = 0xCC;

    switch (DM_IMU->DM_IMU_ControlModes)
    {
    case DM_IMU_DataType_accel:
        data[2] = DM_IMU_DataType_accel;
        break;
    case DM_IMU_DataType_omega:
        data[2] = DM_IMU_DataType_omega;
        break;
    case DM_IMU_DataType_euler:
        data[2] = DM_IMU_DataType_euler;
        break;
    case DM_IMU_DataType_quaternion:
        data[2] = DM_IMU_DataType_quaternion;
        break;
    default:
        data[2] = DM_IMU_DataType_euler;
        break;
    }

    memcpy(DM_IMU->can_ins->tx_buff, data, 4);
    CANTransmit(DM_IMU->can_ins, 0x6FF, DM_IMU->can_ins->tx_buff, 4);
}
