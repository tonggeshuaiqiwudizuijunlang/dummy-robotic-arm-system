/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "imu_can.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
static IMUCANInstance *imu_can_instance = NULL; // 可以由app保存此指针
static uint8_t imulostflag;                     // IMU丢失标志
static void IMUCANRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    IMU_CAN_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &imu_can_instance->imu_msg;
    // roll ：X = ((RollH << 8) | RollL) / 32768 * 180
    // pitch ：Y = ((PitchH << 8) | PitchL) / 32768 * 180
    // yaw ：Z = ((YawH << 8) | YawL) / 32768 * 180
    Msg->roll = (float)(rxbuff[1] << 8 | rxbuff[0]) / 32768.0 * 180;
    Msg->pitch = (float)(rxbuff[3] << 8 | rxbuff[2]) / 32768.0 * 180;
    Msg->yaw = (float)(rxbuff[5] << 8 | rxbuff[4]) / 32768.0 * 180;
    if (Msg->roll > 180)
    {
        Msg->roll -= 360;
    }
    if (Msg->pitch > 180)
    {
        Msg->pitch -= 360;
    }
    imulostflag = 0; // 收到数据清除丢失标志
}

// IMU丢失回调函数
static void IMUCANLostCallback(void *imu_ptr)
{
    imulostflag=1;                                  // 设置丢失标志
    IMUCANInstance *imu = (IMUCANInstance *)imu_ptr;
    uint16_t can_bus = imu->can_ins->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[IMUCANInstance] Motor lost, can bus [%d] , id [%d]", can_bus, imu->can_ins->tx_id);
}
IMUCANInstance *IMUCANInit(IMU_CAN_Init_Config_s *imu_config)
{
    imu_can_instance = (IMUCANInstance *)malloc(sizeof(IMUCANInstance));
    memset(imu_can_instance, 0, sizeof(IMUCANInstance));
    
    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = IMUCANLostCallback,
        .owner_id = imu_can_instance,
        .reload_count = 25, // 25ms未收到数据则丢失
    };
    imu_can_instance->daemon = DaemonRegister(&daemon_config);
    imu_can_instance->lost_flag = &imulostflag;                 // 丢失标志

    imu_config->can_config.can_module_callback = IMUCANRxCallback;
    imu_can_instance->can_ins = CANRegister(&imu_config->can_config);
    return imu_can_instance;
}

void IMUCANSend(IMUCANInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

IMU_CAN_Msg_s IMUCANGet(IMUCANInstance *instance)
{
    return instance->imu_msg;
}

