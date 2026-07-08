#include "XLmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static XLMotorInstance *XL_motor_instance[XL_MOTOR_MX_CNT];
static osThreadId XL_TaskHandle[XL_MOTOR_MX_CNT];

static uint16_t Math_Float_to_Uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float Math_Uint_to_Float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}


void XL_Decode(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    XLMotorInstance *motor = (XLMotorInstance *)motor_can->id;
    XL_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = Math_Uint_to_Float(tmp, XL_P_MIN, XL_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->omega = Math_Uint_to_Float(tmp, XL_V_MIN, XL_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = Math_Uint_to_Float(tmp, XL_T_MIN, XL_T_MAX, 12);

    measure->temperature = (float)rxbuff[6];

}

void XLMotorGetError(XLMotorInstance *Motor)
{
    uint16_t ID_temp = Motor->motor_can_instance->txconf.StdId;

    Motor->motor_can_instance->txconf.StdId = (Motor->motor_can_instance->txconf.StdId << 5) + 0x003;

    for(int i=0; i<=7; i++)
    {
        Motor->motor_can_instance->tx_buff[i] = 0x00;
    }

    CANTransmit(Motor->motor_can_instance, 1);

    Motor->motor_can_instance->txconf.StdId = ID_temp;

}

void XLMotorClearError(XLMotorInstance *Motor)
{
    uint16_t ID_temp = Motor->motor_can_instance->txconf.StdId;

    Motor->motor_can_instance->txconf.StdId = (Motor->motor_can_instance->txconf.StdId << 5) + 0x018;

    for(int i=0; i<=7; i++)
    {
        Motor->motor_can_instance->tx_buff[i] = 0x00;
    }

    CANTransmit(Motor->motor_can_instance, 1);

    Motor->motor_can_instance->txconf.StdId = ID_temp;
}

void XL_Position_Set(XLMotorInstance *Motor, float angle_)
{
    Motor->XL_Motor_Send.position = angle_;
}

void XLMotorEnable(XLMotorInstance *Motor)
{
    uint16_t ID_temp = Motor->motor_can_instance->txconf.StdId;

    Motor->motor_can_instance->txconf.StdId = (Motor->motor_can_instance->txconf.StdId << 5) + 0x007;

    for(int i=0; i<=7; i++)
    {
        Motor->motor_can_instance->tx_buff[i] = 0x00;
    }
    Motor->motor_can_instance->tx_buff[0] = 0x08;
    Motor->motor_can_instance->tx_buff[4] = 0x00;

    CANTransmit(Motor->motor_can_instance, 1);

    Motor->motor_can_instance->txconf.StdId = ID_temp;
}

void XLMotor_Mode_Set(XLMotorInstance *Motor, enum Enum_XL_Motor_Status Motor_Driver_Mode)
{
    uint16_t ID_temp = Motor->motor_can_instance->txconf.StdId;

    Motor->motor_can_instance->txconf.StdId = (Motor->motor_can_instance->txconf.StdId << 5) + 0X00B;

    Motor->Motor_Driver_Mode = Motor_Driver_Mode;

    for(int i=0; i<=7; i++)
    {
        Motor->motor_can_instance->tx_buff[i] = 0x00;
    }

    switch(Motor->Motor_Driver_Mode)
    {
        case XL_Motor_ControlModes_Position:
        {
            Motor->motor_can_instance->tx_buff[0] = 0x03;
            Motor->motor_can_instance->tx_buff[4] = 0x01;
            break;
        }
        case XL_Motor_ControlModes_Speed:
        {
            Motor->motor_can_instance->tx_buff[0] = 0x02;
            Motor->motor_can_instance->tx_buff[4] = 0x02;
            break;
        }
        case XL_Motor_ControlModes_Position_xiebo:
        {
            Motor->motor_can_instance->tx_buff[0] = 0x03;
            Motor->motor_can_instance->tx_buff[4] = 0x03;
        }
    }
    CANTransmit(Motor->motor_can_instance, 1);

    Motor->motor_can_instance->txconf.StdId = ID_temp;

}

void XLMotorDisnable(XLMotorInstance *Motor)
{
    Motor->motor_status = MOTOR_STOP;
}

//这里面放什么东西我还没想好
static void XL_LostCallback(void *Motor_ptr)
{
    XLMotorInstance *Motor = (XLMotorInstance *)Motor_ptr;

    uint16_t can_bus = Motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;

    // LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
    // DM_Disnable(Motor);

}

XLMotorInstance *XL_Init(Motor_Init_Config_s *Config)
{
    XLMotorInstance *Motor = (XLMotorInstance *)malloc(sizeof(XLMotorInstance));
    memset(Motor, 0, sizeof(XLMotorInstance));

    Motor->motor_settings = Config->controller_setting_init_config;

    Config->can_init_config.can_module_callback = XL_Decode;
    Config->can_init_config.id = Motor;
    Motor->motor_can_instance = CANRegister(&Config->can_init_config);

    Daemon_Init_Config_s conf = 
    {
        .callback = XL_LostCallback,
        .owner_id = Motor,
        .reload_count = 10,
    };

    Motor->daemon = DaemonRegister(&conf);

    XLMotorGetError(Motor);
    XLMotorClearError(Motor);
    XLMotorEnable(Motor);
    
    // DWT_Delay(0.1);
    XL_motor_instance[idx++] = Motor;//储存电机实例指针

    return Motor;
}

void XL_Controll_Task(void const *argument)
{
    float Set;
    XLMotorInstance *Motor = (XLMotorInstance *)argument;

    Motor_Control_Setting_s *Setting = &Motor->motor_settings;
    
    Motor_Controller_s *motor_controller;   // 电机控制器
    XL_Motor_Measure_s *Measure;           // 电机测量值
    // XL_Motor_Send_s XL_Send;

    uint16_t ID_temp;

    while(1)
    {
        memset(Motor->motor_can_instance->tx_buff, 0, sizeof(Motor->motor_can_instance->tx_buff));

        ID_temp = Motor->motor_can_instance->txconf.StdId;

        Motor->motor_can_instance->txconf.StdId = (Motor->motor_can_instance->txconf.StdId << 5) + 0x0C;

        // if(Motor->motor_status == MOTOR_STOP)
        // {
        //     Motor->XL_Motor_Send.position = 0.0f;
        // }

        // LIMIT_MIN_MAX(Set, XL_P_MIN, XL_P_MAX);
        Set = Math_Float_to_Uint(Motor->XL_Motor_Send.position, XL_P_MIN, XL_P_MAX, 32);
        
        // Motor->motor_can_instance->tx_buff[0] = (uint8_t)(Set);           // 低位字节
        // Motor->motor_can_instance->tx_buff[1] = (uint8_t)(Set >> 8); // 第二低位字节
        // Motor->motor_can_instance->tx_buff[2] = (uint8_t)(Set >> 16); // 第三低位字节
        // Motor->motor_can_instance->tx_buff[3] = (uint8_t)(Set >> 24); // 高位字节

        Motor->motor_can_instance->tx_buff[0] = *(uint8_t *)&Motor->XL_Motor_Send.position;           // 低位字节
        Motor->motor_can_instance->tx_buff[1] = *(uint8_t *)((uint32_t)&Motor->XL_Motor_Send.position + 1); // 第二低位字节
        Motor->motor_can_instance->tx_buff[2] = *(uint8_t *)((uint32_t)&Motor->XL_Motor_Send.position + 2); // 第三低位字节
        Motor->motor_can_instance->tx_buff[3] = *(uint8_t *)((uint32_t)&Motor->XL_Motor_Send.position + 3); // 高位字节

        CANTransmit(Motor->motor_can_instance, 1);

        Motor->motor_can_instance->txconf.StdId = ID_temp;

        osDelay(4);

    }
}

void XL_ControInit()
{
    char dm_task_name[5] = "xl";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char DM_ID[1] = {0};
        __itoa(i, DM_ID, 10);
        strcat(dm_task_name, DM_ID);
        osThreadDef(dm_task_name, XL_Controll_Task, osPriorityAboveNormal, 0, 128);
        XL_TaskHandle[i] = osThreadCreate(osThread(dm_task_name), XL_motor_instance[i]);
    }
}