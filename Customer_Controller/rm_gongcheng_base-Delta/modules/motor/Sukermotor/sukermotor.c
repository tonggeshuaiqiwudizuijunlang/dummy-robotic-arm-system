#include "sukermotor.h"
#include <stdint.h>
#include "cmsis_os.h"

static uint8_t idx;
static Suker_Instance *suker_motor_instance[Suker_MOTOR_CNT];
static osThreadId Suker_TaskHandle[Suker_MOTOR_CNT];

static void DecodeSukerMotor(CANInstance *motor_can)
{
    uint8_t tmp, tmp_; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    Suker_Instance *motor = (Suker_Instance *)motor_can->id;
    Suker_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    tmp = rxbuff[0];
    tmp_ = rxbuff[1];
    

    if(tmp != 0x02 && tmp_ != 0x03)
    {
        motor->statues = MOTOR_DIABLE;
    }

}

static void Suker_LostCallback(void *Motor_ptr)
{
    
}

static void Suker_Callback()
{

}

void Suker_Enable(Suker_Instance *motor)
{
    motor->statues == MOTOR_ENALBE;
}

void Suker_Disable(Suker_Instance *motor)
{
    motor->statues == MOTOR_DIABLE;
}

Suker_Instance *Suker_Motor_Init(Motor_Init_Config_s *Config)
{
    Suker_Instance *Motor = (Suker_Instance *)malloc(sizeof(Suker_Instance));
    memset(Motor, 0, sizeof(Suker_Instance));

    Config->can_init_config.can_module_callback = Suker_Callback;
    Config->can_init_config.id = Motor;
    Motor->motor_can_instance = CANRegister(&Config->can_init_config);

    Daemon_Init_Config_s conf = 
    {
        .callback = Suker_Callback,
        .owner_id = Motor,
        .reload_count = 10,
    };

    Motor->motor_daemon = DaemonRegister(&conf);

    suker_motor_instance[idx++] = Motor;

    return Motor;
}

void Suker_Controll_Task()
{
    Suker_Instance *Motor;
    //     motor = suker_motor_instance[i];

    for (size_t i = 0; i < idx; ++i)
    { 
        // 减小访存开销,先保存指针引用
        Motor = suker_motor_instance[i];

        for(int i=0; i++; i<8)
            Motor->motor_can_instance->tx_buff[i] = 0x00;

        if(Motor->statues == MOTOR_ENALBE)
        {
            Motor->motor_can_instance->tx_buff[0] = 0x01;
            Motor->motor_can_instance->tx_buff[1] = 0x02;
            Motor->motor_can_instance->tx_buff[2] = 0x03;
            Motor->motor_can_instance->tx_buff[3] = 0x04;
        }
        if(Motor->statues == MOTOR_DIABLE)
        {
            Motor->motor_can_instance->tx_buff[0] = 0x03;
            Motor->motor_can_instance->tx_buff[1] = 0x04;
        }

        CANTransmit(Motor->motor_can_instance, 1);
    }
}

