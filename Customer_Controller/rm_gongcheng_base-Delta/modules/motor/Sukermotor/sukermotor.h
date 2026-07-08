#ifndef __SUKERMOTOR_H
#define __SUKERMOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define Suker_MOTOR_CNT 3

typedef enum
{
    MOTOR_ENALBE = 0,
    MOTOR_DIABLE = 1,
}Suker_Motor_Statue_s; 

typedef struct 
{
    uint8_t id;
    uint8_t state;

}Suker_Motor_Measure_s;

typedef struct 
{
    Suker_Motor_Measure_s measure;
    Suker_Motor_Statue_s statues;
    CANInstance *motor_can_instance;
    DaemonInstance* motor_daemon;
}Suker_Instance;

void Suker_Enable(Suker_Instance *motor);
void Suker_Disable(Suker_Instance *motor);
Suker_Instance *Suker_Motor_Init(Motor_Init_Config_s *Config);
void Suker_Controll_Task();


// void DMMotorSetRef(DM_Instance *motor, float ref);

#endif