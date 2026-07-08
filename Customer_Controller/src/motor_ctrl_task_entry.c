#include "motor_ctrl_task.h"
#include "dji_motor.h"
#include "rs_motor.h"

/* Motor_Ctrl_Task entry function */
/* pvParameters contains TaskHandle_t */
void motor_ctrl_task_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    /* TODO: add your own code here */
    while (1)
    {
        RSMotorControl();
        DJIMotorControl();
        vTaskDelay(1);
    }
}
