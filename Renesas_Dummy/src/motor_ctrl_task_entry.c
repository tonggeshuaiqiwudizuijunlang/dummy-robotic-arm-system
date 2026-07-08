#include "motor_ctrl_task.h"
#include "dummy_motormatic.h"

/* Motor_Ctrl_Task entry function */
/* pvParameters contains TaskHandle_t */
void motor_ctrl_task_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    Dummy_Motormatic_Init();
    /* TODO: add your own code here */
    while (1)
    {
        Dummy_Motormatic_Task();
        vTaskDelay(5);
    }
}
