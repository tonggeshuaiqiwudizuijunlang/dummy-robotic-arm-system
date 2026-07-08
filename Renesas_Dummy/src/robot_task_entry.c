#include "robot_task.h"
#include "dummy_cmd.h"
/* RobotTask entry function */
/* pvParameters contains TaskHandle_t */
void robot_task_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    DummyCmd_Init();
    /* TODO: add your own code here */
    while (1)
    {
        DummyCmd_Task();
        vTaskDelay(5);
    }
}
