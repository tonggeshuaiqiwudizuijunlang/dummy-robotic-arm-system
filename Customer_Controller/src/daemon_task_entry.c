#include "daemon_task.h"
#include "daemon.h"
#include "vision.h"

Transmit_Data_s temp_data = {0};

/* DaemonTask entry function */
/* pvParameters contains TaskHandle_t */
void daemon_task_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    
    /* TODO: add your own code here */
    while (1)
    {
        DaemonTask();
        vTaskDelay(10);
    }
}
