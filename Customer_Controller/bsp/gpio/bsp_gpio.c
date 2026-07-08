#include "bsp_gpio.h"

void LED_Set(uint8_t state)
{
    if (state == ON)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, User_LED, BSP_IO_LEVEL_LOW);
    }
    else if (state == OFF)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, User_LED, BSP_IO_LEVEL_HIGH);
    }
}

void Buzzer_Set(uint8_t state)
{
    if (state == ON)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, buzzer, BSP_IO_LEVEL_LOW);
    }
    else if (state == OFF)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, buzzer, BSP_IO_LEVEL_HIGH);
    }
}
void WIFI_Weakmode_Set(uint8_t state)
{
    if (state == ON)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, WIFI_WAKE, BSP_IO_LEVEL_LOW);
        R_BSP_PinWrite(WIFI_RESET, BSP_IO_LEVEL_HIGH);
    }
    else if (state == OFF)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, WIFI_WAKE, BSP_IO_LEVEL_HIGH);
        R_BSP_PinWrite(WIFI_RESET, BSP_IO_LEVEL_LOW);
    }
}

void WIFI_Reset_Set(void)
{
    /* 拉低复位 */
    R_BSP_PinWrite(WIFI_RESET, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS); // 保持低电平一段时间
    
    /* 拉高释放 */
    R_BSP_PinWrite(WIFI_RESET, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS); // 等待模块启动稳定
}
