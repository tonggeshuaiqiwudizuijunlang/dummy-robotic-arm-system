#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H

#include "stdint.h"
#include "string.h"
#include "bsp_dwt.h"
#include "hal_data.h"

#define ON 1
#define OFF 0


void LED_Set(uint8_t state);
void Buzzer_Set(uint8_t state);
void WIFI_Weakmode_Set(uint8_t state);
void WIFI_Reset_Set(void);

#endif
