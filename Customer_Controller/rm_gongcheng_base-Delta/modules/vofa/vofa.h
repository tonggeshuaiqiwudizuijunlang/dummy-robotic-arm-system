#ifndef VOFA_H
#define VOFA_H
#include "usart.h"
#include "bsp_usart.h"
#include <stdint.h>

#define VOFA_TX_BUFF_SIZE 128
#define VOFA_RX_BUFF_SIZE 128

// Define a callback type for handling parsed float commands from VOFA
// Returns 1 if handled, 0 if not
typedef void (*Vofa_Cmd_Callback)(float *cmd_data, uint8_t cmd_num);

typedef struct
{
    float ch[10];
} VofaData_s;

typedef struct
{
    USARTInstance *usart;
    VofaData_s data;
    uint8_t tx_buff[VOFA_TX_BUFF_SIZE];
    Vofa_Cmd_Callback cmd_callback;
} VofaInstance_s;

typedef struct
{
    USART_Init_Config_s usart_config;
    Vofa_Cmd_Callback cmd_callback;
} Vofa_Init_Config_s;

VofaInstance_s* VofaInit(Vofa_Init_Config_s *init_config);
void VofaSend(VofaInstance_s *vofa, float* data, uint8_t num);
void VofaJustFloatSend(VofaInstance_s *vofa, float* data, uint8_t num);

#endif
