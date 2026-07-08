#include "vofa.h"
#include <stdlib.h>
#include <string.h>

// Global pointer to instances for callback mapping since HAL doesn't pass user context
#define MAX_VOFA_INSTANCES 3
static VofaInstance_s* vofa_instances[MAX_VOFA_INSTANCES] = {NULL};
static uint8_t vofa_idx = 0;

static void VofaRxCallback()
{
    for(uint8_t i = 0; i < vofa_idx; i++) {
        VofaInstance_s *vofa = vofa_instances[i];
        if(vofa && vofa->usart && vofa->usart->recv_buff_size > 0) {
            // Check if we received the JustFloat tail 0x00 0x00 0x80 0x7f at the end of the packet
            // Usually we might just receive string commands if we want to change PID,
            // or we might receive JustFloat format. Let's support a simple string format first.

            // For strings like "kp=1.5\n", you could parse it here, but to be compatible
            // with VOFA+ JustFloat, we usually receive float arrays exactly like we send.

            // Look for the tail signature
            int tail_found_idx = -1;
            for (int j = 0; j < VOFA_RX_BUFF_SIZE - 3; j++) {
                if (vofa->usart->recv_buff[j] == 0x00 &&
                    vofa->usart->recv_buff[j+1] == 0x00 &&
                    vofa->usart->recv_buff[j+2] == 0x80 &&
                    vofa->usart->recv_buff[j+3] == 0x7f) {
                    tail_found_idx = j;
                    break;
                }
            }

            if (tail_found_idx > 0 && vofa->cmd_callback) {
                // The floats are before the tail
                uint8_t float_count = tail_found_idx / 4;
                if (float_count > 0 && float_count <= 10) {
                    float received_floats[10] = {0};
                    memcpy(received_floats, vofa->usart->recv_buff, tail_found_idx);
                    vofa->cmd_callback(received_floats, float_count);
                }
            }
        }
    }
}

VofaInstance_s* VofaInit(Vofa_Init_Config_s *init_config)
{
    if (vofa_idx >= MAX_VOFA_INSTANCES) return NULL;

    VofaInstance_s *vofa = (VofaInstance_s *)malloc(sizeof(VofaInstance_s));
    memset(vofa, 0, sizeof(VofaInstance_s));

    vofa->cmd_callback = init_config->cmd_callback;

    init_config->usart_config.module_callback = VofaRxCallback;
    vofa->usart = USARTRegister(&init_config->usart_config);

    vofa_instances[vofa_idx++] = vofa;

    return vofa;
}

// Send data in JustFloat protocol format used by VOFA+
// Format: float array + tail(0x00 0x00 0x80 0x7f)
void VofaJustFloatSend(VofaInstance_s *vofa, float* data, uint8_t num)
{
    if(num > 10) num = 10;

    uint16_t total_size = num * sizeof(float) + 4;

    if(total_size > VOFA_TX_BUFF_SIZE) return;

    // Copy float data
    memcpy(vofa->tx_buff, data, num * sizeof(float));

    // Add tail
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    memcpy(vofa->tx_buff + num * sizeof(float), tail, 4);

    USARTSend(vofa->usart, vofa->tx_buff, total_size, USART_TRANSFER_DMA);
}

void VofaSend(VofaInstance_s *vofa, float* data, uint8_t num)
{
    VofaJustFloatSend(vofa, data, num);
}
