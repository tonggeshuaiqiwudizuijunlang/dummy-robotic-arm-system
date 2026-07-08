#include "bsp_can.h"

#define CAN_MX_REGISTER_CNT 16
static CANInstance *can0_instance = NULL;
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t can_idx = 0;

/* 1. 定义过滤器规则 (全接收 Accept All) */
const canfd_afl_entry_t can_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    {
        .id = {
            .id = 0,
            .frame_type = CAN_FRAME_TYPE_DATA,
            .id_mode = CAN_ID_MODE_STANDARD
        },
        .mask = {
            .mask_id = 0x00000000, // 29位全 1：表示 ID 的每一位都不关心 -> 接收所有 ID
            .mask_frame_type = 0,  // 0：必须匹配 .id 中的 frame_type (只收数据帧)
            .mask_id_mode = 0      // 0：必须匹配 .id 中的 id_mode (只收标准帧)
        },
        .destination = {
            .minimum_dlc = CANFD_MINIMUM_DLC_0, .rx_buffer = CANFD_RX_MB_NONE,
            .fifo_select_flags = CANFD_RX_FIFO_0 // 接收到 RX FIFO 0
        }
    }
};
/**
 * @brief CAN 初始化函数 (包含 AFL 全接收配置)
 * @return fsp_err_t 初始化结果
 */
CANInstance *BSP_CAN_Init(CAN_Init_Config_s * config)
{
    fsp_err_t err;

    if (config == NULL || can_idx >= CAN_MX_REGISTER_CNT)
        return NULL;

    if (can0_instance == NULL)
    {
        err = R_CANFD_Open(config->p_ctrl, config->p_cfg);
        if (FSP_SUCCESS != err)
        {
            return NULL;
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    if (instance == NULL) return NULL;
    memset(instance, 0, sizeof(CANInstance));

    instance->p_ctrl = config->p_ctrl;
    instance->p_cfg  = config->p_cfg;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->id = config->id;
    instance->can_module_callback = config->can_module_callback;

    can_instance[can_idx++] = instance;
    if (can0_instance == NULL)
    {
        can0_instance = instance;
    }

    return instance;
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    return BSP_CAN_Init(config);
}

/**
 * @brief 设置 DLC
 */
void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (length > 8)
        length = 8;
    if (length == 0)
        length = 1;
    _instance->tx_frame.data_length_code = length;
}

/**
 * @brief RASC 配置的 Callback (需要在配置里填 can_callback)
 *        对应 STM32 的 HAL_CAN_RxFifo0MsgPendingCallback
 */
void can_callback(can_callback_args_t *p_args)
{
    if (CAN_EVENT_RX_COMPLETE == p_args->event)
    {
        for (uint8_t i = 0; i < can_idx; i++)
        {
            CANInstance *instance = can_instance[i];
            if (instance == NULL)
                continue;
            if (instance->rx_id != 0 && instance->rx_id != p_args->frame.id)
                continue;

            instance->current_rx_id = p_args->frame.id;
            instance->rx_len = p_args->frame.data_length_code;

            uint8_t copy_len = instance->rx_len > 8 ? 8 : instance->rx_len;
            memcpy(instance->rx_buff, p_args->frame.data, copy_len);

            if (instance->can_module_callback)
            {
                instance->can_module_callback(instance);
            }
        }

    }
}

/**
 * @brief 发送 CAN 消息 (原子操作)
 * @param std_id 目标电机 ID (包含命令码)
 * @param data   要发送的数据指针
 * @param len    数据长度 (0-8)
 */
uint8_t CANTransmit(CANInstance *_instance, uint32_t std_id, uint8_t *data, uint8_t len)
{
    fsp_err_t err;

    /* 1. 填充 ID */
    _instance->tx_frame.id = std_id;
    _instance->tx_frame.id_mode = CAN_ID_MODE_STANDARD;
    _instance->tx_frame.type = CAN_FRAME_TYPE_DATA;

    _instance->tx_frame.options = 0;
    /* 2. 填充长度 */
    _instance->tx_frame.data_length_code = (len > 8) ? 8 : len;

    /* 3. 填充数据 (直接在这里拷贝，不需要再依赖 tx_buff) */
    if (len > 0 && data != NULL)
    {
        memcpy(_instance->tx_frame.data, data, _instance->tx_frame.data_length_code);
    }

    /* 4. 发送 (非阻塞尝试) */
    /* 注意：R_CAN_Write 是标准CAN, R_CANFD_Write 是FD。根据你的RASC选择 */
    err = R_CANFD_Write(_instance->p_ctrl, CANFD_TX_BUFFER_FIFO_COMMON_1, &_instance->tx_frame);

    return (err == FSP_SUCCESS) ? 1 : 0;
}
