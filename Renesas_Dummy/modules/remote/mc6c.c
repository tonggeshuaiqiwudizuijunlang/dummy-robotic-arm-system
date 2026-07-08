#include "mc6c.h"

#define MC_FRAME_SIZE 25u // 遥控器接收的buffer大小
#define RAW_FIFO_SIZE 128
// s为soft的意思，软件fifo，存储原始数据的环形缓冲区，大小128字节，足够存储5帧数据（25*5=125）
static uint8_t s_raw_fifo[RAW_FIFO_SIZE];
static uint16_t s_head = 0;
static uint16_t s_tail = 0;

// 遥控器数据
static MC_ctrl_t mc_ctrl[2];
static uint8_t mc_init_flag = 0;
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *mc_usart_instance;
static DaemonInstance *mc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过800或者小于-800的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (abs(*(&mc_ctrl[TEMP].rocker_r_ + i)) > MC_DATA_MAX)
            *(&mc_ctrl[TEMP].rocker_r_ + i) = 0;
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void MCLostCallback(void *id)
{
    (void)id;
    memset(&mc_ctrl[TEMP], 0, sizeof(MC_ctrl_t)); // 遥控器数据清零
    USART_Force_Restart(mc_usart_instance);
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_mc(const uint8_t *sbus_buf)
{
    if ((sbus_buf[0] != 0x0F) || (sbus_buf[24] != 0x00))
        return;

    mc_ctrl[TEMP].rocker_r_ = ((sbus_buf[1] | ((uint16_t)sbus_buf[2] << 8)) & 0x07FF) - MC_DATA_MID; 
    mc_ctrl[TEMP].rocker_r1 = (((sbus_buf[2] >> 3) | ((uint16_t)sbus_buf[3] << 5)) & 0x07FF) - MC_DATA_MID; 
    mc_ctrl[TEMP].rocker_l1 = (((sbus_buf[3] >> 6) | ((uint16_t)sbus_buf[4] << 2) | ((uint16_t)sbus_buf[5] << 10)) & 0x07FF) - MC_DATA_MID; 
    mc_ctrl[TEMP].rocker_l_ = (((sbus_buf[5] >> 1) | ((uint16_t)sbus_buf[6] << 7)) & 0x07FF) - MC_DATA_MID; 
    mc_ctrl[TEMP].switch_l = (((sbus_buf[6] >> 4) | ((uint16_t)sbus_buf[7] << 4)) & 0x07FF);
    mc_ctrl[TEMP].switch_r = (((sbus_buf[7] >> 7) | ((uint16_t)sbus_buf[8] << 1) | ((uint16_t)sbus_buf[9] << 9)) & 0x07FF);
    mc_ctrl[TEMP].none[0] = (((sbus_buf[9] >> 2) | ((uint16_t)sbus_buf[10] << 6)) & 0x07FF);
    mc_ctrl[TEMP].none[1] = (((sbus_buf[10] >> 5) | ((uint16_t)sbus_buf[11] << 3)) & 0x07FF);

    mc_data_dead_limit(mc_ctrl[TEMP].rocker_l_, 5);
    
    // 把原来简单的5死区，换成与上一次的值做比较，变化量小于20则认为是0（即保持原值，忽略抖动）
    // 因为rocker_l1是不回中式的，所以用绝对控制好一些
    if (abs(mc_ctrl[TEMP].rocker_l1 - mc_ctrl[LAST].rocker_l1) < 30) {
        mc_ctrl[TEMP].rocker_l1 = mc_ctrl[LAST].rocker_l1;
    }

    mc_data_dead_limit(mc_ctrl[TEMP].rocker_r_, 5);
    mc_data_dead_limit(mc_ctrl[TEMP].rocker_r1, 5);

    mc_data_change(mc_ctrl[TEMP].switch_l);
    mc_data_change(mc_ctrl[TEMP].switch_r);

    RectifyRCjoystick();
    memcpy(&mc_ctrl[LAST], &mc_ctrl[TEMP], sizeof(MC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
    if (mc_daemon_instance)
        DaemonReload(mc_daemon_instance);
}

/* 这是 BSP 调用的回调函数 */
static void MCRxCallback(void)
{
    // 1. 从 BSP 拿到这 25 字节乱序/正序未知的数据
    uint8_t *new_data = mc_usart_instance->recv_buff; // 假设能访问到

    // 2. 全部塞入软 FIFO
    for (int i = 0; i < 25; i++)
    {
        s_raw_fifo[s_head] = new_data[i];
        s_head = (s_head + 1) % RAW_FIFO_SIZE;
    }

    // 3. 在 FIFO 里滑窗搜索合法的 SBUS 帧 (0x0F 开头, 0x00 结尾)
    // 只有当数据量 >= 25 时才找
    while ((s_head + RAW_FIFO_SIZE - s_tail) % RAW_FIFO_SIZE >= 25)
    {
        // A. 检查帧头 0x0F
        if (s_raw_fifo[s_tail] != 0x0F)
        {
            // 不是帧头，抛弃这 1 字节，继续滑
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
            continue;
        }

        // B. 检查帧尾 0x00 (位置是 tail + 24)
        uint16_t end_idx = (s_tail + 24) % RAW_FIFO_SIZE;
        if (s_raw_fifo[end_idx] != 0x00)
        {
            // 校验失败（可能是数据里的假 0x0F），抛弃头部，继续滑
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
            continue;
        }

        // C. 找到啦！提取这一帧
        uint8_t valid_frame[25];
        for (int j = 0; j < 25; j++)
        {
            valid_frame[j] = s_raw_fifo[s_tail];
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
        }

        // D. 去解析
        sbus_to_mc(valid_frame);
    }
}



/* Online保留 */
uint8_t MCControlIsOnline()
{
    if (mc_init_flag)
        return DaemonIsOnline(mc_daemon_instance);
    return 0;
}

/*
 * 修改：参数改为直接传入 ctrl 和 cfg 指针
 * 这样调用时直接传 &g_uart0_ctrl, &g_uart0_cfg 即可
 */
MC_ctrl_t *MCControlInit(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    USART_Init_Config_s conf;
    conf.module_callback = MCRxCallback;

    /* 直接填充 FSP 指针 */
    conf.p_uart_ctrl = p_ctrl;
    conf.p_uart_cfg = p_cfg;
    conf.recv_buff_size = MC_FRAME_SIZE;

    mc_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200,
        .callback = MCLostCallback,
        .owner_id = NULL,
    };
    mc_daemon_instance = DaemonRegister(&daemon_conf);

    mc_init_flag = 1;
    return mc_ctrl;
}
