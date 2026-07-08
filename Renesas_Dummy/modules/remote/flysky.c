#include "flysky.h"

#define RAW_FIFO_SIZE 128
// s为soft的意思，软件fifo，存储原始数据的环形缓冲区，大小128字节，足够存储5帧数据（25*5=125）
static uint8_t s_raw_fifo[RAW_FIFO_SIZE];
static uint16_t s_head = 0;
static uint16_t s_tail = 0;

// 遥控器数据
static FS_ctrl_t fs_ctrl[2];
static uint8_t fs_init_flag = 0;
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *fs_usart_instance;
static DaemonInstance *fs_daemon_instance;

static uint16_t FSDataChange(uint16_t data)
{
    if (data < 650)
        return FS_SW_UP;
    if (data > 1400)
        return FS_SW_DOWN;
    return FS_SW_MID;
}

/**
 * @brief 矫正遥控器摇杆的值,超过900或者小于-900的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick(void)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (abs(*(&fs_ctrl[TEMP].rocker_r_ + i)) > FS_ROCKER_MAX)
            *(&fs_ctrl[TEMP].rocker_r_ + i) = 0;
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void FSLostCallback(void *id)
{
    (void)id;
    memset(fs_ctrl, 0, sizeof(fs_ctrl)); // 遥控器数据清零
    USART_Force_Restart(fs_usart_instance);
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_fs(const uint8_t *sbus_buf)
{
    if ((sbus_buf[0] != 0x0F) || (sbus_buf[24] != 0x00))
        return;

    fs_ctrl[TEMP].channel[0] = ((sbus_buf[1] | ((uint16_t)sbus_buf[2] << 8)) & 0x07FF);
    fs_ctrl[TEMP].channel[1] = (((sbus_buf[2] >> 3) | ((uint16_t)sbus_buf[3] << 5)) & 0x07FF);
    fs_ctrl[TEMP].channel[2] = (((sbus_buf[3] >> 6) | ((uint16_t)sbus_buf[4] << 2) | ((uint16_t)sbus_buf[5] << 10)) & 0x07FF);
    fs_ctrl[TEMP].channel[3] = (((sbus_buf[5] >> 1) | ((uint16_t)sbus_buf[6] << 7)) & 0x07FF);
    fs_ctrl[TEMP].channel[4] = (((sbus_buf[6] >> 4) | ((uint16_t)sbus_buf[7] << 4)) & 0x07FF);
    fs_ctrl[TEMP].channel[5] = (((sbus_buf[7] >> 7) | ((uint16_t)sbus_buf[8] << 1) | ((uint16_t)sbus_buf[9] << 9)) & 0x07FF);
    fs_ctrl[TEMP].channel[6] = (((sbus_buf[9] >> 2) | ((uint16_t)sbus_buf[10] << 6)) & 0x07FF);
    fs_ctrl[TEMP].channel[7] = (((sbus_buf[10] >> 5) | ((uint16_t)sbus_buf[11] << 3)) & 0x07FF);
    fs_ctrl[TEMP].channel[8] = ((sbus_buf[12] | ((uint16_t)sbus_buf[13] << 8)) & 0x07FF);
    fs_ctrl[TEMP].channel[9] = (((sbus_buf[13] >> 3) | ((uint16_t)sbus_buf[14] << 5)) & 0x07FF);

    fs_ctrl[TEMP].rocker_r_ = (int16_t)fs_ctrl[TEMP].channel[0] - FS_DATA_MID;
    fs_ctrl[TEMP].rocker_r1 = (int16_t)fs_ctrl[TEMP].channel[1] - FS_DATA_MID;
    fs_ctrl[TEMP].rocker_l1 = (int16_t)fs_ctrl[TEMP].channel[2] - FS_DATA_MID;
    fs_ctrl[TEMP].rocker_l_ = (int16_t)fs_ctrl[TEMP].channel[3] - FS_DATA_MID;

    fs_ctrl[TEMP].switch_l1 = FSDataChange(fs_ctrl[TEMP].channel[4]);
    fs_ctrl[TEMP].switch_l2 = FSDataChange(fs_ctrl[TEMP].channel[5]);
    fs_ctrl[TEMP].switch_r1 = FSDataChange(fs_ctrl[TEMP].channel[6]);
    fs_ctrl[TEMP].switch_r2 = FSDataChange(fs_ctrl[TEMP].channel[7]);

    fs_ctrl[TEMP].knob_l = (int16_t)fs_ctrl[TEMP].channel[8] - FS_DATA_MIN;
    fs_ctrl[TEMP].knob_r = (int16_t)fs_ctrl[TEMP].channel[9] - FS_DATA_MIN;

    fs_data_dead_limit(fs_ctrl[TEMP].rocker_l_, 5);
    fs_data_dead_limit(fs_ctrl[TEMP].rocker_l1, 5);
    fs_data_dead_limit(fs_ctrl[TEMP].rocker_r_, 5);
    fs_data_dead_limit(fs_ctrl[TEMP].rocker_r1, 5);

    RectifyRCjoystick();
    fs_ctrl[TEMP].online_flag = 1;
    memcpy(&fs_ctrl[LAST], &fs_ctrl[TEMP], sizeof(FS_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
    if (fs_daemon_instance)
        DaemonReload(fs_daemon_instance);
}

/* 这是 BSP 调用的回调函数 */
static void FSRxCallback(void)
{
    // 1. 从 BSP 拿到这一帧长度的数据
    uint8_t *new_data = fs_usart_instance->recv_buff;
    uint16_t recv_len = fs_usart_instance->recv_len;

    if (recv_len == 0U)
        recv_len = fs_usart_instance->recv_buff_size;

    // 2. 全部塞入软 FIFO
    for (uint16_t i = 0; i < recv_len; i++)
    {
        s_raw_fifo[s_head] = new_data[i];
        s_head = (s_head + 1) % RAW_FIFO_SIZE;
    }

    // 3. 在 FIFO 里滑窗搜索合法的 SBUS 帧 (0x0F 开头, 0x00 结尾)
    // 只有当数据量 >= FS_FRAME_SIZE 时才找
    while ((s_head + RAW_FIFO_SIZE - s_tail) % RAW_FIFO_SIZE >= FS_FRAME_SIZE)
    {
        // A. 检查帧头 0x0F
        if (s_raw_fifo[s_tail] != 0x0F)
        {
            // 不是帧头，抛弃这 1 字节，继续滑
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
            continue;
        }

        // B. 检查帧尾 0x00 (位置是 tail + 24)
        uint16_t end_idx = (s_tail + FS_FRAME_SIZE - 1) % RAW_FIFO_SIZE;
        if (s_raw_fifo[end_idx] != 0x00)
        {
            // 校验失败（可能是数据里的假 0x0F），抛弃头部，继续滑
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
            continue;
        }

        // C. 找到啦！提取这一帧
        uint8_t valid_frame[FS_FRAME_SIZE];
        for (uint16_t j = 0; j < FS_FRAME_SIZE; j++)
        {
            valid_frame[j] = s_raw_fifo[s_tail];
            s_tail = (s_tail + 1) % RAW_FIFO_SIZE;
        }

        // D. 去解析
        sbus_to_fs(valid_frame);
    }
}

/* Online保留 */
uint8_t FSControlIsOnline(void)
{
    if (fs_init_flag)
        return DaemonIsOnline(fs_daemon_instance);
    return 0;
}

FS_ctrl_t *FSControlInit(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
{
    USART_Init_Config_s conf;
    conf.module_callback = FSRxCallback;

    /* 直接填充 FSP 指针 */
    conf.p_uart_ctrl = p_ctrl;
    conf.p_uart_cfg = p_cfg;
    conf.recv_buff_size = FS_FRAME_SIZE;

    fs_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 200,
        .callback = FSLostCallback,
        .owner_id = NULL,
    };
    fs_daemon_instance = DaemonRegister(&daemon_conf);

    fs_init_flag = 1;
    return fs_ctrl;
}
