/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

#include "bsp_dwt.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz = 0;
static uint32_t CPU_FREQ_Hz_ms = 0;
static uint32_t CPU_FREQ_Hz_us = 0;
static uint32_t CYCCNT_RountCount = 0;
static uint32_t CYCCNT_LAST = 0;
static uint64_t CYCCNT64 = 0;

/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
        {
            CYCCNT_RountCount++;
        }

        CYCCNT_LAST = cnt_now;
        bit_locker = 0;
    }
}

/**
 * @brief 初始化 DWT 计数器
 * @param CPU_Freq_MHz : CPU 频率，单位 MHz（例如 200 表示 200 MHz）
 * @note 也可以调用 DWT_InitFromSystemCoreClock() 自动根据 SystemCoreClock 初始化
 */
void DWT_Init(uint32_t CPU_Freq_MHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_MHz * 1000000u;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000u;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000u;
    CYCCNT_RountCount = 0;

    /* 初始化上次计数 */
    DWT_CNT_Update();
}

/**
 * @brief 便捷初始化：根据 SystemCoreClock 初始化（若工程提供 SystemCoreClock）
 * @note 若 hal_data 中未定义 SystemCoreClock，请改用 DWT_Init(<MHz>)
 */
void DWT_InitFromSystemCoreClock(void)
{
#ifdef SystemCoreClock
    /* SystemCoreClock 单位是 Hz */
    uint32_t mhz = (uint32_t)(SystemCoreClock / 1000000u);
    if (mhz == 0)
    {
        /* 若 SystemCoreClock < 1MHz，仍设置为 1MHz 避免除0 */
        mhz = 1u;
    }
    DWT_Init(mhz);
#else
    /* 无 SystemCoreClock 定义，使用默认 100 MHz（可根据芯片修改） */
    DWT_Init(100u);
#endif
}

/**
 * @brief 获取两次采样之间的时间增量（秒）
 * @param cnt_last: 指向上一次CYCCNT值的指针（调用者维护）
 * @return 秒数（float）
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    /* 无符号差值以支持溢出计算 */
    uint32_t diff = cnt_now - *cnt_last;
    float dt = ((float)diff) / ((float)CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief 获取两次采样之间的时间增量（秒），双精度版本
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    uint32_t diff = cnt_now - *cnt_last;
    double dt = ((double)diff) / ((double)CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = (float)SysTime.s + (float)SysTime.ms * 0.001f + (float)SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = (float)SysTime.s * 1000.0f + (float)SysTime.ms + (float)SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = (uint64_t)SysTime.s * 1000000ull + (uint64_t)SysTime.ms * 1000ull + (uint64_t)SysTime.us;

    return DWT_Timelinef32;
}

void DWT_Delay(float Delay_s)
{
    /* Delay_s 单位为秒，原实现接受任意单位。这里明确为秒。 */
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay_s;

    /* 防止 CPU_FREQ_Hz 为0 导致死循环 */
    if (CPU_FREQ_Hz == 0u)
    {
        return;
    }

    while ((uint32_t)(DWT->CYCCNT - tickstart) < (uint32_t)(wait * (float)CPU_FREQ_Hz))
        ;
}
