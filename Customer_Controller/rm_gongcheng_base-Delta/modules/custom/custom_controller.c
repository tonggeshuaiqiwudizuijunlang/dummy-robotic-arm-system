#include "custom_controller.h"
#include "string.h"
#include "stdlib.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "referee_protocol.h"

static uint16_t new_referee_info_CmdID;

static Video_transmission_link_struct *video_instance;
static osThreadId custom_task_handle;
static Re_Custom_data_t re_custom_data;
/**
 * @brief 机器人回传数据解析函数 (基于命令码 0x0309)
 * @param rx_buf 接收缓冲区指针
 * @param len    接收到的数据长度
 */
void Tran_Link_Parse(uint8_t *rx_buf, uint16_t len)
{
    // 1. 最小数据长度判断 (帧头5 + 命令码2 + 包尾2 = 9字节)
    if (rx_buf == NULL || len < 9)
        return;

    // 2. SOF 校验
    if (rx_buf[0] != 0xA5)
        return;

    // 3. 帧头 CRC8 校验
    if (Verify_CRC8_Check_Sum(rx_buf, 5) == 0)
        return;

    // 4. 解析数据长度与命令码 (小端模式转换)
    uint16_t data_length = rx_buf[1] | (rx_buf[2] << 8);
    uint16_t cmd_id = rx_buf[5] | (rx_buf[6] << 8);

    // 5. 校验整包长度与包尾 CRC16 校验
    if (len >= data_length + 9)
    {
        if (Verify_CRC16_Check_Sum(rx_buf, data_length + 9) != 0)
        {
            // 6. 提取 0x0309 自定义控制器接收的机器人数据
            if (cmd_id == 0x0309 && data_length == 30)
            {
                if (video_instance != NULL)
                {
                    // TODO: 在头文件 Video_transmission_link_struct 结构体中添加接收数组，例如 uint8_t robot_recv_data[30];
                    memcpy(video_instance->data_recb_buf, &rx_buf[7], 30);
                }
            }
        }
    }
}
/*
 * @brief 仿照湖大框架编写的decode函数
 */
static void JudgeRoad_imageRoad(uint8_t *buff)
{
    uint16_t judge_length;
    uint8_t Offest_Hex = 0;
    for (Offest_Hex; Offest_Hex < 255; Offest_Hex++)
    {
        if (buff[Offest_Hex] == 0xa5)
        {
            judge_length = 0;
            break;
        }
    }
    if (buff == NULL)
        return;

    if (buff[5 + Offest_Hex] == 0x02 || buff[6 + Offest_Hex] == 0x03)
    {
        (Verify_CRC8_Check_Sum(buff + Offest_Hex, LEN_HEADER) == TRUE);
    }
    if (buff[SOF + Offest_Hex] == REFEREE_SOF)
    {
        if (Verify_CRC8_Check_Sum(buff + Offest_Hex, LEN_HEADER) == TRUE)
        {
            // judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // memcpy(&imageRoad_info, (buff + DATA_Offset), LEN_image_road);
            new_referee_info_CmdID = (buff[6 + Offest_Hex] << 8 | buff[5 + Offest_Hex]);
            // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
            // 第8个字节开始才是数据 data=7
            switch (new_referee_info_CmdID)
            {
            case 0x0309:
                judge_length = buff[DATA_LENGTH + Offest_Hex] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
                memcpy((uint8_t*)&re_custom_data, (buff + DATA_Offset + Offest_Hex), LEN_image_customs);
                break;

            default:
                break;
            }
        }
    }
}

void Tran_Link_RxCallback()
{
    if (video_instance != NULL && video_instance->uart_ins != NULL)
    {
        // 提取底层实例的接收 buffer 并传入原有的解析函数
        // 根据之前的计算，定长为 39 字节
        JudgeRoad_imageRoad(video_instance->uart_ins->recv_buff);
    }
    // video_instance->data_recb_buf
}
Video_transmission_link_struct *Tran_Link_Init(UART_HandleTypeDef *HUART)
{
    Video_transmission_link_struct *custom_controller = (Video_transmission_link_struct *)malloc(sizeof(Video_transmission_link_struct));
    USART_Init_Config_s conf;
    conf.usart_handle = HUART;
    conf.recv_buff_size = 255u;
    conf.module_callback = Tran_Link_RxCallback;
    custom_controller->uart_ins = USARTRegister(&conf);
    video_instance = custom_controller;
    return custom_controller;
}
void Tran_Link_customdata_float_to_hex(Video_transmission_link_struct *custom_controller,
                                       const Custom_data_t data)
{
    memcpy(&custom_controller->CustomControl_Data, (uint8_t *)&data, 30);
}
void Tran_Link_recustomdata_float_to_hex(Video_transmission_link_struct *custom_controller,
                                         Re_Custom_data_t *data)
{
    if (data == NULL || custom_controller == NULL)
        return;
    memcpy((uint8_t *)data, &custom_controller->data_recb_buf, 30);
}
// 是图传链路，跟上海大学SRM战队的开源对应上了
static int16_t Tran_Link_Send(Video_transmission_link_struct *Video_transmission_link)
{
    uint16_t CMD_ID = 0x0302;
    uint8_t *frame_point; // 读写指针
    uint16_t frametail = 0xFFFF;
    uint16_t data_length = 30;
    robot_interaction_data_t robot_interaction_data_send; // 交互数据接收信息

    /* 完成帧头打包 */
    frame_header Frame_head_send; // 帧头
    frame_point = Video_transmission_link->data_send_buf;
    Frame_head_send.SOF = 0xA5;
    Frame_head_send.data_length = data_length;
    Frame_head_send.seq = Video_transmission_link->data_seq++;

    memcpy(Video_transmission_link->data_send_buf, &Frame_head_send.SOF, 1);
    memcpy(Video_transmission_link->data_send_buf + 1, &Frame_head_send.data_length, 2);
    memcpy(Video_transmission_link->data_send_buf + 3, &Frame_head_send.seq, 1);
    Frame_head_send.CRC8 = Get_CRC8_Check_Sum(frame_point, 4, 0xFF);
    memcpy(Video_transmission_link->data_send_buf + 4, &Frame_head_send.CRC8, 1);
    memcpy(Video_transmission_link->data_send_buf + 5, &CMD_ID, 2);
    memcpy(Video_transmission_link->data_send_buf + 7, Video_transmission_link->CustomControl_Data.data, data_length);

    /* 整包校验 */
    frametail = Get_CRC16_Check_Sum(Video_transmission_link->data_send_buf, Frame_head_send.data_length + 7, 0xFFFF);
    memcpy(Video_transmission_link->data_send_buf + Frame_head_send.data_length + 7, &frametail, 2);

    /* 数据发送 */
    USARTSend(Video_transmission_link->uart_ins, Video_transmission_link->data_send_buf, Frame_head_send.data_length + 9, USART_TRANSFER_DMA);
    return 0;
}
void Custom_Task(void const *argument)
{
    Video_transmission_link_struct *motor = (Video_transmission_link_struct *)argument;
    while (1)
    {
        Tran_Link_Send(motor);
        osDelay(40);
    }
}
void Custom_Task_Init(void)
{
    char custom_task_name[12] = "custom_task";
    osThreadDef(custom_task_name, Custom_Task, osPriorityNormal, 0, 512);
    custom_task_handle = osThreadCreate(osThread(custom_task_name), video_instance);
}
Re_Custom_data_t *Recustom_get(void)
{
    return &re_custom_data;
}