#include "new_imageRoad.h"
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "referee_protocol.h"

#define NE_RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *new_imageRoad_usart_instance; // 裁判系统串口实例
static DaemonInstance *new_imageRoad_daemon;

static remote_data_t *new_ImageRoad_info, new_imageRoad_info;
static New_ImageRoadRC new_imageRoad_key[2];  //新增图传遥控

New_ImageRoadRC test_imageRoad_key;

static uint8_t new_imageroad_lostflag;//与getlostflag向外提供接口链路是否丢失
static uint8_t new_imageroad_lostcount;
static uint8_t new_custom_lostflag;

static uint16_t new_referee_info_CmdID;
static custom_robot_data_t custom_info_data;

static uint8_t cus_flag = 0;
static uint8_t pre_cus_flag = 0;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&new_imageRoad_key[TEMP].rocker_l_ + i)) > 660)
            *(&new_imageRoad_key[TEMP].rocker_l_ + i) = 0;

}

/**
 * @brief 图传数据解析
 *
 */
static void KeyParse()
{
    //摇杆部分
    new_imageRoad_key[TEMP].rocker_l1 = new_ImageRoad_info->ch_2 - 1024;
    new_imageRoad_key[TEMP].rocker_l_ = -(new_ImageRoad_info->ch_3 - 1024);
    new_imageRoad_key[TEMP].rocker_r1 = new_ImageRoad_info->ch_1 - 1024;
    new_imageRoad_key[TEMP].rocker_r_ = new_ImageRoad_info->ch_0 - 1024;

    // //设置死区部分
    // // RectifyRCjoystick();

    // // //上端的拨轮与扳机
    new_imageRoad_key[TEMP].wheel = new_ImageRoad_info->wheel;
    new_imageRoad_key[TEMP].trigger = new_ImageRoad_info->trigger;

    // //三个自定义按键以及中间的挡位选择拨杆
    new_imageRoad_key[TEMP].pause = new_ImageRoad_info->pause;
    new_imageRoad_key[TEMP].mode_sw = new_ImageRoad_info->mode_sw;
    new_imageRoad_key[TEMP].left_buttom = new_ImageRoad_info->fn_1;
    new_imageRoad_key[TEMP].right_buttom = new_ImageRoad_info->fn_2;

    //鼠标解析
    // new_imageRoad_key[TEMP].mouse_x = new_ImageRoad_info->mouse_x;
    // new_imageRoad_key[TEMP].mouse_y = new_ImageRoad_info->mouse_y;
    // new_imageRoad_key[TEMP].mouse_z = new_ImageRoad_info->mouse_z;
    // new_imageRoad_key[TEMP].right_button_down = new_ImageRoad_info->mouse_right;
    // new_imageRoad_key[TEMP].left_button_down  = new_ImageRoad_info->mouse_left;
    // new_imageRoad_key[TEMP].mouse_middle = new_ImageRoad_info->mouse_middle;

    //改成位域
    *(uint16_t *)&new_imageRoad_key[TEMP].key[KEY_PRESS] = (uint16_t)(new_ImageRoad_info->key);  //改为此方法，更迅速，为减少延迟
    if (new_imageRoad_key[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
       new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL] = new_imageRoad_key[TEMP].key[KEY_PRESS];
    else
        memset(&new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (new_imageRoad_key[TEMP].key[KEY_PRESS].shift) // shift键按下
        new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT] = new_imageRoad_key[TEMP].key[KEY_PRESS];
    else
        memset(&new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    //计数解析
    uint16_t key_now = new_imageRoad_key[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = new_imageRoad_key[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = new_imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = new_imageRoad_key[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = new_imageRoad_key[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            new_imageRoad_key[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            new_imageRoad_key[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            new_imageRoad_key[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&new_imageRoad_key[LAST], &new_imageRoad_key[TEMP], sizeof(New_ImageRoadRC)); // 保存上一次的数据,用于按键持续按下和切换的判断
    memcpy(&test_imageRoad_key, &new_imageRoad_key[TEMP], sizeof(New_ImageRoadRC)); // 保存上一次的数据,用于按键持续按下和切换的判断

}

static void New_ImageRoadLostCallback(void *arg)
{
	USARTServiceInit(new_imageRoad_usart_instance);
    new_imageroad_lostflag = 1;
	LOGWARNING("[rm_img] lost referee data");
}

static void JudgeRoadData_new_imageRoad(uint8_t *buff)
{
    uint16_t judge_length;
	if (buff == NULL)
		return;


	if (buff[SOF] == 0xA9 && buff[SOF+1] == 0x53)
	{
		if (Verify_CRC16_Check_Sum(buff, 21) == TRUE) 
            memcpy(new_ImageRoad_info, buff, 21);

        new_imageroad_lostflag = 0;
	}

    if(buff[5]==0x02 || buff[6]==0x03)
    {
        (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE);
    }
    
	if (buff[SOF] == REFEREE_SOF)
	{
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// memcpy(&imageRoad_info, (buff + DATA_Offset), LEN_image_road);
            new_referee_info_CmdID = (buff[6] << 8 | buff[5]);
            // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
            // 第8个字节开始才是数据 data=7
			switch (new_referee_info_CmdID)
            {
            case ID_imageroad_selfcontroller:
                judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
                memcpy(&custom_info_data, (buff + DATA_Offset), LEN_image_customs);
                cus_flag++;
                break;

            default:
                break;
            }
		}

        pre_cus_flag = cus_flag;

        if(pre_cus_flag == cus_flag)
            new_custom_lostflag = 1;
        else if(pre_cus_flag != cus_flag)
            new_custom_lostflag = 0;

	}

}

static void New_ImageRoadRxCallback()
{
    DaemonReload(new_imageRoad_daemon);
    JudgeRoadData_new_imageRoad(new_imageRoad_usart_instance->recv_buff);
    KeyParse();
}

remote_data_t *New_ImageRoadInit(UART_HandleTypeDef *imageRoad_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = New_ImageRoadRxCallback;
	conf.usart_handle = imageRoad_usart_handle;
	conf.recv_buff_size = NE_RE_RX_BUFFER_SIZE; // mx 255(u8)
	new_imageRoad_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = New_ImageRoadLostCallback,
		.owner_id = new_imageRoad_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	new_imageRoad_daemon = DaemonRegister(&daemon_conf);

    new_custom_lostflag = 1;

	return &new_imageRoad_info;
}

remote_data_t *New_ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle)
{
    new_ImageRoad_info = New_ImageRoadInit(imageRoad_usart_handle);
    return new_ImageRoad_info;
}

void Get_Key_New(New_ImageRoadRC* KEY)
{
    memcpy(KEY,new_imageRoad_key,2*sizeof(New_ImageRoadRC));
}

uint8_t Get_img_lostflag(void)
{
    return new_imageroad_lostflag;
}

uint8_t Get_cus_lostflag(void)
{
    return new_custom_lostflag;
}

custom_robot_data_t* Get_custom_controller_data()//自定义控制器数据接口
{
    return &custom_info_data;
}