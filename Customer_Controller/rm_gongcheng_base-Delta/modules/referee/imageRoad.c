/**
 * @file imageRoad.c
 * @author Jiaxing He (1756735553@qq.com)
 * @version 0.1
 * @date 2024-10-20
 *
 */
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
#include "imageRoad.h"
#include "referee_protocol.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *imageRoad_usart_instance; // 裁判系统串口实例
static remote_control_t *ImageRoad_info,imageRoad_info;
static custom_robot_data_t custom_info;
static DaemonInstance *imageRoad_daemon;
static ImageRoadRC imageRoad_key[2];  //新增图传遥控

static uint8_t imageroad_lostflag;//与getlostflag向外提供接口链路是否丢失
static uint8_t imageroad_lostcount;
static uint16_t referee_info_CmdID;
/**
 * @brief 图传数据解析
 *
 */
static void KeyParse()
{
    // for(int i = 0; i < 16; i++){  //感觉这样延迟更高
    //     if(imageRoad_data->keyboard_value&(1<<i)){
    //         imageRoad_key[TEMP].key[KEY_PRESS][i]=1;
    //     }
    //     else{
    //         imageRoad_key[TEMP].key[KEY_PRESS][i]=0;
    //     }
    // }
    //改成位域
    *(uint16_t *)&imageRoad_key[TEMP].key[KEY_PRESS] = (uint16_t)(ImageRoad_info->keyboard_value);  //改为此方法，更迅速，为减少延迟
    if (imageRoad_key[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
       imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL] = imageRoad_key[TEMP].key[KEY_PRESS];
    else
        memset(&imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (imageRoad_key[TEMP].key[KEY_PRESS].shift) // shift键按下
        imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT] = imageRoad_key[TEMP].key[KEY_PRESS];
    else
        memset(&imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    //计数解析
    uint16_t key_now = imageRoad_key[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = imageRoad_key[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = imageRoad_key[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = imageRoad_key[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = imageRoad_key[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = imageRoad_key[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            imageRoad_key[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            imageRoad_key[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            imageRoad_key[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&imageRoad_key[LAST], &imageRoad_key[TEMP], sizeof(ImageRoadRC)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

static void ImageRoadLostCallback(void *arg)
{
	USARTServiceInit(imageRoad_usart_instance);
    imageroad_lostflag=1;
	LOGWARNING("[rm_img] lost referee data");
}
static void JudgeReadData_imageRoad(uint8_t *buff)
{
	uint16_t judge_length;
	if (buff == NULL)
		return;

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
            referee_info_CmdID = (buff[6] << 8 | buff[5]);
            // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
            // 第8个字节开始才是数据 data=7
			switch (referee_info_CmdID)
            {
            case ID_imageroad_selfcontroller:
                judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
                memcpy(&custom_info, (buff + DATA_Offset), LEN_image_customs);
                break;
            case ID_imageroad_mousekey:
                judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
                memcpy(&imageRoad_info, (buff + DATA_Offset), LEN_image_road);
                break;
            default:
                break;
            }
            imageroad_lostflag = 0;
		}
	}
}
static void ImageRoadRxCallback()
{
	DaemonReload(imageRoad_daemon);
	JudgeReadData_imageRoad(imageRoad_usart_instance->recv_buff);
    KeyParse();
}
remote_control_t *ImageRoadInit(UART_HandleTypeDef *imageRoad_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = ImageRoadRxCallback;
	conf.usart_handle = imageRoad_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	imageRoad_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = ImageRoadLostCallback,
		.owner_id = imageRoad_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	imageRoad_daemon = DaemonRegister(&daemon_conf);

	return &imageRoad_info;
}

remote_control_t *ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle)
{
    ImageRoad_info = ImageRoadInit(imageRoad_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    return ImageRoad_info;
}

void Get_Key(ImageRoadRC* KEY)
{
    memcpy(KEY,imageRoad_key,2*sizeof(ImageRoadRC));
}

uint8_t Get_imgroad_lostflag(void) //提供判断是否丢失信号的接口
{
    return imageroad_lostflag;
}
