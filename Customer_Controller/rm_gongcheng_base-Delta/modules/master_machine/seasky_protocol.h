#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;

typedef struct
{
    uint8_t header;
    uint8_t tracking : 1;
    uint8_t id : 3;
    uint8_t armors_num : 3;
    uint8_t reserved : 1;
    uint8_t x1;
    uint8_t x2;
    uint8_t y1;
    uint8_t y2;
    uint8_t z1;
    uint8_t z2;
	uint8_t pitch1;
	uint8_t pitch2;
    uint8_t yaw1;
    uint8_t yaw2;
    uint8_t vx1;
    uint8_t vx2;
    uint8_t vy1;
    uint8_t vy2;
    uint8_t vz1;
    uint8_t vz2;
    uint8_t v_yaw1;
    uint8_t v_yaw2;
    uint8_t r1_1;
    uint8_t r1_2;
    uint8_t r2_1;
    uint8_t r2_2;
    uint8_t dz1;
    uint8_t dz2;
	// uint8_t cap_timestamp1;
	// uint8_t cap_timestamp2;
	// uint8_t cap_timestamp3;
	// uint8_t cap_timestamp4;
	// uint8_t t_offset1;
	// uint8_t t_offset2;
    uint8_t check_sum1;
    uint8_t check_sum2;
} RX_PACKET;

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_data(uint16_t send_id,		 // 信号id
							uint16_t flags_register, // 16位寄存器
							float *tx_data,			 // 待发送的float数据
							uint8_t float_length,	 // float的数据长度
							uint8_t *tx_buf,		 // 待发送的数据帧
							uint16_t *tx_buf_len);	 // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   uint16_t *flags_register, // 接收数据的16位寄存器地址
						   uint8_t *rx_data);			 // 接收的float数据存储地址


void get_yaw_pitch(uint8_t *rx_buf,uint8_t *rx_data);
float Get_yaw(void);
float Get_pitch(void);
float Get_flag(void);
RX_PACKET* Get_rxpack(void);
#endif
