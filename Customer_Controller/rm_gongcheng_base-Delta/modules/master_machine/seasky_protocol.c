/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief 湖南大学RoBoMatster串口通信协议
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
//#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

RX_PACKET rx_packet;
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check = rx_buf[3];
            pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            float *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len = float_length * 4 + 2;
    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;        // 低位在前
    tx_buf[2] = (data_len >> 8) & 0xff; // 低位在前
    tx_buf[3] = crc_8(&tx_buf[0], 3);   // 获取CRC8校验位

    /*数据的信号id*/
    tx_buf[4] = send_id & 0xff;
    tx_buf[5] = (send_id >> 8) & 0xff;

    /*建立16位寄存器*/
    tx_buf[6] = flags_register & 0xff;
    tx_buf[7] = (flags_register >> 8) & 0xff;

    /*float数据段*/
    for (int i = 0; i < 4 * float_length; i++)
    {
        tx_buf[i + 8] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
    }

    /*整包校验*/
    crc16 = crc_16(&tx_buf[0], data_len + 6);
    tx_buf[data_len + 6] = crc16 & 0xff;
    tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 8;
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据
                           uint16_t *flags_register, // 接收数据的16位寄存器地址
                           uint8_t *rx_data)         // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf))
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        if (CRC16_Check_Sum(&rx_buf[0], date_length))
        {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];
            memcpy(rx_data, rx_buf + 8, pro.header.data_length - 2);
            return pro.cmd_id;
        }
    }
    return 0;
}

//add
#include "math.h"
//RX_PACKET rx_packet;
#define g 9.780665f
#define PI 3.14159265358979f
float initial_speed = 27.4;
//
float angle_pitch, angle_yaw;
float alpha, theta;
float x = 0.0f, y = 0.0f, z = 0.0f, length = 0.0f;
int16_t px = 0, py = 0, pz = 0;
int16_t pitch_x=0,yaw_x=0;

float angle_to_rad(float angle)
{
    float rad = 0.0f;
    rad = angle / 180.0 * PI;
    return rad;
}

float get_pitch_angle()
{

    uint8_t *ptr_x, *ptr_y, *ptr_z;

    float temp_angle = 0.0f;

    static uint8_t positive_cnt = 0;

    ptr_x = (uint8_t *)&px;
    ptr_x[0] = rx_packet.x1;
    ptr_x[1] = rx_packet.x2;

    ptr_y = (uint8_t *)&py;
    ptr_y[0] = rx_packet.y1;
    ptr_y[1] = rx_packet.y2;

    ptr_z = (uint8_t *)&pz;
    ptr_z[0] = rx_packet.z1;
    ptr_z[1] = rx_packet.z2;

    x = px / 1000.0;
    y = py / 1000.0;
    z = pz / 1000.0 - 0.10f;

    length = sqrt(pow(x, 2) + pow(z, 2));

    if(z + g * x * x / (initial_speed * initial_speed) <= length)
    {
        alpha = asinf((z + g * x * x / (initial_speed * initial_speed)) / length);
        theta = atanf(z / x);

        if(alpha < PI / 2.0)
        {
            angle_pitch = (alpha + theta) / 2.0;
        }

        else if(alpha > PI / 2.0)
        {
            angle_pitch = (PI - alpha + theta) / 2.0;
        }
    }

    temp_angle = angle_pitch;
    angle_pitch = 0.0f;

    return (temp_angle);
}

float get_yaw_angle()
{
    float temp_angle;

    uint8_t *ptr_x, *ptr_y;
    ptr_x = (uint8_t *)&px;
    ptr_x[0] = rx_packet.x1;
    ptr_x[1] = rx_packet.x2;

    ptr_y = (uint8_t *)&py;
    ptr_y[0] = rx_packet.y1;
    ptr_y[1] = rx_packet.y2;

    x = px / 1000.0;
    y = py / 1000.0f;

    if(x > 3.50f)
    {
        y += 0.07f;
    }
    else
    {
        y -= 0.05f;
    }

    angle_yaw = atan((y) / x);

    temp_angle = angle_yaw;

    angle_yaw = 0.0f;

    return temp_angle;
}
const uint16_t W_CRC_TABLE1[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Get_CRC16_Check_Sum1(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;

    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }

    while (dwLength--)
    {
        ch_data = *pchMessage++;
        (wCRC) =
            ((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE1[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
    }

    return wCRC;
}
uint32_t Verify_CRC16_Check_Sum1(const uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_expected = 0;

    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }

    w_expected = Get_CRC16_Check_Sum1(pchMessage, dwLength - 2, 0xffff);
    return (
               (w_expected & 0xff) == pchMessage[dwLength - 2] &&
               ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}


float pitch_turn = 0,yaw_turn = 0;
float flag_vision = 0;
void get_yaw_pitch(uint8_t *rx_buf,          // 接收到的原始数据
                           uint8_t *rx_data)         // 接收的float数据存储地址
{
    uint8_t i = 0;
    uint8_t pos = 0;
    if(rx_buf[0]==PROTOCOL_CMD_ID){
        // if(Verify_CRC16_Check_Sum1((const uint8_t *)&rx_buf, sizeof(rx_packet))){
        //if(CRC16_Check_Sum((uint8_t *)&rx_packet, sizeof(rx_packet))){
        // if(CRC16_Check_Sum(&rx_buf[0], sizeof(rx_packet))){
            // for (i = 0; i < sizeof(rx_packet); i++)
            // {
            //     *((uint8_t *)&rx_packet + pos) = rx_buf[i];
            //     pos++;
            //     rx_buf[i] = 0;
            // }
            memcpy(&rx_packet,rx_buf,sizeof(rx_packet));
        // }
        
        // uint8_t *pc, *yw;
        // pc = (uint8_t *)&pitch_x;
        // pc[0] = rx_packet.pitch1;
        // pc[1] = rx_packet.pitch2;
        // yw = (uint8_t *)&yaw_x;
        // yw[0] = rx_packet.yaw1;
        // yw[1] = rx_packet.yaw2;

        // pitch_turn = pitch_x/1000.0f;
        // yaw_turn = yaw_x/1000.0f;

        pitch_turn = get_pitch_angle();
        yaw_turn = get_yaw_angle();
        
        flag_vision = 1;
    }
}
float Get_yaw(void){
    flag_vision=0;
    return yaw_turn;
}
float Get_pitch(void){
    flag_vision=0;
    return pitch_turn;
}
float Get_flag(void){
    return flag_vision;
}
RX_PACKET* Get_rxpack(void){
    return &rx_packet;
}