// #include "bluetooth.h"

// /* --- 内部控制变量 --- */
// static USARTInstance *bt_uart_instance = NULL;
// static bt_step_t current_step = BT_STEP_IDLE;
// static bool tx_awaiting_response = false;

// /* 行解析缓存 */
// static char line_buffer[128];
// static uint8_t line_idx = 0;

// /* 双机通信数据流缓存控制 (新增针对结构体解包的缓存) */
// static uint8_t data_rx_buf[sizeof(RX_BT_Data_s)];
// static uint8_t data_rx_idx = 0;

// /* 对外暴露的结构体实例 (供其他应用层使用) */
// RX_BT_Data_s bt_received_data;

// /* --- 内部函数声明 --- */
// static void BT_Hardware_Reset(void);
// static void BT_Send_Async(char *cmd);
// static void BT_Parse_Line(char *line);
// static uint16_t BT_Read_RingBuffer(uint8_t *buff, uint16_t len);
// static void BT_Process_TX(void); 
// static void BT_Parse_Binary_Data(uint8_t ch); // 专门处理透传阶段的数据帧

// /* --- 核心：中断回调触发处理 --- */
// static void BT_RxCallback(void) 
// {
//     uint8_t ch;
    
//     // 直接从 BSP 层提供的底层环形缓冲区中提取并消耗数据
//     while (BT_Read_RingBuffer(&ch, 1) > 0) 
//     {
//         // 关键分流：如果建立了透传连接，则完全不考虑字符匹配，改走二进制帧解包
//         if (current_step == BT_STEP_DATA_TRANS) 
//         {
//             BT_Parse_Binary_Data(ch);
//             continue;
//         }

//         // 遇到换行，或者特殊提示符 '>'，开始解析
//         if (ch == '\n' || (ch == '>' && current_step == BT_STEP_ENTER_SPP)) 
//         {
//             line_buffer[line_idx] = '\0';
//             BT_Parse_Line(line_buffer); 
//             line_idx = 0; // 重置行指针
//             memset(line_buffer, 0, sizeof(line_buffer));
//         }
//         else 
//         {
//             // --- 若在 AT 控制阶段，拼接行进行指令解析 ---
//             if (line_idx < sizeof(line_buffer) - 1) {
//                 line_buffer[line_idx++] = (char)ch;
//             }
//         }
//     }

//     // 数据解析完了，尝试驱动发送状态机
//     BT_Process_TX();
// }

// /* --- 解析接收透传的二进制固定帧 --- */
// static void BT_Parse_Binary_Data(uint8_t ch)
// {
//     // 如果找帧头的过程中，当前索引为0，但收到数据不是 0x5A，则丢弃
//     if (data_rx_idx == 0 && ch != 0x5A) {
//         return; 
//     }

//     // 将合法数据依次存入缓存
//     data_rx_buf[data_rx_idx++] = ch;

//     // 当收完了指定长度的一帧数据 `sizeof(RX_BT_Data_s)`
//     if (data_rx_idx >= sizeof(RX_BT_Data_s))
//     {
//         // 强制转换为结构体来校验和解析
//         RX_BT_Data_s *parsed_data = (RX_BT_Data_s *)data_rx_buf;
        
//         // 校验帧尾是否为 0xFFFB
//         if (parsed_data->tailer == 0xFFFB)
//         {
//             // 校验成功，拷贝存入全局结构体，供控制流读取
//             memcpy(&bt_received_data, parsed_data, sizeof(RX_BT_Data_s));
            
//             // TODO: 如果你想一旦收到指令就触发某个动作或马上回应，可以在这里调用你的应用层回调函数
//             // 例如：On_Motor_Target_Received();
//         }
        
//         // 无论结尾对不对，解析完一次必定清零索引，重新寻找下一个帧头 0xAA
//         data_rx_idx = 0;
//     }
// }

// /* --- 直接从 BSP 环形缓冲区提取数据 --- */
// static uint16_t BT_Read_RingBuffer(uint8_t *buff, uint16_t len)
// {
//     if (!bt_uart_instance || !buff || len == 0) return 0;
    
//     uint16_t count = 0;
//     // 直接操作 BSP 中的环形队列头尾指针提取数据 (依赖于 bsp_uart.c 的 RX_CHAR)
//     while (count < len && bt_uart_instance->tail != bt_uart_instance->head)
//     {
//         buff[count++] = bt_uart_instance->rx_buffer[bt_uart_instance->tail];
//         bt_uart_instance->tail = (bt_uart_instance->tail + 1) % UART_RING_BUFFER_SIZE;
//     }
//     return count;
// }

// /* --- 发送状态机 (被回调函数或初始化函数触发) --- */
// static void BT_Process_TX(void)
// {
//     if (tx_awaiting_response == false) 
//     {
//         switch (current_step)
//         {
//             // 注意：IDLE, HARD_RESET 和 WAIT_READY 已经移出这里放到 Init 里处理了
//             // 这里直接从发送首个指令开始
//             case BT_STEP_SEND_AT:
//                 BT_Send_Async("AT\r\n");
//                 break;

//             case BT_STEP_SEND_ECHO:
//                 BT_Send_Async("ATE0\r\n");
//                 break;

//             case BT_STEP_BLE_INIT:
//                 // W800 开启蓝牙并设为从机 (Peripheral)
//                 BT_Send_Async("AT+BLEINIT=2\r\n"); 
//                 break;

//             case BT_STEP_SET_NAME:
//                 BT_Send_Async("AT+BLENAME=\"W800_Motor_BLE\"\r\n");
//                 break;

//             case BT_STEP_ADV_START:
//                 BT_Send_Async("AT+BLEADVSTART\r\n");
//                 // 开启广告发完后，等待响应 OK
//                 break;

//             case BT_STEP_WAIT_CONN:
//                 // 等待手机建立连接，此时不发命令
//                 break;

//             case BT_STEP_ENTER_SPP:
//                 // 收到连接事件后，发送透传开启命令
//                 BT_Send_Async("AT+BLESPP=1\r\n");
//                 break;

//             case BT_STEP_DATA_TRANS:
//                 // 正在透传数据，不发送任何 AT 指令
//                 break;

//             case BT_STEP_ERROR:
//                 // 发生错误可以尝试重启指令流
//                 break;

//             default:
//                 break;
//         }
//     }
// }

// /* --- 解析收到的行，驱动状态流转 (在回调上下文中执行) --- */
// static void BT_Parse_Line(char *line)
// {
//     // 阶段1：特殊字符匹配 (透传提示符)
//     if (current_step == BT_STEP_ENTER_SPP) 
//     {
//         if (strstr(line, ">")) {
//             tx_awaiting_response = false;
//             current_step = BT_STEP_DATA_TRANS; 
//         }
//         return;
//     }

//     // 阶段2：被动监听连接事件
//     if (current_step == BT_STEP_WAIT_CONN) 
//     {
//         // W800 蓝牙连上后通常会上报这类似字符串 (请根据固件实际打印输出调整)
//         if (strstr(line, "+BLECONN") || strstr(line, "+BTCONN")) {
//             tx_awaiting_response = false;
//             current_step = BT_STEP_ENTER_SPP; 
//         }
//         return;
//     }

//     // 通用指令正常响应处理
//     if (strstr(line, "OK"))
//     {
//         if (tx_awaiting_response) 
//         {
//             tx_awaiting_response = false;
//             if (current_step < BT_STEP_WAIT_CONN) 
//             {
//                 current_step++; 
//             }
//         }
//     }
//     // 错误处理
//     else if (strstr(line, "ERROR"))
//     {
//         tx_awaiting_response = false;
//         current_step = BT_STEP_ERROR; 
//     }
    
//     // 阶段4：断开连接监听 (任何状态下如果监听到断开，就重回发广告状态)
//     if (strstr(line, "+BLEDISC") || strstr(line, "+BTDISC")) 
//     {
//         tx_awaiting_response = false;
//         current_step = BT_STEP_ADV_START; 
//         BT_Process_TX(); // 立即触发一次状态机，重新发广告
//     }
// }

// /* --- 发送与硬件控制 --- */
// static void BT_Send_Async(char *cmd)
// {
//     if (bt_uart_instance && bt_uart_instance->p_ctrl) 
//     {
//         R_SCI_UART_Write(bt_uart_instance->p_ctrl, (uint8_t *)cmd, strlen(cmd));
//         tx_awaiting_response = true; // 上锁等待回应
//     }
// }

// /* --- 公开 API --- */
// RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg)
// {
    
//     USART_Init_Config_s config;
//     config.recv_buff_size = 1; // 等待AT指令回复，接收不定长，必须强制1字节
//     config.p_uart_ctrl = p_ctrl;
//     config.p_uart_cfg  = p_cfg;
//     config.module_callback = BT_RxCallback; 
//     bt_uart_instance = USARTRegister(&config);
    
//     current_step = BT_STEP_SEND_AT;
//     tx_awaiting_response = false;

//     // 发第一条 AT
//     BT_Process_TX();

//     return &bt_received_data;
// }

// bool BT_IsConnected(void)
// {
//     return (current_step == BT_STEP_DATA_TRANS);
// }

// // 修改 BT_SendData：不再暴露原屎指针，而是直接提供打包功能给上层
// bool BT_SendData(uint8_t* tx_data, uint16_t len)
// {
//     if (current_step != BT_STEP_DATA_TRANS || tx_data == NULL) return false;

//     // 将结构体作为二进制字节流，整块推入 BSP 层
//     R_SCI_UART_Write(bt_uart_instance->p_ctrl, tx_data, len);
    
//     return true;
// }

// void test(void)
// {
//     BT_Send_Async("AT\r\n");
// }

// // static void BT_Hardware_Reset(void)
// // {
// //     R_BSP_PinWrite(BT_PIN_RESET, BSP_IO_LEVEL_LOW);
// //     R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS); 
// //     R_BSP_PinWrite(BT_PIN_RESET, BSP_IO_LEVEL_HIGH);
// //     R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS); // 等待启动完成吐出 Ready
// // }
