# XLmotor使用说明和补充

<span style="font-size: 20px;">**==特别提醒==**</span>

1. 该电机采用开源的驱动板，can稳定性有待提高，当用can分析仪检查电机心跳发现没有反应时（前提是波特率配对了哦），请怀疑是否是电机can坏了,可以的话请压力公司技术
2. 同上，因为电机驱动板稳定性不行，__<span style="color: red;">请断电后再对电机进行各种插拔行为</span>__，尽可能的确保电机不会烧坏
3. 电机驱动板有最大电流值参数，当驱动电机所需电流超过该值时电机会停止并进入错误模式,该模式下必须执行“读取错误”与“清除错误”两个命令后才能重新控制电机
>>驱动电机所需电流：以上位机控制电机，模式为位置模式为例，当目标值(蓝色指针)与实际值(白色指针)相差过大时，电机会以非常快的速度达到目标值，此时电流极有可能超过限制，电机会停止并进入错误模式
4. 上位机可以使用odrivetool或者是电机精灵,如果选择电机精灵需要为电机USB接口安装libusb驱动，与说明文档里面的并不一致，且电机固件必须是0.5.13以上(不过不出意外的话我们的电机应该是可以用的,因为我就在用电机精灵调参)
5. 使用位置模式与速度模式电机会自带PID,可以使用电机精灵直接调参,这样调完就可以直接使用


<span style="font-size: 20px;">**==can帧格式说明==**</span>
>>请注意，在该电机中发送和接收的can.ID并非唯一，故与电机进行can通信时请注意canID的变化

 1. ![alt text](can帧格式.png)
   1. ![alt text](通信数据格式-1.png)
   2. ![alt text](通信数据格式-2.png)

<span style="font-size: 20px;">**==上位机常用命令==**</span>

 1. dump_errors(odrv0) 打印所有的错误信息 
 2. odrv0.clear_errors() 清除所有的错误信息

 3. odrv0.save_configuration() 保存修改参数，在输入该命令后电机会重启，断联一会后再重新连接，故显示电机断联时请不用惊慌
  
 4. odrv0.axis0.requested_state=1 停止电机，进⼊空闲状态 
 5. odrv0.axis0.requested_state=8 启动电机，进⼊闭环状态
  
 6. odrv0.axis0.controller.config.control_mode 控制模式 
 >> 0：电压控制 1：力矩控制 2：速度控制 3：位置控制

 7. odrv0.axis0.controller.config.input_mode 输⼊模式
 >> 0：闲置 1：直接控制 2：速度斜坡 3：位置滤波 5：梯形曲线 6：力矩斜坡 9：运动控制（MIT）

 8. odrv0.can.config.r120_gpio_num 控制 CAN 接口的 120R 匹配电阻开关的 GPIO 号 
 9. odrv0.can.config.enable_r120 控制 CAN 接口的 120R 匹配电阻开关
 10. odrv0.can.config.baud_rate CAN 的波特率设置


<span style="font-size: 20px;">**==心跳消息补充说明==**</span>

1. 如果需要电机主动发送编码器位置，请启用:Get_Encoder_Count命令而不是Get_Sensorless_Error命令，因为我之前就是启用这个命令结果没反应
2. 心跳消息可作为检查电机can通信功能的最后一个手段，当电机的心跳消息都发不出来的时候can多半是坏了

![alt text](心跳消息总览.png)

