#include "common_inc.h"


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000);

RGB rgb(0);
// Robot instance
DummyRobot dummy(&hcan1);


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
//controlLoopFixUpdateHandle 是该线程的句柄，用于标识和操作线程。
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY)）。该线程在没有任务通知时会一直阻塞。

        if (dummy.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            //一旦接收到任务通知，线程首先检查机器人是否启用
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);
                    dummy.UpdateJointPose6D();
                    break;
// 如果机器人启用，根据不同的指令模式执行相应的操作：
//如果是 COMMAND_TARGET_POINT_SEQUENTIAL, COMMAND_TARGET_POINT_INTERRUPTABLE, 
//或 COMMAND_CONTINUES_TRAJECTORY，
//则通过调用 dummy.MoveJoints(dummy.targetJoints) 
//发送控制命令到电机，并更新关节状态和六自由度姿态

                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    break;
//如果是 COMMAND_MOTOR_TUNING，
//则调用 dummy.tuningHelper.Tick(10) 执行调谐，
//并更新六自由度姿态。



            }
        } else
        {
            // Just update Joint states
            dummy.UpdateJointAngles();
            dummy.UpdateJointPose6D();
        }
//如果机器人未启用，只会更新关节状态和六自由度姿态。
//ThreadControlLoopUpdate 线程：
//ControlLoopUpdateHandle 是该线程的句柄，用于标识和操作线程。
//线程进入无限循环，在循环中通过 dummy.commandHandler.Pop(osWaitForever) 
//从 dummy.commandHandler 中弹出指令，
//并通过 dummy.commandHandler.ParseCommand() 解析和处理该指令。

        
    }
}


osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}


osThreadId_t oledTaskHandle;
//该函数是一个FreeRTOS线程，用于更新OLED显示屏上的信息。
//在每次循环中，首先获取当前时间戳 t，然后调用mpu6050.Update(true)来获取MPU6050传感器的数据。
//使用oled对象操作OLED显示屏，显示IMU的加速度数据、帧率、关节角度、机器人姿态等信息。
void ThreadOledUpdate(void* argument)
{
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    for (;;)
    {
        // 获取IMU数据
        mpu6050.Update(true);

        // 清空缓冲区
        oled.clearBuffer();
        // 显示IMU的加速度数据和帧率
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", 1000000 / (micros() - t));
        t = micros();
        // 绘制关节角度信息
        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
                    (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
                    (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));

        // 绘制机器人姿态信息
        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
                    (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
                    (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");
        // 绘制机器人状态信息
        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        if (dummy.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
        }

        // 将缓冲区内容发送到OLED显示屏
        oled.sendBuffer();
    }
    //
//这是一个用于更新OLED显示屏的线程，主要通过读取MPU6050传感器的数据和机器人的状态信息来实现。以下是代码的主要功能和结构解释：
//线程函数 ThreadOledUpdate:
//该函数是一个FreeRTOS线程，用于更新OLED显示屏上的信息。
//在每次循环中，首先获取当前时间戳 t，然后调用mpu6050.Update(true)来获取MPU6050传感器的数据。
//使用oled对象操作OLED显示屏，显示IMU的加速度数据、帧率、关节角度、机器人姿态等信息。
cpp
Copy code
osThreadId_t oledTaskHandle;
void ThreadOledUpdate(void* argument)
{
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    for (;;)
    {
        // 获取IMU数据
        mpu6050.Update(true);

        // 清空缓冲区
        oled.clearBuffer();
        
        // 显示IMU的加速度数据和帧率
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", 1000000 / (micros() - t));
        t = micros();

        // 绘制关节角度信息
        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
                    (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
                    (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));

        // 绘制机器人姿态信息
        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
                    (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
                    (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        // 绘制机器人状态信息
        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        if (dummy.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
        }

        // 将缓冲区内容发送到OLED显示屏
        oled.sendBuffer();
    }
}
//变量说明:
//t: 上一个时间戳，用于计算帧率。
//buf: 用于存储机器人状态的缓冲区。
//cmdModeNames: 机器人命令模式的名称，用于显示在OLED上。
//OLED显示内容:

//显示了IMU的X和Y轴加速度数据，帧率，6个关节的角度，机器人的位置（XYZ）和姿态（ABC），以及机器人的工作状态和命令模式。
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

osThreadId_t rgbTaskHandle;
void ThreadRGBUpdate(void* argument) {
    for (;;) {
        if (dummy.GetRGBEnabled())
        {
            rgb.Run((RGB::Rgb_style_t)dummy.GetRGBMode());
            osDelay(30);
        }else
        {
            rgb.Run(RGB::ALLOff);
            osDelay(30);
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2)
    {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
        rgb.Interrupt(1);
    }
}

/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init IMU.
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    const osThreadAttr_t rgbTask_attributes = {
            .name = "RGBTask",
            .stack_size = 2000,
            .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    rgbTaskHandle = osThreadNew(ThreadRGBUpdate, nullptr, &rgbTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}

