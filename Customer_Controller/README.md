# 🤖 Customer Controller - 双MCU机器人控制器

基于 **Renesas RA6M5 + STM32F407** 双控制器架构的工业机器人控制系统，专为Delta机器人设计，支持实时运动学解算、多传感器融合和VOFA+数据可视化。

## ✨ 主要特性

### 双控制器架构
- **主控制器 (Renesas RA6M5)** - 高级逻辑处理、通信管理、任务调度
- **从控制器 (STM32F407)** - 实时电机控制、运动学解算、传感器数据采集
- **CAN总线通信** - 主从控制器高速数据交换

### 核心功能
- **Delta机器人控制** - 3/4轴Delta并联机器人运动控制
- **正运动学 (FK)** - 根据关节角度计算末端执行器位置
- **逆运动学 (IK)** - 根据目标位置计算各轴运动参数
- **实时轨迹规划** - 平滑的运动轨迹生成

### 控制模式
- **VOFA+调试** - 实时数据可视化和参数调整
- **串口调试** - UART命令行调试接口
- **CAN通信** - 多设备协同控制

### 高级功能
- **四元数姿态解算** - 基于EKF的精确姿态估计
- **电机闭环控制** - PID位置/速度/电流三环控制
- **多任务调度** - FreeRTOS实时任务管理
- **故障检测** - 实时状态监控和错误处理

## 🛠️ 硬件要求

### 主控制器 (Renesas RA6M5)
- **MCU**: R7FA6M5BH
- **内核**: ARM Cortex-M33
- **主频**: 200MHz
- **Flash**: 2MB
- **RAM**: 512KB
- **外设**: UART, CAN, GPIO, Timer

### 从控制器 (STM32F407)
- **MCU**: STM32F407IGHx
- **内核**: ARM Cortex-M4
- **主频**: 168MHz
- **Flash**: 1MB
- **RAM**: 192KB
- **外设**: CAN, SPI, I2C, PWM, ADC, Encoder

### 电机驱动
- **步进电机** - Delta机器人各轴驱动
- **伺服电机** - 可选的高精度电机
- **编码器** - 位置反馈传感器

## 📁 项目结构

```
Customer_Controller/
├── application/                    # 主控制器应用层
│   ├── dummy_cmd.c/h             # 命令解析和处理
│   ├── dummy_motormatic.c/h      # 电机运动控制
│   └── bsp/                      # 板级支持包
├── modules/                       # 主控制器功能模块
│   ├── bluetooth/                # 蓝牙通信
│   ├── controller/               # 控制器抽象层
│   ├── dummy_kinematic/          # 运动学解算
│   ├── vision/                   # 视觉识别
│   ├── step_motor/               # 步进电机驱动
│   ├── can_comm/                 # CAN通信
│   └── daemon/                   # 守护任务
├── rm_gongcheng_base-Delta/       # STM32 Delta机器人工程
│   ├── application/              # 应用层
│   │   ├── cmd/                  # 命令处理
│   │   ├── gimbal/               # 云台控制
│   │   └── robot_def.h           # 机器人定义
│   ├── modules/                  # 功能模块
│   │   ├── algorithm/            # 算法库
│   │   │   ├── QuaternionEKF.c/h # 四元数EKF
│   │   │   └── user_lib.c/h     # 用户库
│   │   ├── delta_cal/            # Delta运动学
│   │   ├── Arm_Moveit/           # 机械臂运动规划
│   │   └── custom/               # 定制控制器
│   ├── bsp/                      # 板级支持包
│   ├── Drivers/                  # STM32 HAL驱动
│   └── Middlewares/              # 中间件(FreeRTOS, DSP)
├── Dummy 源码注释/                 # 源码注释和文档
├── DebugConfig/                   # 调试配置
├── .vscode/                       # VS Code配置
└── *.csv                          # 测试数据文件
```

## 🚀 快速开始

### 1. 开发环境

**必需工具：**
- [Keil MDK](https://www.keil.com/) v5.38+ (主控制器)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) 或 [CLion](https://www.jetbrains.com/clion/) (从控制器)
- [Renesas FSP](https://www.renesas.com/us/en/software-tool/flexible-software-package-fsp) v4.0+
- [VOFA+](https://www.vofa.plus/) - 数据可视化工具
- J-Link / ST-Link调试器

**可选工具：**
- [VS Code](https://code.visualstudio.com/) + Keil Assistant / Cortex-Debug插件
- 串口调试工具（如SSCOM、PuTTY、XCOM）

### 2. 编译步骤

#### 主控制器 (Renesas RA6M5)
1. 使用Keil MDK打开 `FSP_Project.uvprojx`
2. 确保已安装Renesas RA系列芯片支持包
3. 点击"Build"按钮（F7）编译

#### 从控制器 (STM32F407)
1. 进入 `rm_gongcheng_base-Delta/` 目录
2. 使用STM32CubeIDE导入工程，或使用Makefile编译：
   ```bash
   cd rm_gongcheng_base-Delta
   make
   ```

### 3. 烧录和调试

#### 主控制器
1. 连接J-Link到SWD接口
2. 在Keil中点击"Download"（F8）
3. 使用"Debug"（Ctrl+F5）进行调试

#### 从控制器
1. 连接ST-Link到SWD接口
2. 使用STM32CubeIDE烧录，或：
   ```bash
   make flash
   ```

### 4. 运行和调试

1. **连接VOFA+**
   - 打开VOFA+软件
   - 选择串口/网络连接
   - 配置数据通道

2. **发送控制指令**
   - 使用串口终端发送命令
   - 或通过VOFA+调整参数

## ⚙️ 配置说明

### 主控制器配置
在 `application/dummy_cmd.h` 中配置：

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `DUMMY_CMD_UART_MODE` | `BLUETOOTH` | 串口模式 |
| `DUMMY_CMD_FKIK_TEST_ENABLE` | `0` | 正逆解测试模式 |
| `DUMMY_CMD_REMOTE_POSE_TEST_ENABLE` | `0` | 遥控器姿态测试 |

### 从控制器配置
在 `rm_gongcheng_base-Delta/application/robot_def.h` 中配置：

```c
// 机器人类型定义
#define DELTA_ROBOT       // Delta机器人
#define ROBOT_AXIS_NUM 3  // 轴数量

// 运动学参数
#define DELTA_BASE_RADIUS 100.0f  // 基座半径(mm)
#define DELTA_ARM_LENGTH  200.0f  // 臂长(mm)
#define DELTA_EFF_RADIUS  50.0f   // 末端半径(mm)
```

## 📡 通信协议

### 主从通信 (CAN)
- **波特率**: 1Mbps
- **协议**: CAN 2.0B
- **数据格式**: 自定义二进制协议

### 调试串口 (UART)
- **波特率**: 115200
- **数据格式**: 8N1
- **协议**: 文本命令 / 二进制数据

### VOFA+通信
- **协议**: FireWater / RawData
- **波特率**: 115200
- **数据格式**: CSV / 二进制

## 🎮 使用说明

### VOFA+调试模式
1. 连接VOFA+到调试串口
2. 配置数据通道和显示波形
3. 实时调整PID参数和运动参数
4. 记录和分析测试数据

### 命令行调试
1. 打开串口终端（115200, 8N1）
2. 输入命令：
   ```
   help          # 显示帮助
   fk <a1> <a2> <a3>  # 正运动学解算
   ik <x> <y> <z>     # 逆运动学解算
   move <x> <y> <z>   # 移动到目标位置
   ```

### 自动运行模式
1. 配置运动轨迹
2. 设置运行参数
3. 启动自动运行
4. 监控运行状态

## 📊 测试数据

项目包含测试数据文件：
- `vofa+.csv` - VOFA+采集的运行数据
- `解算测试数据.csv` - 运动学解算验证数据

## 🐛 常见问题

### Q: 主从控制器无法通信？
A: 检查：
1. CAN总线连接是否正确
2. CAN波特率是否匹配（1Mbps）
3. 终端电阻是否安装（120Ω）
4. CAN_H和CAN_L是否正确连接

### Q: 运动学解算不准确？
A: 检查：
1. DH参数是否正确配置
2. 电机零位是否校准
3. 机械结构参数是否准确
4. 编码器反馈是否正常

### Q: VOFA+无法显示数据？
A: 检查：
1. 串口连接是否正常
2. 波特率是否匹配（115200）
3. 数据格式是否正确（FireWater/RawData）
4. 数据通道配置是否正确

### Q: 编译时找不到库文件？
A: 检查：
1. Keil MDK是否安装了Renesas芯片包
2. STM32CubeIDE是否安装了STM32F4支持包
3. FSP版本是否匹配
4. 工程路径是否正确

## 📚 相关文档

- [Delta运动学解算](modules/dummy_kinematic/)
- [CAN通信协议](can_message_format.md)
- [源码注释](Dummy%20源码注释/)
- [VOFA+使用指南](https://www.vofa.plus/)

## 🤝 贡献指南

欢迎提交Issue和Pull Request！

1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👥 作者

- **tonggeshuaiqiwudizuijunlang** - [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang)

## 🙏 致谢

- [Renesas Electronics](https://www.renesas.com/) - 主控制器MCU和开发工具
- [STMicroelectronics](https://www.st.com/) - 从控制器MCU和HAL库
- [FreeRTOS](https://www.freertos.org/) - 实时操作系统
- [ARM CMSIS-DSP](https://developer.arm.com/tools-and-software/embedded/cmsis) - 数字信号处理库
- [VOFA+](https://www.vofa.plus/) - 优秀的数据可视化工具

---

**⭐ 如果这个项目对你有帮助，请给个Star支持一下！**
