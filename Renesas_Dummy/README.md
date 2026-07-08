# 🤖 Dummy 6-DOF Robotic Arm

基于 Renesas RA6M5 MCU 的六轴机械臂控制系统，支持正逆运动学解算、多种控制模式和实时调试。

## ✨ 主要特性

### 核心功能
- **6自由度运动控制** - 支持6个关节的独立控制和协调运动
- **正运动学 (FK)** - 根据关节角度计算末端位置
- **逆运动学 (IK)** - 根据目标位置计算关节角度（支持权重优化）
- **夹爪控制** - 支持夹爪校准和精确控制

### 控制模式
- **蓝牙遥控** - 通过蓝牙模块进行无线控制
- **Flysky遥控器** - 支持FS-i6X遥控器接入（18通道）
- **串口调试** - UART串口调试和数据监控
- **CAN总线通信** - 支持CAN设备通信

### 高级功能
- **工作空间限制** - X/Y: ±450mm, Z: -120~650mm
- **姿态控制** - 末端执行器姿态精确控制
- **非阻塞反馈** - 实时状态反馈机制
- **多任务调度** - 基于FreeRTOS的多任务架构

## 🛠️ 硬件要求

### 主控芯片
- **MCU**: Renesas RA6M5 (R7FA6M5BH)
- **内核**: ARM Cortex-M33
- **主频**: 200MHz
- **Flash**: 2MB
- **RAM**: 512KB

### 外设接口
- **UART** - 蓝牙模块、串口调试
- **CAN** - CAN总线设备
- **GPIO** - 步进电机控制、限位开关
- **USB** - 虚拟串口（可选）

### 电机驱动
- **步进电机** - 6轴关节电机
- **夹爪电机** - 夹爪开合控制

## 📁 项目结构

```
Renesas_Dummy/
├── application/              # 应用层代码
│   ├── dummy_cmd.c/h        # 命令解析和控制逻辑
│   ├── dummy_motormatic.c/h # 电机运动控制
│   └── dummy_workspace_notes.md # 工作空间说明文档
├── bsp/                      # 板级支持包
│   ├── uart/                # UART驱动
│   ├── can/                 # CAN驱动
│   ├── gpio/                # GPIO驱动
│   └── dwt/                 # 延时函数
├── modules/                  # 功能模块
│   ├── bluetooth/           # 蓝牙通信模块
│   ├── remote/              # 遥控器模块(Flysky/MC6C)
│   ├── vision/              # 视觉识别模块
│   ├── step_motor/          # 步进电机驱动
│   ├── can_comm/            # CAN通信协议
│   ├── controller/          # 控制器抽象层
│   ├── daemon/              # 守护任务
│   ├── message_center/      # 消息中心
│   └── dummy_kinematic/     # 运动学解算
├── src/                      # 主程序入口
│   ├── hal_entry.c          # HAL入口
│   └── new_thread0_entry.c  # 主线程入口
├── ra/                       # Renesas FSP库
├── ra_gen/                   # 自动生成的配置代码
└── Dummy 源码注释/           # 源码注释和文档
```

## 🚀 快速开始

### 1. 开发环境

**必需工具：**
- [Keil MDK](https://www.keil.com/) v5.38+
- [Renesas FSP](https://www.renesas.com/us/en/software-tool/flexible-software-package-fsp) v4.0+
- J-Link调试器

**可选工具：**
- [VS Code](https://code.visualstudio.com/) + Keil Assistant插件
- 串口调试工具（如SSCOM、PuTTY）

### 2. 编译步骤

1. **克隆仓库**
   ```bash
   git clone https://github.com/tonggeshuaiqiwudizuijunlang/Renesas_Dummy.git
   cd Renesas_Dummy
   ```

2. **打开工程**
   - 使用Keil MDK打开 `FSP_Project.uvprojx`
   - 或者在VS Code中打开文件夹

3. **配置芯片包**
   - 确保已安装Renesas RA系列芯片支持包
   - 检查FSP版本是否匹配

4. **编译**
   - 点击Keil的"Build"按钮（F7）
   - 或使用命令行：
     ```bash
     uv4 -b FSP_Project.uvprojx -o build.log
     ```

### 3. 烧录和调试

1. **连接J-Link**
   - 将J-Link连接到开发板的SWD接口
   - 确保J-Link驱动已安装

2. **烧录程序**
   - 在Keil中点击"Download"按钮（F8）
   - 或使用J-Flash工具

3. **调试**
   - 点击"Debug"按钮（Ctrl+F5）
   - 设置断点，单步调试

## ⚙️ 配置说明

### 编译宏定义

在 `dummy_cmd.h` 中可以配置以下宏：

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `DUMMY_CMD_UART_MODE` | `BLUETOOTH` | 串口模式（蓝牙/调试） |
| `DUMMY_CMD_FKIK_TEST_ENABLE` | `0` | 正逆解测试模式 |
| `DUMMY_CMD_REMOTE_POSE_TEST_ENABLE` | `0` | 遥控器姿态测试 |
| `DUMMY_CMD_NONBLOCKING_FINISH_FEEDBACK_ENABLE` | `1` | 非阻塞完成反馈 |
| `DUMMY_GRIPPER_CALIB_ENABLE` | `1` | 夹爪校准功能 |

### 工作空间参数

```c
// 工作空间范围（单位：mm）
X: -450 ~ +450
Y: -450 ~ +450
Z: -120 ~ +650

// 姿态角度范围（单位：度）
Roll/Pitch/Yaw: -180 ~ +180
```

## 📡 通信协议

### 蓝牙通信
- **波特率**: 115200
- **数据格式**: 8N1
- **协议**: 自定义二进制协议

### 串口调试
- **波特率**: 115200
- **输出内容**: 关节角度、末端位置、状态信息

### CAN通信
- **波特率**: 500kbps / 1Mbps
- **协议**: CAN 2.0B

## 🎮 使用说明

### 蓝牙控制模式
1. 确保蓝牙模块已配对连接
2. 使用手机APP或上位机发送控制指令
3. 机械臂将执行相应动作

### 遥控器控制模式
1. 连接Flysky FS-i6X遥控器
2. 配置通道映射
3. 使用摇杆控制各关节运动

### 正逆解测试
1. 启用 `DUMMY_CMD_FKIK_TEST_ENABLE` 宏
2. 重新编译并烧录
3. 通过串口查看解算结果

## 📊 测试数据

项目包含测试数据文件：
- `18通道测试数据.csv` - 遥控器通道数据
- `正逆解测试数据.csv` - 运动学解算验证数据

## 🐛 常见问题

### Q: 编译时提示找不到FSP库？
A: 确保已正确安装Renesas FSP，并在Keil中配置了正确的FSP路径。

### Q: J-Link无法连接？
A: 检查：
1. J-Link驱动是否正确安装
2. SWD接口连接是否正确
3. 目标芯片是否供电正常

### Q: 蓝牙无法连接？
A: 检查：
1. 蓝牙模块是否正确连接到UART
2. 波特率是否匹配（115200）
3. 蓝牙模块是否已配对

### Q: 运动学解算不准确？
A: 检查：
1. DH参数是否正确配置
2. 电机零位是否校准
3. 工作空间限制是否合理

## 📚 相关文档

- [工作空间说明](application/dummy_workspace_notes.md)
- [CAN通信协议](can_message_format.md)
- [源码注释](Dummy%20源码注释/)

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

- [Renesas Electronics](https://www.renesas.com/) - 提供优秀的MCU和开发工具
- [FreeRTOS](https://www.freertos.org/) - 实时操作系统
- [ARM](https://www.arm.com/) - Cortex-M33内核

---

**⭐ 如果这个项目对你有帮助，请给个Star支持一下！**
