# 🤖 Dummy Robotic Arm System

基于瑞萨RA6M5微控制器的力反馈远程操作机械臂系统，支持六自由度位姿输入、主从同步控制及多模式操作。

## 📖 项目简介

针对远程操作机器人在人机交互过程中控制精度不足、操作反馈不直观等问题，本系统设计并实现了一种基于瑞萨RA6M5微控制器的力反馈远程操作控制系统。

系统采用自主设计的六自由度控制器作为人机交互终端，通过三个电机分别完成X、Y、Z三个方向的位置输入，并在控制器末端集成BMI088 IMU，实时获取末端姿态信息，实现六自由度位姿输入。执行端驱动六轴机械臂及一自由度夹爪完成相应动作，同时根据执行端状态向主控制器回传反馈信息，实现稳定的人机交互。

## ✨ 主要特性

### 🎯 六自由度位姿输入

- 三轴独立运动：X/Y轴采用DJI M6020电机，Z轴采用RS05电机
- 末端姿态采集：集成DM科技BMI088 IMU，实时输出姿态角
- CAN总线通信：所有设备统一采用CAN总线，实时性高、抗干扰强

### 🕹️ 主从实时控制

- 双RA6M5分布式架构：主控制器+执行端控制器
- 有线串口通信：保证实时性和稳定性
- 运动学解算：支持正逆运动学计算

### 💪 力反馈交互

- 实时力感知：根据执行端反馈调整控制器输出
- 直观操作体验：操作者可感知执行端受力变化
- 提高操控精度：增强远程操作的安全性

### 🖥️ 多模式控制

- **上位机控制**：ROS2 MoveIt + RViz运动规划
- **自定义控制器**：手握式从臂遥操作
- **遥控器控制**：Flysky FS-i6X无线操控

### 🏗️ 模块化软件架构

- BSP层：板级支持包，硬件抽象
- Modules层：设备管理、通信协议
- Application层：业务逻辑、运动控制
- 基于FreeRTOS多任务调度

## 📦 系统组成

| 组件                    | 说明                         | 仓库                                                         |
| ----------------------- | ---------------------------- | ------------------------------------------------------------ |
| **Renesas_Dummy**       | 执行端固件（6轴机械臂+夹爪） | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Renesas_Dummy) |
| **Customer_Controller** | 主从控制器系统（双RA6M5）    | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Customer_Controller) |
| **dummy_moveit_ws**     | ROS2 MoveIt工作空间          | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/dummy_moveit_ws) |

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      控制层（三选一）                          │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   上位机     │  │ 自定义控制器 │  │   遥控器     │      │
│  │ ROS2+RViz   │  │  手握式从臂  │  │  Flysky      │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
                         有线串口/无线
                              │
┌─────────────────────────────────────────────────────────────┐
│                    主控制器（RA6M5）                          │
│              (Customer_Controller)                           │
│    ┌─────────────────────────────────────────────────┐      │
│    │  位姿采集 │ 运动学解算 │ 力反馈处理 │ 通信管理  │      │
│    └─────────────────────────────────────────────────┘      │
│         │ CAN总线              │ 有线串口                    │
└─────────┼──────────────────────┼────────────────────────────┘
          │                      │
┌─────────┴─────────┐  ┌─────────┴─────────┐
│  六自由度控制器    │  │   执行端控制器    │
│  ┌─────┬─────┐    │  │    (RA6M5)        │
│  │X轴  │Y轴  │Z轴 │  │  ┌─────────────┐  │
│  │M6020│M6020│RS05│  │  │ 6轴机械臂   │  │
│  └─────┴─────┘    │  │  │ + 夹爪      │  │
│       + BMI088    │  │  └─────────────┘  │
└───────────────────┘  └───────────────────┘
```

## 🛠️ 硬件规格

### 主控制器

- **MCU**: Renesas RA6M5 (R7FA6M5BH)
- **内核**: ARM Cortex-M33, 200MHz
- **Flash**: 2MB, **RAM**: 512KB
- **通信**: CAN, UART, SPI, I2C

### 六自由度控制器

| 轴   | 电机型号   | 驱动方式 | 功能           |
| ---- | ---------- | -------- | -------------- |
| X轴  | DJI M6020  | CAN总线  | 水平前后       |
| Y轴  | DJI M6020  | CAN总线  | 水平左右       |
| Z轴  | RS05       | CAN总线  | 垂直上下       |
| 姿态 | BMI088 IMU | CAN总线  | Roll/Pitch/Yaw |

### 执行端

- **机械臂**: 6轴步进电机驱动
- **夹爪**: 1自由度
- **反馈**: 编码器位置反馈

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/tonggeshuaiqiwudizuijunlang/dummy-robotic-arm-system.git
cd dummy-robotic-arm-system
```

### 2. 编译固件

**主控制器（Customer_Controller）：**

```bash
cd Customer_Controller
# 使用Keil MDK打开 FSP_Project.uvprojx
```

**执行端（Renesas_Dummy）：**

```bash
cd Renesas_Dummy
# 使用Keil MDK打开 FSP_Project.uvprojx
```

### 3. 运行ROS2上位机

```bash
cd dummy_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dummy_moveit_config demo.launch.py
```

## 📚 文档

- [Renesas_Dummy文档](Renesas_Dummy/README.md) - 执行端固件说明
- [Customer_Controller文档](Customer_Controller/README.md) - 主从控制器说明
- [dummy_moveit_ws文档](dummy_moveit_ws/README.md) - ROS2工作空间说明

## 🎯 应用场景

- **医疗辅助**: 远程药品取放、器械递送、样本转运
- **危险环境**: 高温、高压、辐射、有毒有害环境作业
- **机器人教学**: 控制算法验证、嵌入式开发实验平台
- **人机交互研究**: 遥操作、力反馈控制研究

## 📄 许可证

本项目采用 MIT 许可证 - 详见各子项目的LICENSE文件


## 🙏 致谢

- [瑞萨电子](https://www.renesas.com/) - 提供RA6M5微控制器及开发工具
- [DJI](https://www.dji.com/) - M6020电机驱动
- [DM科技](https://www.dji.com/) - BMI088 IMU模块
- [ROS社区](https://www.ros.org/) - ROS2及MoveIt框架

---

**⭐ 如果这个项目对你有帮助，请给个Star支持一下！**
