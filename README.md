# 🤖 Dummy Robotic Arm System

一个完整的6自由度机械臂控制系统，支持ROS2软件控制和主从式遥操作。

## 📦 项目组成

| 项目 | 说明 | 独立仓库 |
|------|------|----------|
| **Renesas_Dummy** | 6轴机械臂主控固件（Renesas RA6M5） | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Renesas_Dummy) |
| **Customer_Controller** | 双MCU控制器系统（主臂+从臂） | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Customer_Controller) |
| **dummy_moveit_ws** | ROS2 MoveIt运动规划工作空间 | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/dummy_moveit_ws) |

## 🎮 控制方式

本系统支持三种控制方式：

### 方式一：上位机控制（ROS2 MoveIt）
通过ROS2上位机进行运动规划和控制：
- 使用RViz可视化界面拖拽末端执行器
- MoveIt自动规划运动路径和碰撞检测
- 支持视觉识别和自动抓取任务
- 适合精确的自动化任务

### 方式二：自定义控制器（主从遥操作）
使用自研的手握式从臂控制器进行遥操作：
- **从臂**：手握式小型机械臂控制器
- **主臂**：实际的6轴机械臂
- 人手操作从臂 → 姿态映射 → 主臂跟随运动
- 适合灵活的手动操控场景

### 方式三：遥控器控制（Flysky）
使用航模遥控器进行无线控制：
- **遥控器**：Flysky FS-i6X 18通道遥控器
- **接收器**：iA6B接收器
- 支持多通道独立控制各关节
- 适合远程无线操控

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    控制层（二选一）                            │
│                                                             │
│    ┌─────────────────────┐    ┌─────────────────────┐      │
│    │  ROS2 MoveIt + RViz │    │   手握式从臂控制器   │      │
│    │  软件运动规划       │    │   主从遥操作        │      │
│    └─────────────────────┘    └─────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
                              │
                         USB串口/CAN
                              │
┌─────────────────────────────────────────────────────────────┐
│                  双MCU控制器系统                              │
│               (Customer_Controller)                          │
│    ┌─────────────────────┐  ┌─────────────────────┐        │
│    │   Renesas RA6M5     │  │    STM32F407        │        │
│    │   主控制器          │  │    从控制器         │        │
│    └─────────────────────┘  └─────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
                              │
                            CAN总线
                              │
┌─────────────────────────────────────────────────────────────┐
│                  6轴机械臂（主臂）                            │
│                 (Renesas_Dummy)                              │
│    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│    │ 步进电机    │  │ 编码器      │  │ 传感器      │       │
│    │ 驱动控制    │  │ 位置反馈    │  │ 状态监测    │       │
│    └─────────────┘  └─────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

## ✨ 主要特性

- 🎯 **6-DOF运动控制** - 支持6个关节的独立控制和协调运动
- 🖥️ **ROS2 MoveIt控制** - RViz可视化运动规划，支持碰撞检测和避障
- 🕹️ **主从遥操作** - 手握式从臂直观控制主臂运动
- 👁️ **视觉识别** - 基于OpenCV的物体检测和抓取
- 📹 **WebRTC视频流** - 低延迟远程视频监控
- 🔌 **多通信接口** - UART、CAN、USB串口支持

## 🛠️ 硬件要求

### 主控制器
- **MCU**: Renesas RA6M5 (R7FA6M5BH)
- **内核**: ARM Cortex-M33, 200MHz

### 从控制器
- **MCU**: STM32F407IGHx
- **内核**: ARM Cortex-M4, 168MHz

### 机械臂
- **自由度**: 6轴 + 夹爪
- **驱动**: 步进电机
- **反馈**: 编码器

## 🚀 快速开始

### 1. 克隆仓库
```bash
git clone https://github.com/tonggeshuaiqiwudizuijunlang/dummy-robotic-arm-system.git
cd dummy-robotic-arm-system
```

### 2. 编译固件

**Renesas_Dummy:**
```bash
cd Renesas_Dummy
# 使用Keil MDK打开 FSP_Project.uvprojx
```

**Customer_Controller:**
```bash
cd Customer_Controller
# 使用Keil MDK打开 FSP_Project.uvprojx
# 或编译STM32部分：cd rm_gongcheng_base-Delta && make
```

### 3. 运行ROS2
```bash
cd dummy_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dummy_moveit_config demo.launch.py
```

## 📚 文档

- [Renesas_Dummy文档](Renesas_Dummy/README.md)
- [Customer_Controller文档](Customer_Controller/README.md)
- [dummy_moveit_ws文档](dummy_moveit_ws/README.md)

## 📄 许可证

本项目采用 MIT 许可证 - 详见各子项目的LICENSE文件

## 👥 作者

- **tonggeshuaiqiwudizuijunlang** - [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang)

---

**⭐ 如果这个项目对你有帮助，请给个Star支持一下！**
