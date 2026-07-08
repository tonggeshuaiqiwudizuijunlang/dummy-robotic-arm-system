# 🤖 Dummy Robotic Arm System

一个完整的6自由度机械臂控制系统，包含嵌入式固件、双MCU控制器和ROS2运动规划。

## 📦 项目组成

| 项目 | 说明 | 独立仓库 |
|------|------|----------|
| **Renesas_Dummy** | 6轴机械臂主控固件（Renesas RA6M5） | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Renesas_Dummy) |
| **Customer_Controller** | 双MCU控制器系统（Renesas + STM32） | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/Customer_Controller) |
| **dummy_moveit_ws** | ROS2 MoveIt运动规划工作空间 | [GitHub](https://github.com/tonggeshuaiqiwudizuijunlang/dummy_moveit_ws) |

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 MoveIt 工作空间                       │
│                  (dummy_moveit_ws)                           │
│    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│    │ 运动规划    │  │ 视觉识别    │  │ 碰撞检测    │       │
│    │ MoveIt      │  │ OpenCV      │  │ Gazebo      │       │
│    └─────────────┘  └─────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
                              │
                         USB串口/CAN
                              │
┌─────────────────────────────────────────────────────────────┐
│                  双MCU控制器系统                              │
│               (Customer_Controller)                          │
│    ┌─────────────────────┐  ┌─────────────────────┐        │
│    │   Renesas RA6M5     │  │    STM32F407        │        │
│    │   主控制器          │  │    Delta控制器      │        │
│    └─────────────────────┘  └─────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
                              │
                            CAN总线
                              │
┌─────────────────────────────────────────────────────────────┐
│                  6轴机械臂硬件控制                            │
│                 (Renesas_Dummy)                              │
│    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│    │ 步进电机    │  │ 编码器      │  │ 传感器      │       │
│    │ 驱动控制    │  │ 位置反馈    │  │ 状态监测    │       │
│    └─────────────┘  └─────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

## ✨ 主要特性

- 🎯 **6-DOF运动控制** - 支持6个关节的独立控制和协调运动
- 🧠 **MoveIt运动规划** - 基于OMPL的路径规划和碰撞检测
- 👁️ **视觉识别** - 基于OpenCV的物体检测和抓取
- 📹 **WebRTC视频流** - 低延迟视频传输（SRS服务器）
- 🔌 **多通信接口** - UART、CAN、USB串口支持
- 🎮 **多种控制模式** - 蓝牙遥控、ROS2控制、自动规划

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
