# 🤖 Dummy MoveIt Workspace

基于ROS2 Humble + MoveIt2的6自由度机械臂控制系统，支持运动规划、视觉识别、WebRTC视频流和实时控制。

## ✨ 主要特性

### 核心功能
- **6-DOF运动控制** - 支持6个关节的独立控制和协调运动
- **MoveIt运动规划** - 基于OMPL的路径规划和碰撞检测
- **正逆运动学解算** - KDL解析器支持精确运动学计算
- **实时碰撞检测** - 可配置的碰撞检测频率（默认10Hz）

### ROS2集成
- **URDF/Xacro描述** - 完整的机器人模型定义
- **Gazebo仿真** - 物理仿真环境
- **RViz可视化** - 3D可视化调试
- **Launch启动** - 一键启动多个节点

### 视觉与控制
- **视觉识别** - 基于OpenCV的物体检测
- **手眼标定** - 相机与机械臂坐标系标定
- **WebRTC视频流** - 低延迟视频传输（SRS服务器）
- **USB串口通信** - 与嵌入式控制器通信

### 高级功能
- **抓取规划** - 自动生成抓取姿态
- **避障运动** - 实时避障路径规划
- **多场景支持** - 可配置的仿真和实物环境
- **Fusion360集成** - 支持从Fusion360导出URDF

## 🛠️ 环境要求

### 软件依赖
- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **MoveIt2**: Humble版本
- **Gazebo**: Fortress (Ignition Gazebo)
- **Python**: 3.10+
- **OpenCV**: 4.x

### ROS2包依赖
```bash
sudo apt install \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2
```

### Python依赖
```bash
pip install \
  numpy \
  opencv-python \
  pyserial \
  scipy
```

## 📁 项目结构

```
dummy_moveit_ws/
├── dummy-ros2_description/          # 机器人URDF描述
│   ├── urdf/                        # URDF/Xacro文件
│   │   ├── dummy-ros2.xacro        # 主机器人描述
│   │   └── materials.xacro         # 材质定义
│   ├── launch/                      # 启动文件
│   │   ├── display.launch.py       # RViz显示
│   │   └── gazebo.launch.py        # Gazebo仿真
│   ├── meshes/                      # 3D模型文件
│   └── setup.py                     # 包配置
├── dummy_moveit_config/             # MoveIt配置
│   ├── config/                      # 配置文件
│   │   ├── dummy-ros2.urdf.xacro   # URDF配置
│   │   └── dummy-ros2.ros2_control.xacro  # 控制器配置
│   ├── launch/                      # MoveIt启动文件
│   │   ├── demo.launch.py          # 演示模式
│   │   ├── demo_custom.launch.py   # 自定义演示
│   │   ├── demo_with_usb.launch.py # USB连接模式
│   │   └── move_group.launch.py    # MoveGroup节点
│   └── setup.py                     # 包配置
├── dummy_controller/                # 控制器模块
│   ├── dummy_controller/            # 控制器代码
│   │   ├── dummy_arm_controller.py # 机械臂控制器
│   │   ├── moveit_server.py        # MoveIt服务器
│   │   ├── add_collision_object.py # 碰撞物体添加
│   │   └── dummy_cli_tool/         # 命令行工具
│   └── setup.py                     # 包配置
├── dummy_vision/                    # 视觉识别模块
│   ├── dummy_vision/                # 视觉代码
│   └── setup.py                     # 包配置
├── dummy_server/                    # 服务器通信模块
│   ├── dummy_server/                # 服务器代码
│   ├── examples/                    # 示例代码
│   └── setup.py                     # 包配置
├── srs_deploy/                      # SRS流媒体服务器
│   ├── docker-compose.yml           # Docker容器编排
│   ├── srs.conf                     # SRS配置
│   └── setup_brow_and_sysctl.sh    # 网络优化脚本
├── generate_xacro.py                # Xacro生成工具
├── test_usb_sender.py               # USB通信测试
├── LICENSE                          # MIT许可证
└── README.md                        # 本文档
```

## 🚀 快速开始

### 1. 安装ROS2和MoveIt2

```bash
# 安装ROS2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# 安装MoveIt2
sudo apt install ros-humble-moveit

# 安装其他依赖
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# 创建工作空间
mkdir -p ~/dummy_moveit_ws/src
cd ~/dummy_moveit_ws/src

# 克隆仓库
git clone https://github.com/your_username/dummy_moveit_ws.git .

# 安装Python依赖
pip3 install -r requirements.txt
```

### 2. 编译工作空间

```bash
cd ~/dummy_moveit_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 运行演示

#### RViz可视化
```bash
# 显示机器人模型
ros2 launch dummy-ros2_description display.launch.py

# MoveIt演示
ros2 launch dummy_moveit_config demo.launch.py
```

#### Gazebo仿真
```bash
# 启动Gazebo仿真
ros2 launch dummy-ros2_description gazebo.launch.py

# 启动MoveIt控制
ros2 launch dummy_moveit_config demo_custom.launch.py
```

#### USB连接实物
```bash
# 连接真实机械臂
ros2 launch dummy_moveit_config demo_with_usb.launch.py
```

### 4. SRS流媒体服务器（可选）

```bash
cd srs_deploy

# 设置环境变量（替换为你的服务器IP）
export SRS_CANDIDATE_IP=your_server_ip

# 启动SRS服务器
docker-compose up -d

# 访问Web界面
# http://your_server_ip:8080
```

## 🎮 使用说明

### 基本操作

#### 1. 运动规划
```python
# 使用MoveIt Python API
from moveit.planning import MoveItPy

# 初始化
moveit = MoveItPy(node_name="moveit_py")

# 获取规划组
arm_group = moveit.get_planning_component("dummy_arm")

# 设置目标姿态
arm_group.set_pose_target(target_pose)

# 执行规划
plan_result = arm_group.plan()

# 执行运动
if plan_result:
    arm_group.execute(plan_result)
```

#### 2. 添加碰撞物体
```python
# 添加桌面
ros2 run dummy_controller add_collision_object --ros-args \
  -p object_name:=table \
  -p position:=[0.5, 0.0, 0.0] \
  -p size:=[0.8, 0.6, 0.05]
```

#### 3. 视觉识别
```python
# 启动视觉节点
ros2 run dummy_vision vision_node --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p detection_topic:=/detected_objects
```

### 命令行工具

```bash
# 查看机器人状态
ros2 topic echo /joint_states

# 手动控制关节
ros2 run dummy_controller dummy_cli_tool

# 测试USB通信
python3 test_usb_sender.py /dev/ttyUSB0
```

## ⚙️ 配置说明

### MoveIt配置

在 `dummy_moveit_config/config/` 中修改：

**运动学配置 (`dummy-ros2.urdf.xacro`)：**
```xml
<!-- 关节限位 -->
<limit lower="-3.14" upper="3.14" velocity="2.0" effort="100"/>

<!-- 运动学求解器 -->
<kinematics_solver>kdl_kinematics_plugin/KDLKinematicsPlugin</kinematics_solver>
```

**碰撞检测 (`dummy_simulated_config.yaml`)：**
```yaml
collision_check_rate: 10.0  # Hz
max_collision_distance: 0.01  # 米
```

### 控制器配置

在 `dummy_controller/` 中修改：

**串口配置：**
```python
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 1.0
```

**运动参数：**
```python
MAX_VELOCITY = 2.0  # rad/s
MAX_ACCELERATION = 1.0  # rad/s²
PLANNING_TIME = 5.0  # 秒
```

## 📡 通信协议

### ROS2话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/robot_description` | `std_msgs/String` | 机器人URDF |
| `/move_group/display_planned_path` | `moveit_msgs/DisplayTrajectory` | 规划路径 |
| `/detected_objects` | `vision_msgs/Detection3DArray` | 检测到的物体 |

### ROS2服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/move_group/plan` | `moveit_msgs/GetMotionPlan` | 运动规划 |
| `/move_group/execute` | `moveit_msgs/ExecuteTrajectory` | 执行轨迹 |
| `/check_state_validity` | `moveit_msgs/GetStateValidity` | 状态有效性检查 |

### USB串口协议

```python
# 命令格式
[header][command][data_length][data...][checksum]

# 示例：移动关节
[0xAA][0x01][0x06][joint1, joint2, ..., joint6][0x55]
```

## 🐛 常见问题

### Q: MoveIt规划失败？
A: 检查：
1. 关节限位是否正确配置
2. 碰撞物体是否合理设置
3. 规划时间是否足够（默认5秒）
4. 目标姿态是否在工作空间内

### Q: Gazebo仿真卡顿？
A: 尝试：
1. 降低渲染质量
2. 减少碰撞检测频率
3. 使用简化模型
4. 关闭不必要的可视化

### Q: USB串口连接失败？
A: 检查：
1. 串口权限：`sudo chmod 666 /dev/ttyUSB0`
2. 波特率是否匹配（115200）
3. 串口设备是否正确
4. 是否被其他程序占用

### Q: 视觉识别不准确？
A: 调整：
1. 相机标定参数
2. 光照条件
3. 检测阈值
4. 手眼标定精度

## 📚 相关文档

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [MoveIt2教程](https://moveit.picknik.ai/humble/doc/tutorials/tutorials_in_ide/tutorials_in_ide.html)
- [Gazebo教程](https://gazebosim.org/api)
- [URDF教程](https://wiki.ros.org/urdf/Tutorials)

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

- [ROS2社区](https://ros.org/) - 提供优秀的机器人框架
- [MoveIt](https://moveit.ros.org/) - 运动规划框架
- [Gazebo](https://gazebosim.org/) - 物理仿真环境
- [SRS](https://github.com/ossrs/srs) - 流媒体服务器
- [Fusion360](https://www.autodesk.com/products/fusion-360) - 3D CAD设计

---

**⭐ 如果这个项目对你有帮助，请给个Star支持一下！**
