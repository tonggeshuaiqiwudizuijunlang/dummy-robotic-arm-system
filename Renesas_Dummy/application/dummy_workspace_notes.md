# Dummy 机械臂工作空间与末端控制注意事项

本文档记录当前 Dummy 机械臂在本工程中的运动学参数、建议工作空间、关节限位、坐标约定，以及使用遥控器控制末端位置和姿态时需要注意的事项。

## 1. 当前运动学模型参数

参数来自 `application/dummy_motormatic.c` 的 `Dummy_Motormatic_Init()`。

| 参数 | 数值 | 单位 | 说明 |
|---|---:|---|---|
| `L_BASE` | 109.0 | mm | 基座高度相关长度 |
| `D_BASE` | 35.0 | mm | 基座水平偏移 |
| `L_ARM` | 146.0 | mm | 大臂长度 |
| `L_FOREARM` | 115.0 | mm | 小臂长度 |
| `D_ELBOW` | 52.0 | mm | 肘部结构偏移 |
| `L_WRIST` | 72.0 | mm | 腕部到末端长度 |

有效小臂长度近似为：

```text
l_ew = sqrt(L_FOREARM^2 + D_ELBOW^2)
     = sqrt(115^2 + 52^2)
     ≈ 126.2 mm
```

理论最大展开距离约为：

```text
L_ARM + l_ew + L_WRIST ≈ 146 + 126.2 + 72 ≈ 344.2 mm
```

实际工作空间会受到 `D_BASE`、姿态、关节限位、腕部方向影响，不是完整球体或长方体。

## 2. 当前关节软限位

| 关节 | 最小值 | 最大值 | 单位 |
|---|---:|---:|---|
| J1 | -175 | 175 | deg |
| J2 | -25 | 150 | deg |
| J3 | 0 | 180 | deg |
| J4 | -180 | 180 | deg |
| J5 | -120 | 120 | deg |
| J6 | -360 | 360 | deg |

注意：

- IK 求解后会通过 `Kinematic_Select_Best_Sol()` 按这些限位过滤。
- 如果目标 XYZ 在几何范围内但 IK 无解，常见原因是姿态要求过强或关节限位挡住了。
- J2/J3 最容易因为机械零点、方向、偏移出问题，需要重点确认反馈角度是否与运动学坐标一致。

## 3. 建议测试工作空间

遥控器末端位姿测试目前使用保守工作空间：

| 坐标或姿态 | 建议最小值 | 建议最大值 | 单位 |
|---|---:|---:|---|
| X | -250 | 300 | mm |
| Y | -300 | 300 | mm |
| Z | -50 | 450 | mm |
| A / Roll | -180 | 180 | deg |
| B / Pitch | -180 | 180 | deg |
| C / Yaw | -180 | 180 | deg |

注意：

- 这是测试用安全包围盒，不代表盒子内每一个点都一定可达。
- 同一个 XYZ，姿态不同，IK 可能一个有解一个无解。
- 接近边界时，IK 可能突然切换到另一组分支解，应降低速度并观察关节跳变。

## 4. 坐标和姿态约定

当前 `Pose6D_t` 中：

```text
X/Y/Z：末端位置，单位 mm
A/B/C：末端姿态，单位 deg
```

当前姿态约定为：

| 字段 | 含义 |
|---|---|
| A | Roll，绕 X 轴 |
| B | Pitch，绕 Y 轴 |
| C | Yaw，绕 Z 轴 |

旋转矩阵使用 ZYX 欧拉角关系：

```text
R = Rz(C) * Ry(B) * Rx(A)
```

欧拉角在接近奇异姿态时可能出现 `180` 和 `-180` 的跳变。判断姿态是否连续时，不要只看 A/B/C 数字，也要结合末端实际运动或旋转矩阵。

## 5. 遥控器末端位姿测试模式

默认仍然是关节角控制。若要打开末端位姿测试，在 `application/dummy_cmd.h` 中设置：

```c
#define DUMMY_CMD_REMOTE_POSE_TEST_ENABLE 1
```

若要恢复默认关节角控制：

```c
#define DUMMY_CMD_REMOTE_POSE_TEST_ENABLE 0
```

测试模式只替换 `L2_UP` 正常遥控器控制分支；急停、回零、7 字形点、PC/视觉控制优先级不变。

## 6. 遥控器操作方式

进入末端控制测试需要满足：

```text
R2 不是 DOWN
R1 是 UP
L2 是 UP
```

### L1_DOWN：控制末端位置 XYZ

| 摇杆 | 控制量 |
|---|---|
| 右摇杆上下 `rocker_r1` | X |
| 右摇杆左右 `rocker_r_` | Y |
| 左摇杆上下 `rocker_l1` | Z |

### L1_MID 或 L1_UP：控制末端姿态 ABC

| 摇杆 | 控制量 |
|---|---|
| 左摇杆上下 `rocker_l1` | A / Roll |
| 左摇杆左右 `rocker_l_` | B / Pitch |
| 右摇杆左右 `rocker_r_` | C / Yaw |

## 7. 控制链路

末端位姿控制不是直接下发 XYZ，而是在 `dummy_cmd.c` 中先做 IK，再复用原来的关节角控制链路：

```text
遥控器摇杆
  -> remote_target_pose.X/Y/Z/A/B/C
  -> Kinematic_SolveIK()
  -> Kinematic_Select_Best_Sol()
  -> dummy_cmd_send.joint1_angle ... joint6_angle
  -> ARM_PC_MODE
  -> Dummy_Motormatic_Task()
  -> Joint_Motor_Set_Pos_With_SpeedLimit()
```

这样下游电机控制仍然只接收 6 个关节目标。

## 8. 上板测试建议

1. 先打开 Serial_Debug，观察 12 通道：
   - CH0~CH5：当前 6 轴角度。
   - CH6~CH8：当前 FK 的 X/Y/Z。
   - CH9~CH11：当前 FK 的 A/B/C。
2. 手动或低速移动关节，确认 FK 位姿变化方向符合直觉。
3. 打开 `DUMMY_CMD_REMOTE_POSE_TEST_ENABLE`。
4. 先小幅拨动摇杆，观察 IK 输出的关节目标是否连续。
5. 确认没有大跳变后，再让电机低速跟随。
6. 若某个方向与预期相反，优先调整该摇杆输入符号。
7. 若 IK 经常无解，优先减小 `REMOTE_POSE_POS_GAIN` / `REMOTE_POSE_ATT_GAIN` 或缩小工作空间。

## 9. 重要注意事项

- 末端控制依赖 `dummy_fetch_data.current_joints_feedback` 与运动学关节角坐标系一致。
- 如果 J2/J3 反馈角度存在 `75/90` 偏移或正负号不一致，FK/IK 数学正确也会导致实机运动方向不对。
- `Kinematic_SolveIK()` 使用 Euler 姿态输入时，必须保证 `Pose6D_t.hasR = false`，否则会使用旧的 `R[9]`。
- 当前没有做笛卡尔轨迹规划，只是每个周期重新算 IK 后下发关节目标，因此目标变化过快会导致运动不平滑。
- 建议测试初期降低 `Dummy_Motormatic_Task()` 中 `ARM_PC_MODE` 的速度限制。
- 在机械臂接近展开、收缩极限或腕部奇异姿态时，IK 分支可能切换，表现为某些关节突然跳动。
