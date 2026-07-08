# 当前正逆解算法梳理与优化建议

本文档记录当前 `dummy_kinematic_v2` 中控制器 FK、机械臂 FK、机械臂 IK、选解、角度映射和限位的计算过程，并列出后续可优化点。

## 1. 当前整体计算链路

当前系统中有两层运动学：

1. 控制器三电机 + IMU 到目标末端位姿：

```text
q1/q2/q3 控制器电机读数 + IMU 姿态
        ↓
Kinematic_ControllerFK()
        ↓
目标末端 Pose6D
```

2. 目标末端位姿到机械臂 6 轴电机目标角：

```text
目标 Pose6D
        ↓
Kinematic_SolveIK_To_Motor()
        ↓
机械臂 6 轴 IK 关节角
        ↓
joint_to_motor 映射
        ↓
电机角限位 + q2/q3 耦合限位
        ↓
6 轴机械臂电机目标角
```

在应用层 `dummy_motormatic.c` 中，`ARM_FREE_MODE` 的主流程是：

```text
读取 q1/q2/q3 控制器电机 + IMU
        ↓
Kinematic_ControllerFK()
        ↓
得到 target_pose
        ↓
Kinematic_SolveIK_To_Motor()
        ↓
得到 6 轴机械臂目标电机角 last_valid_joint_cmd
        ↓
发布 dummy_feed
```

---

## 2. 配置来源

DH、限位、映射参数目前由 `application/dummy_motormatic.c` 中的 `Dummy_Motormatic_Build_Kinematic_Config()` 显式配置，再通过 `Kinematic_Init()` 传入 `dummy_kinematic_v2`。

### 2.1 DH 参数

当前 baseline 使用 `rm_gongcheng_base-Delta` 原工程 DH：

```text
a     = {0, 0, 0.250, -0.04347, 0, 0}
alpha = {0, -pi/2, 0, -pi/2, pi/2, -pi/2}
d     = {0, 0, 0, 0.36349, 0, 0.06}
```

DH 长度单位是米，而 `Pose6D_t.X/Y/Z` 单位是毫米，所以配置中有：

```text
pose_to_dh_scale = 0.001
dh_to_pose_scale = 1000
```

即：

```text
Pose6D mm -> DH m: 乘 0.001
DH m -> Pose6D mm: 乘 1000
```

### 2.2 IK 初始参考角

当前初始参考角为：

```text
{0, 75, 105, 0, 0, 0} deg
```

`Kinematic_Init()` 会将其转换为弧度，存入：

```text
handle->current_ik_rad[]
handle->last_answer_rad[]
```

这两个数组对应原工程中的 `q_current` / `q_answer` 状态。

### 2.3 IK 角到电机角映射

当前配置等价于原工程 `IK_Angle_Translate()`：

```text
motor0 =  theta0
motor1 = -theta1 + 75
motor2 = -theta1 - theta2 + 180
motor3 =  theta3
motor4 =  theta4
motor5 =  theta5
```

反向映射用于后续 FK / 反馈闭环：

```text
theta0 = motor0
theta1 = 75 - motor1
theta2 = 105 + motor1 - motor2
theta3 = motor3
theta4 = motor4
theta5 = motor5
```

### 2.4 电机限位和耦合限位

当前配置等价于原工程 `GimbalLimit()`：

```text
motor1/q2: 0 ~ 140 deg
motor2/q3: -110 ~ 40 deg
motor4/q5: -130 ~ 130 deg
motor2 - motor1: -135 ~ 0 deg
```

`Kinematic_Limit_Motor_Angle()` 的执行顺序是：

```text
先限 motor1/q2
再限 motor2/q3
再限 motor2 - motor1 耦合
再限 motor4/q5
最后限其它启用的轴
```

---

## 3. 控制器 FK：三电机 + IMU 到目标 Pose

入口函数：

```c
Kinematic_ControllerFK()
```

输入结构：

```c
typedef struct
{
    float q1_rad;
    float q2_rad;
    float q3_rad;
    float imu_roll_deg;
    float imu_pitch_deg;
    float imu_yaw_deg;
} ControllerInput_t;
```

### 3.1 控制器电机角转换成 Delta 主动臂角

控制器三电机读数先被转换成 Delta 机构三主动臂角：

```text
theta0 = -q1_rad - q1_offset
theta1 =  q2_rad - q2_initial - q2_offset
theta2 = -q3_rad + q3_initial - q3_offset
```

这里 offset 和 initial 都来自 `ControllerKinematicConfig_t`。

### 3.2 Delta FK 求控制器末端 XYZ

Delta FK 使用三球交点法。

先计算三条支链球心：

```text
支链1：
A1 = R + L cos(theta1) - r
C1 = L sin(theta1)

支链2：
A2 = -0.5 * (R + L cos(theta2) - r)
B2 =  sqrt(3)/2 * (R + L cos(theta2) - r)
C2 =  L sin(theta2)

支链3：
A3 = -0.5 * (R + L cos(theta3) - r)
B3 = -sqrt(3)/2 * (R + L cos(theta3) - r)
C3 =  L sin(theta3)
```

其中：

```text
R  = delta_R_m
r  = delta_r_m
L  = delta_L_m
La = delta_La_m
```

再利用三球约束消元，将 `X` 和 `Y` 表示成 `Z` 的线性函数：

```text
X = E1 * Z + F1
Y = E2 * Z + F2
```

代入球面方程，得到：

```text
a Z² + b Z + c = 0
```

当前取：

```text
Z = (-b - sqrt(discriminant)) / (2a)
```

最后得到 Delta 末端位置：

```text
xyz_m[0] = X
xyz_m[1] = Y
xyz_m[2] = Z
```

### 3.3 重力补偿 torque

`Kinematic_ControllerGravityCompensation()` 在 Delta FK 后继续计算重力补偿：

1. 计算三条支链约束向量 `S1/S2/S3`。
2. 计算三条支链主动关节对末端位置的速度方向 `b1/b2/b3`。
3. 构造：

```text
S_T = [S1^T; S2^T; S3^T]
Sb  = diag(S1·b1, S2·b2, S3·b3)
Jacobi = -inv(S_T) * Sb
```

4. 设置反馈力方向：

```text
gravity = {0, -13, 0, 0}
```

5. 计算：

```text
torque = Jacobi^T * gravity
tor[i] = -torque[i]
```

输出的 `torque[0..2]` 由 `dummy_motormatic.c` 转成控制器三电机的 `SetRef()`。

### 3.4 Delta 坐标映射为机械臂目标位置

Delta FK 输出的 `xyz_m` 需要映射为机械臂目标末端位置：

```text
x = -(delta_z - xyz_delta_error_z)
y =  (delta_y - xyz_delta_error_y)
z =  (delta_x - xyz_delta_error_x)

x += xyz_arm_error_x
y += xyz_arm_error_y
z += xyz_arm_error_z

x *= scale_x
y *= scale_y
z *= scale_z

y *= -1
```

最后从米转成毫米，写入 `Pose6D_t.X/Y/Z`。

### 3.5 IMU 姿态映射为目标姿态

姿态计算为：

```text
roll  = imu_roll_deg + 180
pitch = -ScalePitch(imu_pitch_deg) + 90
yaw   = imu_yaw_deg
```

其中 pitch 会先限幅到：

```text
[-pitch_limit_deg, pitch_limit_deg]
```

再缩放到：

```text
[-90, 90]
```

然后构建旋转矩阵：

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

并写入：

```text
Pose.A = roll
Pose.B = pitch
Pose.C = yaw
Pose.R = R
Pose.hasR = true
```

---

## 4. 机械臂 FK：6 轴关节角到 Pose

入口函数：

```c
Kinematic_SolveFK()
```

输入：

```text
Joint6D_t inputJoints，单位 deg，语义为 IK/DH 关节角
```

### 4.1 角度转换

输入角度转换为弧度：

```text
joint.theta[i] = deg_to_rad(inputJoints->a[i])
```

### 4.2 标准 DH 矩阵连乘

每个关节使用标准 DH 矩阵：

```text
[ cosθ  -sinθ cosα   sinθ sinα   a cosθ ]
[ sinθ   cosθ cosα  -cosθ sinα   a sinθ ]
[ 0      sinα        cosα        d      ]
[ 0      0           0           1      ]
```

从 1 到 6 轴依次连乘：

```text
T = T1 * T2 * T3 * T4 * T5 * T6
```

### 4.3 提取位姿

位置：

```text
x = T[0][3]
y = T[1][3]
z = T[2][3]
```

姿态：

```text
R = T 的左上 3x3
```

输出 `Pose6D_t` 时：

```text
Pose.X/Y/Z = position * dh_to_pose_scale
Pose.A/B/C = roll/pitch/yaw
Pose.R = R
Pose.hasR = true
```

---

## 5. 机械臂 IK：Pose 到最多 8 组关节解

入口函数：

```c
Kinematic_SolveIK()
```

内部调用原工程逻辑：

```text
Inverse_Kinematics_From_Rotation_Matrix()
    ↓
Inverse_Kinematics_From_Pose()
```

### 5.1 Pose 转内部 position + rotation

位置单位转换：

```text
position.x = Pose.X * pose_to_dh_scale
position.y = Pose.Y * pose_to_dh_scale
position.z = Pose.Z * pose_to_dh_scale
```

姿态：

- 若 `Pose.hasR == true`，直接使用 `Pose.R`。
- 否则用 `Pose.A/B/C = roll/pitch/yaw` 生成旋转矩阵。

### 5.2 提取方向向量

从目标旋转矩阵 `R06` 提取：

```text
a = [ax, ay, az] = R06 第三列，即末端 Z 轴方向
n = [nx, ny, nz] = R06 第一列，即末端 X 轴方向
```

### 5.3 计算腕心

末端工具长度是：

```text
d6 = dh.d[5]
```

腕心：

```text
px = x - d6 * ax
py = y - d6 * ay
pz = z - d6 * az
```

### 5.4 解 q1 两组

```text
temp = px² + py² - d2²
```

若 `temp < 0`，无解。

否则：

```text
q1_1 = atan2(py, px) - atan2(d2,  sqrt(temp))
q1_2 = atan2(py, px) - atan2(d2, -sqrt(temp))
```

当前原工程 baseline 中 `d2 = 0`，所以近似为：

```text
q1_1 = atan2(py, px)
q1_2 = atan2(py, px) - pi
```

### 5.5 解 q3 两组

对每个 q1，计算：

```text
k = (px² + py² + pz² - a2² - a3² - d2² - d4²) / (2a2)
```

再计算：

```text
temp = a3² + d4² - k²
```

若 `temp < 0`，当前 q1 分支无解。

否则 q3 有两组：

```text
q3_1 = atan2(a3, d4) - atan2(k,  sqrt(temp))
q3_2 = atan2(a3, d4) - atan2(k, -sqrt(temp))
```

### 5.6 解 q2

先计算：

```text
q23 = atan2(
  -(a3 + a2 cos q3) pz
    + (cos q1 px + sin q1 py) (a2 sin q3 - d4),

  (a2 sin q3 - d4) pz
    + (cos q1 px + sin q1 py) (a2 cos q3 + a3)
)
```

然后：

```text
q2 = q23 - q3
```

再按原工程经验补偿：

```text
if q3 < 0:
    q2 += pi/2
else if q3 > 0:
    q2 -= 3pi/2
```

### 5.7 解 q4/q5/q6

有了 `q1` 和 `q23` 后，使用目标姿态的 `a` 轴和 `n` 轴解析腕部角。

q4：

```text
q4 = atan2(
  -ax sin q1 + ay cos q1,
  -ax cos q1 cos q23 - ay sin q1 cos q23 + az sin q23
)
```

q5：

```text
s5 = ...
c5 = ...
q5 = atan2(s5, c5)
```

q6：

```text
s6 = ...
c6 = ...
q6 = atan2(s6, c6)
```

### 5.8 生成最多 8 组解

解组合来源：

```text
2 个 q1
× 2 个 q3/q2
× 2 个 wrist flip
= 最多 8 组
```

普通腕部解：

```text
[q1, q2, q3, q4, q5, q6]
```

腕部翻转解：

```text
[q1, q2, q3, q4 + pi, -q5, q6 + pi]
```

### 5.9 q3 跨 180° patch

当前保留原工程的 q3 patch：

```text
q3_tmp = normalize(q3)

如果 q3_tmp > 160°:
    flag_beyond_180 = 1

如果 q3_tmp < 160° 且 q3_tmp > 0:
    flag_beyond_180 = 0

如果 flag_beyond_180 且 q3_tmp < 0:
    q3_tmp = 2pi + q3_tmp
```

目的：避免 q3 接近 180° 时跨到负角导致机械臂跳变。

---

## 6. 选解逻辑

入口函数：

```c
Kinematic_Select_Best_Sol()
```

### 6.1 单位转换

先将 `IKSolves_t` 中的 degree 转成内部 rad，再将当前参考角也转成 rad。

### 6.2 计算代价

对每组候选解计算：

```text
diff = candidate[j] - reference[j]
```

角度差会 wrap 到 `[-pi, pi]`。

然后加权：

```text
cost += weight[j] * abs(diff)
```

当前权重配置为：

```text
{7, 6, 5, 4, 3, 2}
```

即前面的关节权重更大。

### 6.3 丢弃异常解

如果某个候选角度大于 `300°`，认为无效。

如果某个关节跳变超过 `max_single_joint_jump_rad`，当前为 `6 rad`，认为无效。

### 6.4 cost 过大返回 255

如果最小 cost 大于 `max_total_cost`，当前为 `20`，返回 `255`。

这沿用了原工程行为。

---

## 7. IK 到电机角完整流程

入口函数：

```c
Kinematic_SolveIK_To_Motor()
```

它等价于原工程 `Robot_ik()` 的组合流程。

### 7.1 求所有 IK 解

```text
Inverse_Kinematics_From_Rotation_Matrix()
```

### 7.2 选最优解

将内部解转换为 `IKSolves_t`，然后调用：

```text
Kinematic_Select_Best_Sol()
```

### 7.3 无解或跳变过大 fallback

如果：

```text
selected == -1
selected == 255
selected >= solution_count
```

则使用上一帧：

```text
selected_joint = handle->current_ik_rad
solved = false
```

也就是说，无解时不会输出乱跳角，而是保持上一帧安全角。

### 7.4 有解时腕部连续化

如果有解，则调用：

```text
Fit_Angle_Solution()
```

它只处理 q4/q5/q6：

```text
如果 q4/q5/q6 与上一帧差值 > pi，减 2pi
如果差值 < -pi，加 2pi
```

然后更新：

```text
handle->current_ik_rad
handle->last_answer_rad
```

### 7.5 IK 角转电机角

先把 IK 角从 rad 转 degree，写入 `last_valid_ik_joint_deg`。

再调用：

```text
Kinematic_IKAngle_To_Motor()
```

该函数使用仿射映射：

```text
motor[i] = bias[i] + Σ map[i][j] * ik_joint[j]
```

当前配置等价于原工程 `IK_Angle_Translate()`。

### 7.6 电机角限位

最后执行：

```text
Kinematic_Limit_Motor_Angle()
```

进行单轴限位和 q2/q3 耦合限位。

---

## 8. 可以优化的点

### 优化 1：避免 IK 重复封装维护

`Kinematic_SolveIK()` 和 `Kinematic_SolveIK_To_Motor()` 都会组织 IK 求解流程。后续可以让 `Kinematic_SolveIK_To_Motor()` 直接调用 `Kinematic_SolveIK()`，减少重复维护。

### 优化 2：明确 `Kinematic_SolveIK()` 是否更新 q3 状态

目前 `Kinematic_SolveIK_To_Motor()` 会更新 `handle->flag_beyond_180`，但单独调用 `Kinematic_SolveIK()` 时使用的是局部 flag，不写回 handle。

如果希望 `Kinematic_SolveIK()` 是纯查询函数，应在注释中说明；如果希望它也参与状态更新，应写回 handle。

### 优化 3：IK 解应参与关节限位筛选

当前选解只看距离和跳变，没有用 `joint_limit[]` 过滤候选解。

建议在 `Kinematic_Select_Best_Sol()` 中加入：

```text
如果 joint_limit[j].enable:
    candidate[j] 必须在 min/max 内
```

### 优化 4：电机限位应前置到选解阶段

当前流程是：

```text
先选 IK 解
再 joint_to_motor
再 clamp 电机角
```

问题是 clamp 后的电机角可能不再对应原目标 Pose。

更优流程：

```text
对每组 IK 解：
    joint_to_motor
    检查 motor limit / coupling limit
    不合法则丢弃或加惩罚
再选最优解
```

最后的 clamp 只作为安全兜底。

### 优化 5：q2/q3 耦合限位目前只修 q3

当前耦合限位通过修改 lhs，即 motor2/q3 来满足：

```text
motor2 = motor1 + limit
```

这和原工程行为一致，但新机械臂不一定最优。后续可以考虑同时调整 q2/q3，或者在选解阶段避免违反耦合。

### 优化 6：增强奇异点和 NaN 防护

当前已经对部分 `temp < 0` 做了处理，但还可以增加：

```text
abs(a2) < epsilon 时无解
小负数 temp ∈ [-epsilon, 0] 当 0 处理
腕部奇异时特殊处理 q4/q6
```

### 优化 7：去掉矩阵→欧拉→矩阵绕行

当前 `Inverse_Kinematics_From_Rotation_Matrix()` 仍然会：

```text
rotation matrix -> EulerAngles -> create_end_effector_state -> rotation matrix
```

这继承自原工程，但会带来额外计算和欧拉角奇异风险。

后续可以直接让 IK 使用输入的 `Matrix3x3 R06`，不要绕欧拉角。

### 优化 8：统一 `Pose6D_t.A/B/C` 语义

当前建议统一为：

```text
A = roll
B = pitch
C = yaw
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

所有 IK 路径优先使用 `Pose.R`，降低欧拉角顺序错误风险。

### 优化 9：函数命名可以更清楚

当前：

```text
Kinematic_ControllerFK()
```

实际做的是：

```text
控制器三电机 FK + IMU 姿态 -> 机械臂目标 Pose
```

它不是机械臂 FK。

后续可以考虑重命名为：

```text
Kinematic_BuildPoseFromController()
Kinematic_SolveArmFK()
Kinematic_SolveArmIK()
```

### 优化 10：配置结构可以进一步拆分

现在 `ArmConfig_t` 同时包含：

- 机械臂 DH
- joint/motor 映射
- 限位
- 选解参数
- Delta 控制器参数

后续可以拆成：

```text
ArmKinematicConfig_t
ArmJointMapConfig_t
ArmLimitConfig_t
ControllerDeltaConfig_t
```

这样新机械臂调参会更清晰。

---

## 9. 建议后续优先级

建议按以下顺序优化：

1. 先做实机前检查：DH 参数、坐标方向、A/B/C 姿态定义。
2. 加入 `joint_limit` 参与 IK 选解。
3. 将电机限位和耦合限位前置到选解阶段。
4. 去掉矩阵→欧拉→矩阵的重复转换。
5. 增加奇异点和 epsilon 防护。
6. 再考虑函数重命名和配置结构拆分。

当前版本适合作为 `rm_gongcheng_base-Delta` 控制器算法 baseline。若新机械臂尺寸不同，优先在 `Dummy_Motormatic_Build_Kinematic_Config()` 中调整 DH、限位和映射配置，而不是先改 IK 公式本身。

---

## 10. 本次完善记录（2026-06-12）

已按上述优先级完成第一批低风险完善，重点是让 IK 选解先满足机械/电机约束，再进入连续性 cost 选择：

1. `Kinematic_Select_Best_Sol()` 已在 cost 计算前加入候选解合法性筛选：
   - `joint_limit[]` 启用时先过滤 IK 关节角；
   - 候选 IK 角会先经 `joint_to_motor` 映射为电机角；
   - `motor_limit[]` 与 `coupling_limits[]` 作为硬约束参与筛选；
   - 最后的 `Kinematic_Limit_Motor_Angle()` 仍保留为安全兜底，但正常路径不再依赖“先选解后 clamp”。

2. `Inverse_Kinematics_From_Rotation_Matrix()` 已改为直接使用输入的 `Matrix3x3 R06`，去掉原先 `rotation matrix -> Euler -> rotation matrix` 的绕行。`Pose6D_t.hasR == false` 时仍由 `Pose_To_Vector_Rotation()` 使用 `A/B/C` 构造旋转矩阵。

3. IK 内部已补充奇异点和数值保护：
   - 输入 position / rotation 元素 finite 检查；
   - `fabs(a2) < epsilon` 时直接无解；
   - `sqrt()` 参数小负数归零，明显负数返回无解；
   - 候选解含 NaN/Inf 时不加入解集。

4. `Dummy_Motormatic_Build_Kinematic_Config()` 中保持现有 DH、motor limit、q2/q3 coupling limit 不变，仅对 1:1 映射且已有 motor 限位的 joint5 补充 `joint_limit`，q2/q3 仍优先通过 motor/coupling 约束筛选。

5. 同批还修复了控制器稳定性问题：daemon 注册/离线回调保护、PID 浮点 `fabsf()` 与 dt 下限、UART 接收重启、Vision 发送失败释放 `tx_busy`、夹爪计数下溢、DJI 输出转换前限幅。

6. 蓝牙接收协议已按实机输入调整为 6 轴关节角：`RX_BT_Data_s` 字段为 `joint1` ~ `joint6`，单位 deg；右拨杆上档接收到蓝牙有效帧后进入 `ARM_PC_MODE`，直接填充 `joint*_angle`，不再把蓝牙数据当末端 XYZ/RPY 走 `ARM_CARTESIAN_MODE` IK。

后续若继续优化，建议再做：实机确认 DH/坐标方向与关节正方向、根据机械臂真实极限补全 `joint_limit[]`、再考虑函数重命名和配置结构拆分。
