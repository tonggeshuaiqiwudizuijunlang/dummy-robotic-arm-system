#include "ARM_Moveit/arm_moveit.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>

/** 创建单位矩阵 */
Matrix4x4 identity_matrix4x4()
{
    Matrix4x4 I;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            I.m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    return I;
}

Matrix3x3 identity_matrix3x3()
{
    Matrix3x3 I;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            I.m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    return I;
}

/** 矩阵乘法: C = A * B */
Matrix4x4 matrix4x4_multiply(const Matrix4x4 *A, const Matrix4x4 *B)
{
    Matrix4x4 C;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            C.m[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
            {
                C.m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }
    return C;
}

Matrix3x3 matrix3x3_multiply(const Matrix3x3 *A, const Matrix3x3 *B)
{
    Matrix3x3 C;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C.m[i][j] = 0.0;
            for (int k = 0; k < 3; k++)
            {
                C.m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }
    return C;
}

/** 从旋转矩阵创建齐次变换矩阵 */
Matrix4x4 transform_from_rotation_and_position(const Matrix3x3 *R, const Vector3 *p)
{
    Matrix4x4 T = identity_matrix4x4();

    // 复制旋转部分
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            T.m[i][j] = R->m[i][j];
        }
    }

    // 设置位置部分
    T.m[0][3] = p->x;
    T.m[1][3] = p->y;
    T.m[2][3] = p->z;

    return T;
}

/** 欧拉角转旋转矩阵 (RPY: ZYX顺序) */
Matrix3x3 euler_to_rotation_matrix(double roll, double pitch, double yaw)
{
    Matrix3x3 R;

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    R.m[0][0] = cy * cp;
    R.m[0][1] = cy * sp * sr - sy * cr;
    R.m[0][2] = cy * sp * cr + sy * sr;

    R.m[1][0] = sy * cp;
    R.m[1][1] = sy * sp * sr + cy * cr;
    R.m[1][2] = sy * sp * cr - cy * sr;

    R.m[2][0] = -sp;
    R.m[2][1] = cp * sr;
    R.m[2][2] = cp * cr;

    return R;
}

/** 旋转矩阵转欧拉角 (RPY: ZYX顺序) */
EulerAngles rotation_matrix_to_euler(const Matrix3x3 *R)
{
    EulerAngles euler;

    double sy = sqrt(R->m[0][0] * R->m[0][0] + R->m[1][0] * R->m[1][0]);

    if (sy > 1e-6)
    {
        euler.roll = atan2(R->m[2][1], R->m[2][2]);
        euler.pitch = atan2(-R->m[2][0], sy);
        euler.yaw = atan2(R->m[1][0], R->m[0][0]);
    }
    else
    {
        euler.roll = atan2(-R->m[1][2], R->m[1][1]);
        euler.pitch = atan2(-R->m[2][0], sy);
        euler.yaw = 0.0;
    }

    return euler;
}

/** 从位置和欧拉角创建末端状态 */
EndEffectorState create_end_effector_state(const Vector3 *position, const EulerAngles *orientation)
{
    EndEffectorState state;
    state.position = *position;

    // 计算旋转矩阵
    state.rotation = euler_to_rotation_matrix(orientation->roll, orientation->pitch, orientation->yaw);

    // 计算变换矩阵
    state.transform = transform_from_rotation_and_position(&state.rotation, position);

    return state;
}

/** 角度规范化到 [-π, π] */
double normalize_angle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/** 弧度转角度 */
double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

/** 角度转弧度 */
double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

/** 判断数值是否接近零 */
int is_near_zero(double value, double epsilon)
{
    return fabs(value) < epsilon;
}

// ==================== D-H变换矩阵 ====================

/**
 * 计算单个连杆的D-H变换矩阵
 */
Matrix4x4 dh_transform(double theta, double d, double a, double alpha)
{
    Matrix4x4 T;

    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    T.m[0][0] = ct;
    T.m[0][1] = -st * ca;
    T.m[0][2] = st * sa;
    T.m[0][3] = a * ct;
    T.m[1][0] = st;
    T.m[1][1] = ct * ca;
    T.m[1][2] = -ct * sa;
    T.m[1][3] = a * st;
    T.m[2][0] = 0.0;
    T.m[2][1] = sa;
    T.m[2][2] = ca;
    T.m[2][3] = d;
    T.m[3][0] = 0.0;
    T.m[3][1] = 0.0;
    T.m[3][2] = 0.0;
    T.m[3][3] = 1.0;

    return T;
}

// ==================== 正向运动学 ====================

/**
 * 计算正向运动学
 */
EndEffectorState forward_kinematics(const JointAngles *angles, const DHParameters *dh)
{
    EndEffectorState state;

    // 计算每个连杆的变换矩阵并累乘
    Matrix4x4 T = identity_matrix4x4();

    for (int i = 0; i < 6; i++)
    {

        Matrix4x4 Ti = dh_transform(angles->theta[i], dh->d[i], dh->a[i], dh->alpha[i]);
        T = matrix4x4_multiply(&T, &Ti);
    }

    // 保存变换矩阵
    state.transform = T;

    // 提取位置
    state.position.x = T.m[0][3];
    state.position.y = T.m[1][3];
    state.position.z = T.m[2][3];

    // 提取旋转矩阵
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            state.rotation.m[i][j] = T.m[i][j];
        }
    }

    return state;
}
#define DEBUG 1
static int flag_beyond_180 = 0;
/**
 * 逆运动学求解函数 - 针对 6DOF 拟人机械臂 (Spherical Wrist)
 * 构型: Y-P-P-R-P-Y
 * * @param position 末端位置 (x, y, z)
 * @param orientation 末端姿态欧拉角 (roll, pitch, yaw) 弧度
 * @param dh D-H参数
 * @return 所有可能的关节角解
 */
InverseKinematicsSolutions inverse_kinematics_from_pose(
    const Vector3 *position,
    const EulerAngles *orientation,
    const DHParameters *dh)
{

    InverseKinematicsSolutions solutions;
    solutions.count = 0;

    // 1. 准备目标位姿
    EndEffectorState desired_state = create_end_effector_state(position, orientation);

    // 提取目标旋转矩阵 R_0_6
    Matrix3x3 R06 = desired_state.rotation;

    double ax = R06.m[0][2];
    double ay = R06.m[1][2];
    double az = R06.m[2][2];
    double d2 = dh->d[1];
    double d4 = dh->d[3];
    double a2 = dh->a[2];
    double a3 = dh->a[3];
    double nx = R06.m[0][0];
    double ny = R06.m[1][0];
    double nz = R06.m[2][0];

    // 提取目标位置 P_target
    double px = desired_state.position.x - dh->d[5] * ax;
    double py = desired_state.position.y - dh->d[5] * ay;
    double pz = desired_state.position.z - dh->d[5] * az;

    double temp = px * px + py * py - d2 * d2;
    double theta1_1 = atan2(py, px) - atan2(d2, sqrt(temp));
    double theta1_2 = atan2(py, px) - atan2(d2, -sqrt(temp));

    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0, q6 = 0, q23 = 0;
    for (int i1 = 0; i1 < 2; i1++)
    {
        if (i1 == 0)
            q1 = theta1_1;
        else if (i1 == 1)
            q1 = theta1_2;

        double k = (px * px + py * py + pz * pz - a2 * a2 - a3 * a3 - d2 * d2 - d4 * d4) /
                   (2 * a2);

        temp = a3 * a3 + d4 * d4 - k * k;
        if (temp < 0)
            continue;

        for (int i3 = 0; i3 < 2; i3++)
        {
            // BUG：q3值跳变
            if (i3 == 0)
                q3 = atan2(a3, d4) - atan2(k, sqrt(temp));
            else if (i3 == 1)
                q3 = atan2(a3, d4) - atan2(k, -sqrt(temp));

            q23 = atan2(-(a3 + a2 * cos(q3)) * pz + (cos(q1) * px + sin(q1) * py) * (a2 * sin(q3) - d4),
                        (a2 * sin(q3) - d4) * pz + (cos(q1) * px + sin(q1) * py) * (a2 * cos(q3) + a3));
            q2 = q23 - q3;
            if (q3 < 0)
                q2 += M_PI / 2;
            else if (q3 > 0)
                q2 -= M_PI * 3 / 2;

            q4 = atan2(-ax * sin(q1) + ay * cos(q1),
                       -ax * cos(q1) * cos(q23) - ay * sin(q1) * cos(q23) + az * sin(q23));

            double s5 = -(ax * (cos(q1) * cos(q23) * cos(q4) + sin(q1) * sin(q4)) +
                          ay * (sin(q1) * cos(q23) * cos(q4) - cos(q1) * sin(q4)) -
                          az * (sin(q23) * cos(q4)));

            double c5 = ax * (-cos(q1) * sin(q23)) +
                        ay * (-sin(q1) * sin(q23)) +
                        az * (-cos(q23));

            q5 = atan2(s5, c5);

            double s6 = -nx * (cos(q1) * cos(q23) * sin(q4) - sin(q1) * cos(q4)) -
                        ny * (sin(q1) * cos(q23) * sin(q4) + cos(q1) * cos(q4)) +
                        nz * (sin(q23) * sin(q4));

            double c6 = nx * ((cos(q1) * cos(q23) * cos(q4) + sin(q1) * sin(q4)) * cos(q5) -
                              cos(q1) * sin(q23) * sin(q5)) +
                        ny * ((sin(q1) * cos(q23) * cos(q4) - cos(q1) * sin(q4)) * cos(q5) -
                              sin(q1) * sin(q23) * sin(q5)) -
                        nz * (sin(q23) * cos(q4) * cos(q5) + cos(q23) * sin(q5));

            q6 = atan2(s6, c6);
            // 保存有效解
            if (solutions.count < 8)
            {

                JointAngles *sol = &solutions.angles[solutions.count];
                double q3_tmp = 0;
                q3_tmp = normalize_angle(q3);
                if (q3_tmp * 180 / M_PI > 160)
                    flag_beyond_180 = 1;
                else if (q3_tmp * 180 / M_PI < 160 && q3_tmp > 0)
                    flag_beyond_180 = 0;
                if (flag_beyond_180)
                {
                    if (q3_tmp * 180 / M_PI < 0)
                        q3_tmp = 2 * M_PI + q3_tmp;
                }
                sol->theta[0] = normalize_angle(q1) * DEBUG;
                sol->theta[1] = normalize_angle(q2) * DEBUG;
                sol->theta[2] = q3_tmp * DEBUG;
                sol->theta[3] = normalize_angle(q4) * DEBUG;
                sol->theta[4] = normalize_angle(q5) * DEBUG;
                sol->theta[5] = normalize_angle(q6) * DEBUG;
                solutions.count++;
                sol = &solutions.angles[solutions.count];
                sol->theta[0] = normalize_angle(q1) * DEBUG;
                sol->theta[1] = normalize_angle(q2) * DEBUG;
                sol->theta[2] = q3_tmp * DEBUG;
                sol->theta[3] = normalize_angle(q4 + M_PI) * DEBUG;
                sol->theta[4] = normalize_angle(-q5) * DEBUG;
                sol->theta[5] = normalize_angle(q6 + M_PI) * DEBUG;
                solutions.count++;
            }
        }
    }
    return solutions;
}

/**
 * 逆运动学求解函数 - 从旋转矩阵输入
 * @param position 末端位置
 * @param rotation 末端旋转矩阵
 * @param dh D-H参数
 * @return 所有可能的关节角解
 */
InverseKinematicsSolutions inverse_kinematics_from_rotation_matrix(
    const Vector3 *position,
    const Matrix3x3 *rotation,
    const DHParameters *dh)
{

    // 创建变换矩阵
    Matrix4x4 T = transform_from_rotation_and_position(rotation, position);

    // 提取欧拉角
    EulerAngles orientation = rotation_matrix_to_euler(rotation);

    // 调用主逆运动学函数
    return inverse_kinematics_from_pose(position, &orientation, dh);
}

// ==================== 解的选择与验证 ====================

/**
 * 选择最优解 (最接近参考角度)
 */
int select_best_solution(const InverseKinematicsSolutions *solutions,
                         const JointAngles *reference)
{
    static int flag = 0;
    if (solutions->count == 0)
    {
        return -1;
    }

    double min_cost = DBL_MAX;
    int best_index = -1;

    for (int i = 0; i < solutions->count; i++)
    {
        double cost = 0.0;

        for (int j = 0; j < 6; j++)
        {
            double diff = solutions->angles[i].theta[j] - reference->theta[j];
            if (solutions->angles[i].theta[j] > deg2rad(300))
            {
                cost = DBL_MAX;
                break;
            }
            // 处理角度环绕
            if (diff > M_PI)
                diff -= 2.0 * M_PI;
            if (diff < -M_PI)
                diff += 2.0 * M_PI;
            double weight = 7.0 - j;
            cost += weight * fabs(diff);
            // 加权：前面的关节权重更大
            if (diff >= 6 || diff <= -6)
            {
                cost = DBL_MAX;
            }
        }

        if (cost < min_cost)
        {
            min_cost = cost;
            best_index = i;
        }
    }
    if (min_cost > 20)
        return 255;
    return best_index;
}
/*
 * @brief 消除角度环绕导致的真实机械臂突变
 * @note 对于真实机械臂而言， -180 ~ +180 度是同一个位置，但如果解的角度从179跳到-179，机械臂会快速旋转360度，
 * @note 实际工程应用需要避免这种情况
 */
void fit_angle_solution(JointAngles *solutions, JointAngles *reference)
{
    static double delta = 0;

    for (int j = 3; j < 6; j++)
    {
        delta = solutions->theta[j] - reference->theta[j];
        if (delta > M_PI)
        {
            solutions->theta[j] -= 2 * M_PI;
        }
        else if (delta < -M_PI)
        {
            solutions->theta[j] += 2 * M_PI;
        }
    }
}
/**
 * 验证逆运动学解
 */
double verify_solution(const JointAngles *angles,
                       const Vector3 *expected_position,
                       const EulerAngles *expected_orientation,
                       const DHParameters *dh)
{
    EndEffectorState actual_state = forward_kinematics(angles, dh);

    // 计算位置误差
    double dx = actual_state.position.x - expected_position->x;
    double dy = actual_state.position.y - expected_position->y;
    double dz = actual_state.position.z - expected_position->z;

    double position_error = sqrt(dx * dx + dy * dy + dz * dz);

    // 计算姿态误差 (可选)
    EulerAngles actual_euler = rotation_matrix_to_euler(&actual_state.rotation);

    double droll = actual_euler.roll - expected_orientation->roll;
    double dpitch = actual_euler.pitch - expected_orientation->pitch;
    double dyaw = actual_euler.yaw - expected_orientation->yaw;

    // 处理角度环绕
    if (droll > M_PI)
        droll -= 2.0 * M_PI;
    if (droll < -M_PI)
        droll += 2.0 * M_PI;
    if (dpitch > M_PI)
        dpitch -= 2.0 * M_PI;
    if (dpitch < -M_PI)
        dpitch += 2.0 * M_PI;
    if (dyaw > M_PI)
        dyaw -= 2.0 * M_PI;
    if (dyaw < -M_PI)
        dyaw += 2.0 * M_PI;

    double orientation_error = sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw);

    // 返回综合误差 (可调整权重)
    return position_error + 0.1 * orientation_error;
}

// ==================== 实用函数 ====================

/**
 * 打印关节角度
 */
void print_joint_angles(const JointAngles *angles, const char *label)
{
    printf("%s: [", label);
    for (int i = 0; i < 6; i++)
    {
        printf("%7.2f", rad2deg(angles->theta[i]));
        if (i < 5)
            printf(", ");
    }
    printf("] 度\n");
}

/**
 * 打印末端位姿
 */
void print_end_effector_pose(const Vector3 *position, const EulerAngles *orientation)
{
    printf("末端位置: [%8.4f, %8.4f, %8.4f]\n",
           position->x, position->y, position->z);
    printf("末端姿态: R=%7.2f, P=%7.2f, Y=%7.2f 度\n",
           rad2deg(orientation->roll),
           rad2deg(orientation->pitch),
           rad2deg(orientation->yaw));
}
/**
 * @brief 从4x4的数组（为3x3的矩阵）转换成3x3的矩阵
 */
void Rotation_4x4_To_3x3(float rotation[4][4], Matrix3x3 *__end_rotation)
{
    __end_rotation->m[0][0] = rotation[1][1];
    __end_rotation->m[0][1] = rotation[1][2];
    __end_rotation->m[0][2] = rotation[1][3];
    __end_rotation->m[1][0] = rotation[2][1];
    __end_rotation->m[1][1] = rotation[2][2];
    __end_rotation->m[1][2] = rotation[2][3];
    __end_rotation->m[2][0] = rotation[3][1];
    __end_rotation->m[2][1] = rotation[3][2];
    __end_rotation->m[2][2] = rotation[3][3];
}
