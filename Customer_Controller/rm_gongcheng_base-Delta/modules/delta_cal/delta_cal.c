#include "delta_cal.h"
#include "arm_math.h"
#include "stdint.h"
#include "string.h"
#include "math.h"
#include "User_Math.h"
#include "stdlib.h"
// 此文件的所有矩阵和向量均从1开始计数，0位置不使用
// 均使用弧度制
const static float R = 0.090f;
const static float r = 0.035f;
const static float L = 0.120f;
const static float La = 0.270f;
const static float phi[4] = {0, 0, 2 * pi / 3, 4 * pi / 3};

// 封装函数
/**
 * @brief 计算base的exp次方
 */
static float math_pow(float base, uint32_t exp)
{

    float out1;
    arm_power_f32(&base, exp, &out1);

    return out1;
}
/**
 * @brief 计算atan
 */
static float math_atan(float in)
{
    return atanf(in);
}
/**
 * @brief 计算开方
 */
static float math_sqrt(float in)
{
    float out;
    arm_sqrt_f32(in, &out);
    return out;
}
/**
 * @brief 计算atan2
 */
static float math_atan2(float y, float x)
{
    float res;
    arm_atan2_f32(y, x, &res);
    return res;
}
/**
 * @brief 计算cos
 */
static float math_cos(float in)
{
    return arm_cos_f32(in);
}
/**
 * @brief 计算sin
 */
static float math_sin(float in)
{

    return arm_sin_f32(in);
}
/**
 * @brief 绝对值
 */
static float math_abs(float in)
{
    float ou;
    if (in < 0)
        ou = -in;
    else
        ou = in;
    return ou;
}

/**
 * @brief delta机械臂正解
 * @param theta: theta角度，单位 rad   取1~3
 * @param target_xyz: 输出参数，单位 m  取1~3
 */
void Delta_fk(float *theta, float *target_xyz)
{

    float theta1 = theta[1];
    float theta2 = theta[2];
    float theta3 = theta[3];

    float A1 = R + L * math_cos(theta1) - r;
    float B1 = 1.0;
    float C1 = L * math_sin(theta1);

    float A2 = -0.5 * (R + L * math_cos(theta2) - r);
    float B2 = (math_sqrt(3.0) / 2.0) * (R + L * math_cos(theta2) - r);
    float C2 = L * math_sin(theta2);

    float A3 = -0.5 * (R + L * math_cos(theta3) - r);
    float B3 = -(math_sqrt(3.0) / 2.0) * (R + L * math_cos(theta3) - r);
    float C3 = L * math_sin(theta3);

    float D1 = 0.5 * (A2 * A2 - A1 * A1 + C2 * C2 - C1 * C1 + B2 * B2);
    float A21 = A2 - A1;
    float C21 = C2 - C1;
    float D2 = 0.5 * (A3 * A3 - A1 * A1 + C3 * C3 - C1 * C1 + B3 * B3);
    float A31 = A3 - A1;
    float C31 = C3 - C1;

    float denom1 = A21 * B3 - A31 * B2; /* 公共分母 1 */
    float E1 = (B3 * C21 - B2 * C31) / denom1;
    float F1 = (B3 * D1 - B2 * D2) / denom1;

    float denom2 = A31 * B2 - A21 * B3; /* 公共分母 2 */
    float E2 = (A31 * C21 - A21 * C31) / denom2;
    float F2 = (A31 * D1 - A21 * D2) / denom2;

    float a = E1 * E1 + E2 * E2 + 1.0;
    float b = 2.0 * E2 * F2 + 2.0 * C1 - 2.0 * E1 * (A1 - F1);
    float c = (A1 - F1) * (A1 - F1) + F2 * F2 + C1 * C1 - La * La;

    float Z = (-b - math_sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
    float X = E1 * Z + F1;
    float Y = E2 * Z + F2;
    target_xyz[1] = X;
    target_xyz[2] = Y;
    target_xyz[3] = Z;
}
/**
 * @brief delta机械臂逆解
 * @param target: xyz轴坐标，单位 m  取1~3
 */
void Delta_ik(float *target)
{
    // % %
    //     eta1 = pi / 6;
    // eta2 = pi * 5 / 6;
    // eta3 = -pi / 2;
    // eta = [eta1; eta2; eta3];
    // % alpha : 主动臂和静平台的夹角 % L1为主动臂杆长 % L2为从动臂杆长 % R : 静平台的半径 % r : 动平台的半径 alpha = zeros(3, 1);
    // float eta1 = pi / 6, eta2 = pi * 5 / 6, -pi / 2;
    // TODO: 配置的参数需要修改，同时eta已经被算入里面，无法修改了

    float x = target[1];
    float y = target[2];
    float z = target[3];

    // // % A1, B1, C1
    float A1 = (x * x + y * y + z * z + L * L - La * La + (R - r) * (R - r) - 2 * x * (R - r)) / (2 * L);
    float B1 = -(R - r - x);
    float C1 = z;

    // % A2, B2, C2
    float A2 = (x * x + y * y + z * z + L * L - La * La + (R - r) * (R - r) + (x - math_sqrt(3) * y) * (R - r)) / L;
    float B2 = -2 * (R - r) - (x - math_sqrt(3) * y);
    float C2 = 2 * z;

    // % A3, B3, C3
    float A3 = (x * x + y * y + z * z + L * L - La * La + (R - r) * (R - r) + (x + math_sqrt(3) * y) * (R - r)) / L;
    float B3 = -2 * (R - r) - (x + math_sqrt(3) * y);
    float C3 = 2 * z;

    float K1 = A1 + B1;
    float U1 = 2 * C1;
    float V1 = A1 - B1;

    float K2 = A2 + B2;
    float U2 = 2 * C2;
    float V2 = A2 - B2;

    float K3 = A3 + B3;
    float U3 = 2 * C3;
    float V3 = A3 - B3;

    float theta1 = 2 * math_atan((-U1 - math_sqrt(U1 * U1 - 4 * K1 * V1)) / (2 * K1));
    float theta2 = 2 * math_atan((-U2 - math_sqrt(U2 * U2 - 4 * K2 * V2)) / (2 * K2));
    float theta3 = 2 * math_atan((-U3 - math_sqrt(U3 * U3 - 4 * K3 * V3)) / (2 * K3));
}
/**
 * @brief 3x3矩阵的乘法
 * @param a 输入矩阵a[4][4]
 * @param b 输入矩阵[4][4]
 * @param res 输出矩阵[4][4]
 */
static void Matrix_3_cross_product_3(float a[4][4], float b[4][4], float res[4][4])
{
    // 清零整个矩阵（保持与原始函数相同行为）
    memset(res, 0, 4 * 4 * sizeof(float));

    // 直接计算3x3部分
    res[1][1] = a[1][1] * b[1][1] + a[1][2] * b[2][1] + a[1][3] * b[3][1];
    res[1][2] = a[1][1] * b[1][2] + a[1][2] * b[2][2] + a[1][3] * b[3][2];
    res[1][3] = a[1][1] * b[1][3] + a[1][2] * b[2][3] + a[1][3] * b[3][3];

    res[2][1] = a[2][1] * b[1][1] + a[2][2] * b[2][1] + a[2][3] * b[3][1];
    res[2][2] = a[2][1] * b[1][2] + a[2][2] * b[2][2] + a[2][3] * b[3][2];
    res[2][3] = a[2][1] * b[1][3] + a[2][2] * b[2][3] + a[2][3] * b[3][3];

    res[3][1] = a[3][1] * b[1][1] + a[3][2] * b[2][1] + a[3][3] * b[3][1];
    res[3][2] = a[3][1] * b[1][2] + a[3][2] * b[2][2] + a[3][3] * b[3][2];
    res[3][3] = a[3][1] * b[1][3] + a[3][2] * b[2][3] + a[3][3] * b[3][3];
}

/**
 * @brief 3x1向量相减
 * @param a 输入矩阵a[4] 3x1
 * @param b 输入矩阵b[4] 3x1
 * @param res 输出矩阵[4] 3x1
 */
static void Matrix_reduce_3x1(float *a, float *b, float *res)
{
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
    res[3] = a[3] - b[3];
}
/**
 * @brief 3x1向量相加
 * @param a 输入矩阵a[4] 3x1
 * @param b 输入矩阵b[4] 3x1
 * @param res 输出矩阵[4] 3x1
 */
static void Matrix_add_3x1(float *a, float *b, float *res)
{
    res[1] = a[1] + b[1];
    res[2] = a[2] + b[2];
    res[3] = a[3] + b[3];
}
/**
 * @brief 3x3矩阵转置
 * @param a 输入矩阵a[4][4]  3x3
 * @param res 输出向量[4][4] 3x3
 */
static void Matrix_T_3x3(float a[4][4], float res[4][4])
{
    // 直接展开所有赋值操作，消除循环开销
    res[1][1] = a[1][1];
    res[1][2] = a[2][1];
    res[1][3] = a[3][1];
    res[2][1] = a[1][2];
    res[2][2] = a[2][2];
    res[2][3] = a[3][2];
    res[3][1] = a[1][3];
    res[3][2] = a[2][3];
    res[3][3] = a[3][3];
}
/**
 * @brief 1x3向量叉乘3x1向量
 * @param a 输入矩阵a[4]    1x3
 * @param b 输入矩阵[4]     3x1
 * @param *res 输出向量指针      1
 */
static void Matrix_1x3_cross_product_3x1(float *a, float *b, float *res)
{
    *res = a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}
/**
 * @brief 3x3矩阵求行列式
 * @param in 输入矩阵
 * @param *res 输出行列式指针
 */
static uint8_t Matrix_3x3_det(float in[4][4], float *res)
{
    float a = in[1][1], b = in[1][2], c = in[1][3];
    float d = in[2][1], e = in[2][2], f = in[2][3];
    float g = in[3][1], h = in[3][2], i = in[3][3];
    static float out = 0;
    *res = a * (e * i - f * h) -
           b * (d * i - f * g) +
           c * (d * h - e * g);

    if (math_abs(*res) < 0.001)
        return -1;
    else
        return 1;
}
/**
 * @brief 3x3矩阵求逆
 * @param in 输入矩阵[4][4]
 * @param det 输入矩阵的行列式
 * @param inv 输出矩阵[4][4]
 */
static void Matrix_3x3_inv(float in[4][4], float det, float inv[4][4])
{

    if (det == 0.0f)
    {
        return;
    }

    // 提取 in[1..3][1..3] 的 3×3 子矩阵
    float a = in[1][1], b = in[1][2], c = in[1][3];
    float d = in[2][1], e = in[2][2], f = in[2][3];
    float g = in[3][1], h = in[3][2], i = in[3][3];

    // 伴随矩阵（余子式矩阵的转置）
    inv[1][1] = (e * i - f * h) / det;
    inv[1][2] = -(b * i - c * h) / det;
    inv[1][3] = (b * f - c * e) / det;

    inv[2][1] = -(d * i - f * g) / det;
    inv[2][2] = (a * i - c * g) / det;
    inv[2][3] = -(a * f - c * d) / det;

    inv[3][1] = (d * h - e * g) / det;
    inv[3][2] = -(a * h - b * g) / det;
    inv[3][3] = (a * e - b * d) / det;
}
/**
 * @brief 3x3矩阵取负
 * @param in 输入矩阵[4][4]
 * @param out 输出矩阵[4][4]
 */
static void Matrix_3x3_take_negative(float in[4][4], float out[4][4])
{
    out[1][1] = -in[1][1];
    out[1][2] = -in[1][2];
    out[1][3] = -in[1][3];
    out[2][1] = -in[2][1];
    out[2][2] = -in[2][2];
    out[2][3] = -in[2][3];
    out[3][1] = -in[3][1];
    out[3][2] = -in[3][2];
    out[3][3] = -in[3][3];
}
/**
 * @brief 3x3矩阵叉乘3x1向量
 * @param a 输入矩阵a[4][4] 3x3
 * @param b 输入向量[4]     3x1
 * @param res 输出向量[4]   3x1
 */
static void Matrix_3x3_cross_product_3x1(const float a[4][4], const float *b, float *res)
{
    // 清零结果（假设res是4元素数组）
    res[0] = 0;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;

    // 直接展开矩阵-向量乘法
    res[1] = a[1][1] * b[1] + a[1][2] * b[2] + a[1][3] * b[3];
    res[2] = a[2][1] * b[1] + a[2][2] * b[2] + a[2][3] * b[3];
    res[3] = a[3][1] * b[1] + a[3][2] * b[2] + a[3][3] * b[3];
}
/**
 * @brief 重力补偿
 * @param theta 旋转角度 [4] rad
 * @param tor 旋转角度对应的力矩 [4]
 * @note 应该是已知角度，得出xyz
 * @bug 第一版代码写完，但是还没有验证
 */
void Gravity_compensation(float *theta_tar, float *tor, float *xyz)
{
    float Jacobi[4][4];
    float a = La;
    float phi[4] = {0, 0, 2 * pi / 3, 4 * pi / 3};
    float gravity[4] = {0, -13, 0, 0}; // xyz重力项 //* 我们的是沿x轴反方向
    //
    float target[4];

    float theta[4];
    theta[1] = theta_tar[1];
    theta[2] = theta_tar[2];
    theta[3] = theta_tar[3];
    Delta_fk(theta, target);
    xyz[1] = target[1];
    xyz[2] = target[2];
    xyz[3] = target[3];

    float R1[4][4] = {
        {0, 0, 0, 0},
        {0, math_cos(phi[1]), -math_sin(phi[1]), 0},
        {0, math_sin(phi[1]), math_cos(phi[1]), 0},
        {0, 0, 0, 1}}; // 3x3
    float R2[4][4] = {
        {0, 0, 0, 0},
        {0, math_cos(phi[2]), -math_sin(phi[2]), 0},
        {0, math_sin(phi[2]), math_cos(phi[2]), 0},
        {0, 0, 0, 1}}; // 3x3
    float R3[4][4] = {
        {0, 0, 0, 0},
        {0, math_cos(phi[3]), -math_sin(phi[3]), 0},
        {0, math_sin(phi[3]), math_cos(phi[3]), 0},
        {0, 0, 0, 1}}; // 3x3
    float S1[4], S2[4], S3[4];
    // S1   3x1
    float temp1[4] = {0, L * math_cos(theta[1]) + R - r, 0, -L * math_sin(theta[1])};
    float t1[4] = {0}; // 3x1
    Matrix_3x3_cross_product_3x1(R1, temp1, t1);
    Matrix_reduce_3x1(target, t1, S1); //
    // S2   3x1
    float temp2[4] = {0, L * math_cos(theta[2]) + R - r, 0, -L * math_sin(theta[2])};
    float t2[4] = {0}; // 3x1
    Matrix_3x3_cross_product_3x1(R2, temp2, t2);
    Matrix_reduce_3x1(target, t2, S2);
    // S3   3x1
    float temp3[4] = {0, L * math_cos(theta[3]) + R - r, 0, -L * math_sin(theta[3])};
    float t3[4] = {0}; // 3x1
    Matrix_3x3_cross_product_3x1(R3, temp3, t3);
    Matrix_reduce_3x1(target, t3, S3);

    // b1 b2 b3 3x1
    float b1[4] = {0}, b2[4] = {0}, b3[4] = {0};
    float temp11[4] = {0, -L * math_sin(theta[1]), 0, -L * math_cos(theta[1])};
    float temp21[4] = {0, -L * math_sin(theta[2]), 0, -L * math_cos(theta[2])};
    float temp31[4] = {0, -L * math_sin(theta[3]), 0, -L * math_cos(theta[3])};
    Matrix_3x3_cross_product_3x1(R1, temp11, b1);
    Matrix_3x3_cross_product_3x1(R2, temp21, b2);
    Matrix_3x3_cross_product_3x1(R3, temp31, b3);
    // Jacobi
    float Jacobi_temp[4][4] = {0};

    float S1b1 = 0, S2b2 = 0, S3b3 = 0;
    Matrix_1x3_cross_product_3x1(S1, b1, &S1b1);
    Matrix_1x3_cross_product_3x1(S2, b2, &S2b2);
    Matrix_1x3_cross_product_3x1(S3, b3, &S3b3);
    float Sb[4][4] = {
        {0, 0, 0, 0},
        {0, S1b1, 0, 0},
        {0, 0, S2b2, 0},
        {0, 0, 0, S3b3}};
    float S_T[4][4] = {
        {0, 0, 0, 0},
        {0, S1[1], S1[2], S1[3]},
        {0, S2[1], S2[2], S2[3]},
        {0, S3[1], S3[2], S3[3]},
    };
    float S_det = 0;
    float S_T_inv[4][4] = {0};
    if (Matrix_3x3_det(S_T, &S_det) != -1)
    {
        // 求S的逆矩阵//begin
        Matrix_3x3_inv(S_T, S_det, S_T_inv);
        // end
        Matrix_3_cross_product_3(S_T_inv, Sb, Jacobi_temp);
        Matrix_3x3_take_negative(Jacobi_temp, Jacobi);
    }
    else
    {
        S_det = 0;
        //!
    }
    float Jacobo_T[4][4] = {0};
    Matrix_T_3x3(Jacobi, Jacobo_T);
    float torque[4] = {0};
    Matrix_3x3_cross_product_3x1(Jacobo_T, gravity, torque);
    tor[1] = -torque[1];
    tor[2] = -torque[2];
    tor[3] = -torque[3];
    // clear
    memset(temp1, 0, sizeof(temp1));
    memset(temp2, 0, sizeof(temp2));
    memset(temp3, 0, sizeof(temp3));
    memset(t1, 0, sizeof(t1));
    memset(t2, 0, sizeof(t2));
    memset(t3, 0, sizeof(t3));
}
/**
 * @brief euler的Z-Y-X转换为旋转矩阵
 * @param rotation 输出旋转矩阵[4][4]
 * @param yaw
 *
 */
void Rotation_ZYX_Matrix(float rotation[4][4], float yaw, float pitch, float roll)
{
    yaw *= DEG_TO_RAD;
    pitch *= DEG_TO_RAD;
    roll *= DEG_TO_RAD;
    float R_Z[4][4] = {
        {0, 0, 0, 0},
        {0, math_cos(yaw), -math_sin(yaw), 0},
        {0, math_sin(yaw), math_cos(yaw), 0},
        {0, 0, 0, 1},
    };
    float R_Y[4][4] = {
        {0, 0, 0, 0},
        {0, math_cos(pitch), 0, math_sin(pitch)},
        {0, 0, 1, 0},
        {0, -math_sin(pitch), 0, math_cos(pitch)},
    };
    float R_X[4][4] = {
        {0, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, math_cos(roll), -math_sin(roll)},
        {0, 0, math_sin(roll), math_cos(roll)},
    };
    float temp1[4][4] = {0};
    Matrix_3_cross_product_3(R_Y, R_X, temp1);
    Matrix_3_cross_product_3(R_Z, temp1, rotation);
}