// #include "arm_math.h"
#include "Mathcal.h"
#include "stdint.h"
#include "string.h"
#include "math.h"

static float dh[4][7];
static float d[7];
static float a[7];
static float alpha[7];
static float current_q[7];
typedef enum
{
    NO_Found_BEST_IK = 0x00,
    FOUND_BEST_IK = 0x01,
} ROBOT_IK_SOLUTION;
void DH_Init(void)
{
    d[1] = 0.f;
    d[2] = 0.f;
    d[3] = 0.f;
    d[4] = 0.0425;
    d[5] = 0.076;
    d[6] = 0.05;
    a[1] = 0.f;
    a[2] = 0.25;
    a[3] = 0.4;
    a[4] = 0.f;
    a[5] = 0.f;
    a[6] = 0.f;
    alpha[1] = 0.f;
    alpha[2] = pi / 2;
    alpha[3] = 0.f;
    alpha[4] = pi / 2;
    alpha[5] = -pi / 2;
    alpha[6] = 0.f;
    //* this is my initial position
    current_q[1] = 0.f * Degree_2_Rad;
    current_q[2] = 165.f * Degree_2_Rad;
    current_q[3] = -165.f * Degree_2_Rad;
    current_q[4] = 90.f * Degree_2_Rad;
    current_q[5] = -90.f * Degree_2_Rad;
    current_q[6] = 0.f * Degree_2_Rad;
}
// 封装函数
static float math_pow(float base, uint32_t exp)
{

    float out1;
    arm_power_f32(&base, exp, &out1);
    return out1;
}
static float math_sqrt(float in)
{
    float out;
    arm_sqrt_f32(in, &out);
    return out;
}
static float math_atan2(float y, float x)
{
    float res;
    arm_atan2_f32(y, x, &res);
    return res;
}
static float math_cos(float in)
{
    return arm_cos_f32(in);
}
static float math_sin(float in)
{

    return arm_sin_f32(in);

}
static float math_abs(float in)
{
    float ou;
    arm_abs_f32(&in, &ou, 1);
    return ou;

}
void Transformation_Init(void)
{
    DH_Init();
}
/**
 * @brief 正运动学解算
 * @param *q :关节矩阵
 */
void math_fk(float *q, float transformation_matrix[5][5])
{
}
/**
 * @brief 选择最优解
 * @param q_solution: 所有解的矩阵
 * @param size: 解的数量
 * @param curent_q: 当前关节角度
 * @note 需要注意的是我们可以做一个欧式距离，如果每一个值都很大，那么表明无法在附近找到更好的点，返回原来的值并且返回错误标志位
 */
static uint8_t Select_best_solution(float q_solution[10][7], uint8_t size, float *Curent_q, float *q_val, float *live_error)
{

    // TODO: 需要调整错误系数

    static float q_best[7] = {0.f};
    static float min_error = 0.f;
    memset(q_best, 0.f, sizeof(q_best));
    float theta_error[20] = {0.f};

    for (uint8_t i = 1; i <= size; i++)
    {
        float diff_sum = 0.f;
        float diff_sum_sprt = 0.f;
        for (uint8_t j = 1; j <= 6; j++)
        {
            float diff1 = math_abs(q_solution[i][j] - Curent_q[j]);
            float diff2 = 2 * pi - diff1;
            float min_diff = (diff1 < diff2) ? diff1 : diff2;
            float weight = 7 - j;
            diff_sum += (weight * min_diff);
        }
        theta_error[i] = diff_sum;
    }
    // 选择误差最小的解
    min_error = theta_error[1];

    uint8_t min_index = 1;
    for (uint8_t i = 2; i <= size; i++)
    {
        if (theta_error[i] < min_error)
        {
            min_error = theta_error[i];
            min_index = i;
        }
    }
    *live_error = min_error;
    //* 因为我给的数组里面有0，所以要判断错误，一般都不会全为0，
    //* 可以理解为找不到错误阈值内的解
    if (q_solution[min_index][1] == 0.f && q_solution[min_index][2] == 0.f && q_solution[min_index][3] == 0.f &&
        q_solution[min_index][4] == 0.f && q_solution[min_index][5] == 0.f && q_solution[min_index][6] == 0.f)
    {
        min_error = ERROR_THRESHOLD;
        for (uint8_t i = 1; i <= 6; i++)
            q_val[i] = Curent_q[i];
        return NO_Found_BEST_IK;
    }
    if (min_error > ERROR_THRESHOLD)
    {
        min_error = ERROR_THRESHOLD;
        for (uint8_t i = 1; i <= 6; i++)
            q_val[i] = Curent_q[i];
        return NO_Found_BEST_IK;
    }
    for (uint8_t j = 1; j < 7; j++)
    {
        min_error = ERROR_THRESHOLD;
        q_val[j] = q_solution[min_index][j];
    }
    return FOUND_BEST_IK;
}
// TODO: 还需要考虑q5的除0错误，应该给一个放松值
/**
 * @brief 逆运动学解算
 * @param *target: 目标位姿的矩阵[5][5]
 * @param *q_val :关节矩阵[7]
 * @note 关节角度单位为弧度，长度单位为m
 * @note xyz必要的，左上的rotation可用eye代替作为demo
 * @note 关节角度取最近的
 */
void robot_ik(float target[5][5], float *q_val, float *live_error)
{
    static uint8_t q_solution_index = 0;
    static ROBOT_IK_SOLUTION robot_ik_solution_flag = NO_Found_BEST_IK;
    float nx = target[1][1];
    float ny = target[2][1];
    float nz = target[3][1];
    float ox = target[1][2];
    float oy = target[2][2];
    float oz = target[3][2];
    float ax = target[1][3];
    float ay = target[2][3];
    float az = target[3][3];
    float px = target[1][4];
    float py = target[2][4];
    float pz = target[3][4];
    //
    float q1 = 0.f, q2 = 0.f, q3 = 0.f, q4 = 0.f, q5 = 0.f, q6 = 0.f, q234 = 0.f, q23 = 0.f;
    float A = 0.f, B = 0.f;
    float q_solution[10][7] = {0.f};
    volatile float po1, po2, pos3;
    for (uint8_t i = 1; i <= 2; i++)
    {

        float temp = math_pow((d[6] * ay - py), 2) +
                     math_pow((px - d[6] * ax), 2) - d[4] * d[4];
        if (temp < 0.f)
        {
            continue;
        }
        if (i == 1)
        {
            q1 = math_atan2(d[4], -math_sqrt(temp)) -
                 math_atan2(d[6] * ay - py, px - d[6] * ax);
        }
        else if (i == 2)
        {
            q1 = math_atan2(d[4], math_sqrt(temp)) -
                 math_atan2(d[6] * ay - py, px - d[6] * ax);
        }

        for (uint8_t j = 1; j < 3; j++)
        {
            float temp5 = math_pow((nx * math_sin(q1) - ny * math_cos(q1)), 2) + math_pow(ox * math_sin(q1) - oy * math_cos(q1), 2);

            if (temp5 < 0.f)
            {
                continue;
            }
            if (j == 1)
            {
                q5 = math_atan2(math_sqrt(temp5),
                                -(ax * math_sin(q1) - ay * math_cos(q1)));
            }
            else if (j == 2)
            {
                q5 = math_atan2(-math_sqrt(temp5),
                                -(ax * math_sin(q1) - ay * math_cos(q1)));
            }
            if (math_abs(q5) < math_pow(0.1, 9))
            {
                continue;
            }

            q6 = math_atan2(-(ox * math_sin(q1) - oy * math_cos(q1)) / math_sin(q5),
                            (nx * math_sin(q1) - ny * math_cos(q1)) / math_sin(q5));
            po1 = -az / math_sin(-q5);
            po2 = -(ax * math_cos(q1) + ay * math_sin(q1)) / math_sin(-q5);
            pos3 = po1 + po2;
            if (math_abs(po1) < 0.01)
                q234 = math_atan2(+0.01,
                                  -(ax * math_cos(q1) + ay * math_sin(q1)) / math_sin(-q5));
            else
                q234 = math_atan2(po1,
                                  -(ax * math_cos(q1) + ay * math_sin(q1)) / math_sin(-q5));

            A = px * math_cos(q1) + py * math_sin(q1) - d[5] * math_sin(q234) +
                d[6] * math_sin(-q5) * math_cos(q234);

            B = pz - d[1] + d[5] * math_cos(q234) + d[6] * math_sin(-q5) * math_sin(q234);

            for (uint8_t w = 1; w <= 2; w++)
            {

                float temp2 = A * A + B * B + a[2] * a[2] - a[3] * a[3];
                float temp2_sqrt = 4 * a[2] * a[2] * (A * A + B * B) - temp2 * temp2;

                if (temp2_sqrt < 0.f)
                {
                    continue;
                }
                if (w == 1)
                {
                    q2 = math_atan2(temp2, math_sqrt(temp2_sqrt)) - math_atan2(A, B);
                }
                else if (w == 2)
                {
                    q2 = math_atan2(temp2, -math_sqrt(temp2_sqrt)) - math_atan2(A, B);
                }
                q23 = math_atan2(pz - d[1] + d[5] * math_cos(q234) - a[2] * math_sin(q2) + d[6] * math_sin(-q5) * math_sin(q234),
                                 px * math_cos(q1) + py * math_sin(q1) - d[5] * math_sin(q234) - a[2] * math_cos(q2) + d[6] * math_sin(-q5) * math_cos(q234));
                q3 = q23 - q2;
                q4 = q234 - q23;
                float q[7] = {0.f, q1, q2, q3, q4, q5, q6};

                // 保存解
                q_solution_index++;
                for (uint8_t m = 1; m < 7; m++)
                {
                    if (q[m] > pi)
                    {
                        q[m] -= 2 * pi;
                    }
                    else if (q[m] < -pi)
                    {
                        q[m] += 2 * pi;
                    }
                    if (q[2] < 0.f)
                        q[2] += 2 * pi;
                    q_solution[q_solution_index][m] = q[m];
                }
            }
        }
    }
    // TODO:这里返回了找到解和没有找到解的标志位，需要通过这个标志位来告诉操作手，不可到达姿态
    robot_ik_solution_flag = Select_best_solution(q_solution, q_solution_index, current_q, q_val, live_error);
    //* 如果找到了最好的解，那么把这次的赋值给current_q
    if (robot_ik_solution_flag == FOUND_BEST_IK)
    {
        for (uint8_t i = 1; i <= 6; i++)
            current_q[i] = q_val[i];
    }
    q_solution_index = 0;
    memset(q_solution, 0.f, sizeof(q_solution));
}
