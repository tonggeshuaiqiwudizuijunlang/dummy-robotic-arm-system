#include "dummy_kinematic_v2.h"

#include <float.h>
#include <string.h>

#define KIN_EPSILON 0.000001f

/**
 * @brief 矩阵乘法
 * @param _m A的行数
 * @param _l A的列数 (也是B的行数)
 * @param _n B的列数
 */
static void MatMultiply(const float *_matrix1, const float *_matrix2, float *_matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

static float NormalizeRad(float angle)
{
    angle = fmodf(angle, 2.0f * (float)M_PI);
    if (angle > (float)M_PI)
        angle -= 2.0f * (float)M_PI;
    else if (angle <= -(float)M_PI)
        angle += 2.0f * (float)M_PI;
    return angle;
}

static float NormalizeDeg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
        angle -= 360.0f;
    else if (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

static float AngleDiffDeg(float target, float current)
{
    return NormalizeDeg(target - current);
}

static float ClampCos(float value)
{
    if (value > 1.0f)
        return 1.0f;
    if (value < -1.0f)
        return -1.0f;
    return value;
}

/**
 * @brief 旋转矩阵转 ZYX 欧拉角: A=Roll(X), B=Pitch(Y), C=Yaw(Z)
 *        与 Dummy 源码注释/6dof_kinematic.cpp 保持一致
 */
static void RotMatToEulerAngle(const float *_rotationM, float *_eulerAngles)
{
    float roll, pitch, yaw;

    if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f)
    {
        yaw = 0.0f;
        if (_rotationM[6] < 0.0f)
        {
            pitch = (float)M_PI_2;
            roll = atan2f(_rotationM[1], _rotationM[4]);
        }
        else
        {
            pitch = -(float)M_PI_2;
            roll = -atan2f(_rotationM[1], _rotationM[4]);
        }
    }
    else
    {
        pitch = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        float cb = cosf(pitch);
        if (fabsf(cb) < 0.0001f)
            cb = 0.0001f;
        yaw = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        roll = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = roll;
    _eulerAngles[1] = pitch;
    _eulerAngles[2] = yaw;
}

/**
 * @brief ZYX 欧拉角转旋转矩阵: R = Rz(C) * Ry(B) * Rx(A)
 */
static void EulerAngleToRotMat(const float *_eulerAngles, float *_rotationM)
{
    float cr = cosf(_eulerAngles[0]), sr = sinf(_eulerAngles[0]);
    float cp = cosf(_eulerAngles[1]), sp = sinf(_eulerAngles[1]);
    float cy = cosf(_eulerAngles[2]), sy = sinf(_eulerAngles[2]);

    _rotationM[0] = cy * cp;
    _rotationM[1] = cy * sp * sr - sy * cr;
    _rotationM[2] = cy * sp * cr + sy * sr;

    _rotationM[3] = sy * cp;
    _rotationM[4] = sy * sp * sr + cy * cr;
    _rotationM[5] = sy * sp * cr - cy * sr;

    _rotationM[6] = -sp;
    _rotationM[7] = cp * sr;
    _rotationM[8] = cp * cr;
}

// -------------------------------------------------------------------------
//                            核心接口实现
// -------------------------------------------------------------------------

/**
 * @brief 初始化运动学句柄
 */
void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config)
{
    memcpy(&handle->config, config, sizeof(ArmConfig_t));

    const ArmConfig_t *cfg = &handle->config;

    // 参考 Dummy 源码注释/6dof_kinematic.cpp 的 DH home offset
    float tmp_DH[6][4] = {
        {0.0f,             cfg->L_BASE,    cfg->D_BASE,  -(float)M_PI_2},
        {-(float)M_PI_2,   0.0f,           cfg->L_ARM,   0.0f},
        {(float)M_PI_2,    cfg->D_ELBOW,   0.0f,         (float)M_PI_2},
        {0.0f,             cfg->L_FOREARM, 0.0f,         -(float)M_PI_2},
        {0.0f,             0.0f,           0.0f,         (float)M_PI_2},
        {0.0f,             cfg->L_WRIST,   0.0f,         0.0f}
    };
    memcpy(handle->DH_matrix, tmp_DH, sizeof(tmp_DH));

    float tmp_L1[3] = {cfg->D_BASE, -cfg->L_BASE, 0.0f};
    memcpy(handle->L1_base, tmp_L1, sizeof(tmp_L1));

    float tmp_L2[3] = {cfg->L_ARM, 0.0f, 0.0f};
    memcpy(handle->L2_arm, tmp_L2, sizeof(tmp_L2));

    float tmp_L3[3] = {-cfg->D_ELBOW, 0.0f, cfg->L_FOREARM};
    memcpy(handle->L3_elbow, tmp_L3, sizeof(tmp_L3));

    float tmp_L6[3] = {0.0f, 0.0f, cfg->L_WRIST};
    memcpy(handle->L6_wrist, tmp_L6, sizeof(tmp_L6));

    handle->l_se_2 = cfg->L_ARM * cfg->L_ARM;
    handle->l_se = cfg->L_ARM;
    handle->l_ew_2 = cfg->L_FOREARM * cfg->L_FOREARM + cfg->D_ELBOW * cfg->D_ELBOW;
    handle->l_ew = (handle->l_ew_2 > KIN_EPSILON) ? sqrtf(handle->l_ew_2) : 0.0f;
    handle->atan_e = atan2f(cfg->D_ELBOW, cfg->L_FOREARM);
}

/**
 * @brief 正运动学解算 (Joints -> Pose)
 *        使用参考源码的 DH 旋转矩阵 + 显式连杆向量求位置
 */
bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle, const Joint6D_t *inputJoints, Pose6D_t *outputPose)
{
    float q[6];
    float R[6][9];
    float R02[9], R03[9], R04[9], R05[9], R06[9];
    float L0_bs[3], L0_se[3], L0_ew[3], L0_wt[3];
    float euler[3];

    for (int i = 0; i < 6; i++)
    {
        q[i] = inputJoints->a[i] * DEG_TO_RAD_CONST + handle->DH_matrix[i][0];

        float cosq = cosf(q[i]);
        float sinq = sinf(q[i]);
        float cosa = cosf(handle->DH_matrix[i][3]);
        float sina = sinf(handle->DH_matrix[i][3]);

        R[i][0] = cosq;
        R[i][1] = -cosa * sinq;
        R[i][2] = sina * sinq;
        R[i][3] = sinq;
        R[i][4] = cosa * cosq;
        R[i][5] = -sina * cosq;
        R[i][6] = 0.0f;
        R[i][7] = sina;
        R[i][8] = cosa;
    }

    MatMultiply(R[0], R[1], R02, 3, 3, 3);
    MatMultiply(R02, R[2], R03, 3, 3, 3);
    MatMultiply(R03, R[3], R04, 3, 3, 3);
    MatMultiply(R04, R[4], R05, 3, 3, 3);
    MatMultiply(R05, R[5], R06, 3, 3, 3);

    MatMultiply(R[0], handle->L1_base, L0_bs, 3, 3, 1);
    MatMultiply(R02, handle->L2_arm, L0_se, 3, 3, 1);
    MatMultiply(R03, handle->L3_elbow, L0_ew, 3, 3, 1);
    MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);

    outputPose->X = L0_bs[0] + L0_se[0] + L0_ew[0] + L0_wt[0];
    outputPose->Y = L0_bs[1] + L0_se[1] + L0_ew[1] + L0_wt[1];
    outputPose->Z = L0_bs[2] + L0_se[2] + L0_ew[2] + L0_wt[2];

    memcpy(outputPose->R, R06, 9 * sizeof(float));
    outputPose->hasR = true;

    RotMatToEulerAngle(R06, euler);
    outputPose->A = euler[0] * RAD_TO_DEG_CONST;
    outputPose->B = euler[1] * RAD_TO_DEG_CONST;
    outputPose->C = euler[2] * RAD_TO_DEG_CONST;

    return true;
}

/**
 * @brief 逆运动学解算 (Pose -> Joints)
 *        按 Dummy 源码注释/6dof_kinematic.cpp 的几何法移植，内部单位为 mm
 */
bool Kinematic_SolveIK(const DOF6Kinematic_Handle_t *handle,
                       const Pose6D_t *inputPose,
                       const Joint6D_t *lastJoints,
                       IKSolves_t *outputSolves)
{
    float qs[2];
    float qa[2][2];
    float qw[2][3];
    float cosqs, sinqs;
    float cosqa[2], sinqa[2];
    float cosqw, sinqw;
    float P06[6];
    float R06[9];
    float P0_w[3];
    float P1_w[3];
    float L0_wt[3];
    float L1_sw[3];
    float R10[9];
    float R31[9];
    float R30[9];
    float R36[9];
    float l_sw_2, l_sw, atan_a, acos_a, acos_e;
    float last_rad[6];
    bool has_candidate = false;

    memset(outputSolves, 0, sizeof(IKSolves_t));

    for (int i = 0; i < 6; i++)
        last_rad[i] = lastJoints->a[i] * DEG_TO_RAD_CONST;

    P06[0] = inputPose->X;
    P06[1] = inputPose->Y;
    P06[2] = inputPose->Z;
    if (!inputPose->hasR)
    {
        P06[3] = inputPose->A * DEG_TO_RAD_CONST;
        P06[4] = inputPose->B * DEG_TO_RAD_CONST;
        P06[5] = inputPose->C * DEG_TO_RAD_CONST;
        EulerAngleToRotMat(&(P06[3]), R06);
    }
    else
    {
        memcpy(R06, inputPose->R, 9 * sizeof(float));
    }

    MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);
    for (int i = 0; i < 3; i++)
        P0_w[i] = P06[i] - L0_wt[i];

    if (sqrtf(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= KIN_EPSILON)
    {
        qs[0] = last_rad[0];
        qs[1] = last_rad[0];
        for (int i = 0; i < 4; i++)
        {
            outputSolves->solFlag[i][0] = -1;
            outputSolves->solFlag[4 + i][0] = -1;
        }
    }
    else
    {
        qs[0] = atan2f(P0_w[1], P0_w[0]);
        qs[1] = atan2f(-P0_w[1], -P0_w[0]);
        for (int i = 0; i < 4; i++)
        {
            outputSolves->solFlag[i][0] = 1;
            outputSolves->solFlag[4 + i][0] = 1;
        }
    }

    for (int ind_arm = 0; ind_arm < 2; ind_arm++)
    {
        cosqs = cosf(qs[ind_arm] + handle->DH_matrix[0][0]);
        sinqs = sinf(qs[ind_arm] + handle->DH_matrix[0][0]);

        R10[0] = cosqs;
        R10[1] = sinqs;
        R10[2] = 0.0f;
        R10[3] = 0.0f;
        R10[4] = 0.0f;
        R10[5] = -1.0f;
        R10[6] = -sinqs;
        R10[7] = cosqs;
        R10[8] = 0.0f;

        MatMultiply(R10, P0_w, P1_w, 3, 3, 1);
        for (int i = 0; i < 3; i++)
            L1_sw[i] = P1_w[i] - handle->L1_base[i];

        l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        l_sw = sqrtf(l_sw_2);

        bool arm_valid = true;
        if ((l_sw > handle->l_se + handle->l_ew + KIN_EPSILON) ||
            (l_sw < fabsf(handle->l_se - handle->l_ew) - KIN_EPSILON) ||
            (l_sw <= KIN_EPSILON) || (handle->l_se <= KIN_EPSILON) || (handle->l_ew <= KIN_EPSILON))
        {
            arm_valid = false;
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
        }
        else if (fabsf(handle->l_se + handle->l_ew - l_sw) <= KIN_EPSILON)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
        }
        else if (fabsf(l_sw - fabsf(handle->l_se - handle->l_ew)) <= KIN_EPSILON)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            if (0 == ind_arm)
            {
                qa[0][1] = (float)M_PI;
                qa[1][1] = -(float)M_PI;
            }
            else
            {
                qa[0][1] = -(float)M_PI;
                qa[1][1] = (float)M_PI;
            }
        }
        else
        {
            atan_a = atan2f(L1_sw[1], L1_sw[0]);
            acos_a = ClampCos(0.5f * (handle->l_se_2 + l_sw_2 - handle->l_ew_2) / (handle->l_se * l_sw));
            acos_a = acosf(acos_a);
            acos_e = ClampCos(0.5f * (handle->l_se_2 + handle->l_ew_2 - l_sw_2) / (handle->l_se * handle->l_ew));
            acos_e = acosf(acos_e);
            if (0 == ind_arm)
            {
                qa[0][0] = atan_a - acos_a + (float)M_PI_2;
                qa[0][1] = handle->atan_e - acos_e + (float)M_PI;
                qa[1][0] = atan_a + acos_a + (float)M_PI_2;
                qa[1][1] = handle->atan_e + acos_e - (float)M_PI;
            }
            else
            {
                qa[0][0] = atan_a + acos_a + (float)M_PI_2;
                qa[0][1] = handle->atan_e + acos_e - (float)M_PI;
                qa[1][0] = atan_a - acos_a + (float)M_PI_2;
                qa[1][1] = handle->atan_e - acos_e + (float)M_PI;
            }
        }

        for (int i = 0; i < 2; i++)
        {
            outputSolves->solFlag[4 * ind_arm + i][1] = arm_valid ? 1 : 0;
            outputSolves->solFlag[4 * ind_arm + 2 + i][1] = arm_valid ? 1 : 0;
        }

        for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++)
        {
            cosqa[0] = cosf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
            sinqa[0] = sinf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
            cosqa[1] = cosf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);
            sinqa[1] = sinf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);

            R31[0] = cosqa[0] * cosqa[1] - sinqa[0] * sinqa[1];
            R31[1] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[2] = 0.0f;
            R31[3] = 0.0f;
            R31[4] = 0.0f;
            R31[5] = 1.0f;
            R31[6] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[7] = -cosqa[0] * cosqa[1] + sinqa[0] * sinqa[1];
            R31[8] = 0.0f;

            MatMultiply(R31, R10, R30, 3, 3, 3);
            MatMultiply(R30, R06, R36, 3, 3, 3);

            bool wrist_singular = false;
            if (R36[8] >= 1.0f - KIN_EPSILON)
            {
                cosqw = 1.0f;
                qw[0][1] = 0.0f;
                qw[1][1] = 0.0f;
                wrist_singular = true;
            }
            else if (R36[8] <= -1.0f + KIN_EPSILON)
            {
                cosqw = -1.0f;
                if (0 == ind_arm)
                {
                    qw[0][1] = (float)M_PI;
                    qw[1][1] = -(float)M_PI;
                }
                else
                {
                    qw[0][1] = -(float)M_PI;
                    qw[1][1] = (float)M_PI;
                }
                wrist_singular = true;
            }
            else
            {
                cosqw = ClampCos(R36[8]);
                if (0 == ind_arm)
                {
                    qw[0][1] = acosf(cosqw);
                    qw[1][1] = -acosf(cosqw);
                }
                else
                {
                    qw[0][1] = -acosf(cosqw);
                    qw[1][1] = acosf(cosqw);
                }
            }

            if (wrist_singular)
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = last_rad[3];
                    cosqw = cosf(last_rad[3] + handle->DH_matrix[3][0]);
                    sinqw = sinf(last_rad[3] + handle->DH_matrix[3][0]);
                    qw[0][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][2] = last_rad[5];
                    cosqw = cosf(last_rad[5] + handle->DH_matrix[5][0]);
                    sinqw = sinf(last_rad[5] + handle->DH_matrix[5][0]);
                    qw[1][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                }
                else
                {
                    qw[0][2] = last_rad[5];
                    cosqw = cosf(last_rad[5] + handle->DH_matrix[5][0]);
                    sinqw = sinf(last_rad[5] + handle->DH_matrix[5][0]);
                    qw[0][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][0] = last_rad[3];
                    cosqw = cosf(last_rad[3] + handle->DH_matrix[3][0]);
                    sinqw = sinf(last_rad[3] + handle->DH_matrix[3][0]);
                    qw[1][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                }
                outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow][2] = -1;
                outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            }
            else
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = atan2f(R36[5], R36[2]);
                    qw[1][0] = atan2f(-R36[5], -R36[2]);
                    qw[0][2] = atan2f(R36[7], -R36[6]);
                    qw[1][2] = atan2f(-R36[7], R36[6]);
                }
                else
                {
                    qw[0][0] = atan2f(-R36[5], -R36[2]);
                    qw[1][0] = atan2f(R36[5], R36[2]);
                    qw[0][2] = atan2f(-R36[7], R36[6]);
                    qw[1][2] = atan2f(R36[7], -R36[6]);
                }
                outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow][2] = 1;
                outputSolves->solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }

            for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++)
            {
                int idx = 4 * ind_arm + 2 * ind_elbow + ind_wrist;
                outputSolves->config[idx].a[0] = NormalizeRad(qs[ind_arm]) * RAD_TO_DEG_CONST;
                outputSolves->config[idx].a[1] = NormalizeRad(qa[ind_elbow][0]) * RAD_TO_DEG_CONST;
                outputSolves->config[idx].a[2] = NormalizeRad(qa[ind_elbow][1]) * RAD_TO_DEG_CONST;
                outputSolves->config[idx].a[3] = NormalizeRad(qw[ind_wrist][0]) * RAD_TO_DEG_CONST;
                outputSolves->config[idx].a[4] = NormalizeRad(qw[ind_wrist][1]) * RAD_TO_DEG_CONST;
                outputSolves->config[idx].a[5] = NormalizeRad(qw[ind_wrist][2]) * RAD_TO_DEG_CONST;

                if (outputSolves->solFlag[idx][0] != 0 && outputSolves->solFlag[idx][1] != 0)
                    has_candidate = true;
            }
        }
    }

    return has_candidate;
}

/**
 * @brief 从8组解中挑选限位内、离当前关节角最近的一组
 */
int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle,
                              const IKSolves_t *solves,
                              const Joint6D_t *current_joints)
{
    int best_index = -1;
    float best_score = FLT_MAX;
    static const float joint_weight[6] = {
        8.0f,
        8.0f,
        4.0f,
        1.5f,
        1.0f,
        0.8f,
    };

    for (int i = 0; i < 8; i++)
    {
        if (solves->solFlag[i][0] == 0 || solves->solFlag[i][1] == 0 || solves->solFlag[i][2] == 0)
            continue;

        bool in_limit = true;
        for (int j = 0; j < 6; j++)
        {
            float angle = NormalizeDeg(solves->config[i].a[j]);
            if (angle < handle->config.joint_min_limit[j] || angle > handle->config.joint_max_limit[j])
            {
                in_limit = false;
                break;
            }
        }
        if (!in_limit)
            continue;

        float score = 0.0f;
        for (int j = 0; j < 6; j++)
        {
            float diff = AngleDiffDeg(solves->config[i].a[j], current_joints->a[j]);
            score += joint_weight[j] * diff * diff;
        }

        if (solves->solFlag[i][2] < 0)
            score += 1000000.0f;

        if (score < best_score)
        {
            best_score = score;
            best_index = i;
        }
    }

    return best_index;
}

/**
 * @brief 获取所有电机角度并填入 Joint_Angle_State_t 结构体
 * @param[out] joint_angle 输出的目标结构体指针
 */
void Joint_Motor_Get_All_Angles_And_State(Joint_Angle_State_t *joint_angle)
{
    if (joint_angle == NULL)
        return;

    for (uint8_t i = 0; i < 6; i++)
    {
        float angle = Joint_Motor_Get_Angle(i + 1);
        uint8_t state = Joint_Motor_Get_State(i + 1);
        joint_angle->angle[i] = angle;
        joint_angle->state[i] = state;
    }
}
