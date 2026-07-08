#include "6dof_kinematic.h"
// 定义内联函数，使用 CMSIS-DSP 库的三角函数
inline float cosf(float x)
//inline float cosf(float x)：这个函数是计算输入角度 x 的余弦值。
//arm_cos_f32 是 CMSIS-DSP 库中用于计算单精度浮点数余弦的函数。
//通过将 x 传递给 arm_cos_f32，得到余弦值，并将其作为返回值返回。
{ 
    return arm_cos_f32(x);
    
}

inline float sinf(float x)
//inline float sinf(float x)：
    //这个函数是计算输入角度 x 的正弦值。
    //arm_sin_f32 是 CMSIS-DSP 库中用于计算单精度浮点数正弦的函数。
    //通过将 x 传递给 arm_sin_f32，得到正弦值，并将其作为返回值返回。
{
    return arm_sin_f32(x);
}
// 矩阵相乘函数,用于计算两个矩阵相乘的结果。
//函数的参数包括两个输入矩阵 _matrix1 和 _matrix2，
//以及一个输出矩阵(即勾股定理)
static void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,
                        const int _m, const int _l, const int _n)
//_matrix1 和 _matrix2 是待相乘的两个输入矩阵，分别为 _m x _l 和 _l x _n 大小。
//_matrixOut 是存储矩阵乘法结果的输出矩阵，大小为 _m x _n。
//_m 是矩阵 _matrix1 的行数，_l 是两个矩阵的公共维度，即 
//_matrix1 的列数和 _matrix2 的行数，_n 是矩阵 _matrix2 的列数。
{//函数通过三重嵌套的循环遍历矩阵的每个元素，
//按照矩阵相乘的规则逐个计算输出矩阵 _matrixOut 的每个元素的值。
//具体的计算过程如下：
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    //外层循环 for (i = 0; i < _m; i++)：遍历结果矩阵的行。
    {
        for (j = 0; j < _n; j++)
    //中层循环 for (j = 0; j < _n; j++)：遍历结果矩阵的列。
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }//内层循环 for (k = 0; k < _l; k++)：遍历两个输入矩阵的公共维度，
        //计算相应位置的元素相乘并累加到临时变量 tmp 中。
        //将计算得到的 tmp 赋值给输出矩阵 _matrixOut 的相应位置.
    }
}
//旋转矩阵转换为欧拉角
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles)
//这里的欧拉角按照 ZYX 顺序（绕 Z、Y、X 轴）存储在 _eulerAngles 数组中。
{
    float A, B, C, cb;

    if (fabs(_rotationM[6]) >= 1.0 - 0.0001)
    {
        if (_rotationM[6] < 0)
        {
            A = 0.0f;
            B = (float) M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else
        {
            A = 0.0f;
            B = -(float) M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else
    {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = C;
    _eulerAngles[1] = B;
    _eulerAngles[2] = A;
    //首先，检查旋转矩阵的第 6 个元素 _rotationM[6] 的绝对值是否接近 1.0。
    //这是因为当 _rotationM[6] 接近 ±1 时，说明旋转矩阵对应的旋转角度使得绕 X 轴的旋转接近 ±90 度。在这种情况下，
    //可以通过 _rotationM[6] 的正负性来确定绕 Y 轴的旋转角度。

   //如果条件成立，根据 _rotationM[6] 的正负性确定角度 A 和 B。
   //这种情况下，角度 C 可以通过 atan2f(_rotationM[1], _rotationM[4])
   //或 -atan2f(_rotationM[1], _rotationM[4]) 来计算，具体取决于 _rotationM[6] 的正负。

   //如果条件不成立，表示旋转矩阵不是绕 X 轴旋转 ±90 度。计算角度 B，然后计算角度 A 和 C。
   //这里使用 atan2f 函数是为了处理所有四个象限的情况，确保计算结果的准确性。

  //最后，将计算得到的欧拉角存储在 _eulerAngles 数组中，顺序为 ZYX。

  //总体而言，这个函数是一个将旋转矩阵转换为欧拉角的通用方法，可以处理绕 X 轴旋转 ±90 度的特殊情况。
}
// 欧拉角转换为旋转矩阵
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM)
{
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[0]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[2]);
    sc = sinf(_eulerAngles[0]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[2]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
    //这段代码实现了将欧拉角转换为旋转矩阵的功能。给定欧拉角数组 
    //_eulerAngles，将计算得到的旋转矩阵存储在 _rotationM 数组中
    //这里使用了三个角度分量 _eulerAngles[0]、_eulerAngles[1]、
    //_eulerAngles[2]，它们分别代表绕 Z、Y、X 轴的旋转角度。
    //现在，让我解释一下这个函数的逻辑：
    //1.计算欧拉角中每个角度的正弦和余弦值。
    //ca、cb、cc 分别是欧拉角的三个分量对应的余弦值。
    //sa、sb、sc 分别是欧拉角的三个分量对应的正弦值。
    //2.基于欧拉角的正弦和余弦值，计算旋转矩阵的每个元素。
    //_rotationM[0] 到 _rotationM[8] 分别表示旋转矩阵的每个元素
    //使用欧拉角的正弦和余弦值计算矩阵元素，具体表达式是通过角度转换矩阵计算得到的
    //将计算得到的旋转矩阵存储在 _rotationM 数组中。
    //总体而言，这个函数实现了欧拉角到旋转矩阵的转换，是一个常见的姿态表示方法之一。
}

// 机械臂运动学类的定义
DOF6Kinematic::DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT)
    : armConfig(ArmConfig_t{L_BS, D_BS, L_AM, L_FA, D_EW, L_WT})
  
    float tmp_DH_matrix[6][4] = {
        {0.0f,            armConfig.L_BASE,    armConfig.D_BASE, -(float) M_PI_2},
        {-(float) M_PI_2, 0.0f,                armConfig.L_ARM,  0.0f},
        {(float) M_PI_2,  armConfig.D_ELBOW,   0.0f,             (float) M_PI_2},
        {0.0f,            armConfig.L_FOREARM, 0.0f,             -(float) M_PI_2},
        {0.0f,            0.0f,                0.0f,             (float) M_PI_2},
        {0.0f,            armConfig.L_WRIST, 0.0f, 0.0f}


        //| θi    di      ai       αi      |
          |---------------------------------|
          | 0     L_BASE  D_BASE  -π/2     |
          | -π/2  0       L_ARM   0        |
          | π/2   D_ELBOW 0       π/2      |
          | 0     L_FOREARM 0    -π/2     |
          | 0     0       0       π/2      |
          | 0     L_WRIST 0      0         |

        //θi：关节角度（Joint angle）
        //di：连接两个关节的平移距离（Link offset）
        //ai：前一关节到当前关节的长度（Link length）
        //αi：前一关节绕Z轴的旋转角度（Link twist）

        //第一行： 描述基座到第一个关节的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离。
        //d：关节的x轴轴线与前一个关节x轴的平行距离。
        //θ：绕z轴旋转的角度。
        //α：绕x轴旋转的角度，这里是固定的值 -π/2

        //第二行： 描述第一个到第二个关节的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离，这里是固定的值 0.0。
        //d：关节的x轴轴线与前一个关节x轴的平行距离。
        //θ：绕z轴旋转的角度，这里是固定的值 -π/2。
        //α：绕x轴旋转的角度，这里是固定的值 0.0
        
        //第三行： 描述第二个到第三个关节的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离。
        //d：关节的x轴轴线与前一个关节x轴的平行距离，这里是固定的值 0.0。
        //θ：绕z轴旋转的角度，这里是固定的值 π/2。
        //α：绕x轴旋转的角度，这里是固定的值 -π/2

        //第四行： 描述第三个到第四个关节的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离，这里是固定的值 0.0。
        //d：关节的x轴轴线与前一个关节x轴的平行距离。
        //θ：绕z轴旋转的角度。
        //α：绕x轴旋转的角度，这里是固定的值 -π/2

        //第五行： 描述第四个到第五个关节的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离，这里是固定的值 0.0。
        //d：关节的x轴轴线与前一个关节x轴的平行距离，这里是固定的值 0.0。
        //θ：绕z轴旋转的角度，这里是固定的值 0.0。
        //α：绕x轴旋转的角度，这里是固定的值 π/2\

        //第六行： 描述第五个到末端执行器的转换。参数包括：
        //a：关节的z轴轴线与前一个关节z轴的垂直距离，这里是固定的值 0.0。
        //d：关节的x轴轴线与前一个关节x轴的平行距离。
        //θ：绕z轴旋转的角度，这里是固定的值 0.0。
        //α：绕x轴旋转的角度，这里是固定的值 0.0
        //
        //
        //
        //
        //
        //

    };
    
    memcpy(DH_matrix, tmp_DH_matrix, sizeof(tmp_DH_matrix));
    //将定义好的DH矩阵 tmp_DH_matrix 复制到程序中使用的DH矩阵 

    float tmp_L1_bs[3] = {armConfig.D_BASE, -armConfig.L_BASE, 0.0f};
    memcpy(L1_base, tmp_L1_bs, sizeof(tmp_L1_bs));
    //创建一个长度为3的数组 tmp_L1_bs，表示基座到第一个关节的位移。然后通过 memcpy 将这个数组的值复制到 L1_base 中。
    float tmp_L2_se[3] = {armConfig.L_ARM, 0.0f, 0.0f};
    memcpy(L2_arm, tmp_L2_se, sizeof(tmp_L2_se));
    //创建一个长度为3的数组 tmp_L2_se，表示第一关节到第二关节的位移。然后通过 memcpy 将这个数组的值复制到 L2_arm 中。
    float tmp_L3_ew[3] = {-armConfig.D_ELBOW, 0.0f, armConfig.L_FOREARM};
    memcpy(L3_elbow, tmp_L3_ew, sizeof(tmp_L3_ew));
    //创建一个长度为3的数组 tmp_L3_ew，表示第二关节到第三关节的位移。然后通过 memcpy 将这个数组的值复制到 L3_elbow 中。
    float tmp_L6_wt[3] = {0.0f, 0.0f, armConfig.L_WRIST};
    memcpy(L6_wrist, tmp_L6_wt, sizeof(tmp_L6_wt));
    //创建一个长度为3的数组 tmp_L6_wt，表示第五关节到末端执行器（末端执行器位置）的位移。然后通过 memcpy 将这个数组的值复制到 L6_wrist 

    l_se_2 = armConfig.L_ARM * armConfig.L_ARM;
    //计算并保存 armConfig.L_ARM 的平方到 l_se_2。
    l_se = armConfig.L_ARM;
    //保存 armConfig.L_ARM 的值到 l_se。
    l_ew_2 = armConfig.L_FOREARM * armConfig.L_FOREARM + armConfig.D_ELBOW * armConfig.D_ELBOW;
    //计算并保存 armConfig.L_FOREARM 和 armConfig.D_ELBOW 平方和到 l_ew_2
    l_ew = 0;
    //将 l_ew 初始化为0
    atan_e = 0;
    //将 atan_e 初始化为0
}

bool
DOF6Kinematic::SolveFK(const DOF6Kinematic::Joint6D_t &_inputJoint6D, DOF6Kinematic::Pose6D_t &_outputPose6D)
{//这段代码是机械臂正运动学（Forward Kinematics）的求解函数，
//用于将关节角度转换为末端执行器的位姿。以下是对代码的详细解释：
    float q_in[6];
    //定义一个数组，用于存储输入的6个关节角度。
    float q[6];
    //：定义一个数组，用于存储经过修正后的关节角度。
    float cosq, sinq;
    //定义两个变量，用于存储关节角的余弦和正弦值。
    float cosa, sina;
    //：定义两个变量，用于存储DH矩阵中的角度的余弦和正弦值。
    float P06[6];
    //定义一个数组，用于存储机械臂末端执行器的位置和姿态。
    float R06[9];
    //定义一个数组，用于存储机械臂末端执行器的旋转矩阵。
    float R[6][9];
    //定义一个数组，用于存储每个关节的旋转矩阵。
    float R02[9];
    float R03[9];
    float R04[9];
    float R05[9];
    //定义数组，用于存储每个关节相对于基座坐标系的旋转矩阵。
    float L0_bs[3];
    float L0_se[3];
    float L0_ew[3];
    float L0_wt[3];
    //定义数组，用于存储基座、第一关节、第二关节和末端执行器相对于基座坐标系的位移。

    for (int i = 0; i < 6; i++)
        q_in[i] = _inputJoint6D.a[i] / RAD_TO_DEG;

    for (int i = 0; i < 6; i++)
    {
        q[i] = q_in[i] + DH_matrix[i][0];
        cosq = cosf(q[i]);
        sinq = sinf(q[i]);
        cosa = cosf(DH_matrix[i][3]);
        sina = sinf(DH_matrix[i][3]);

        R[i][0] = cosq;
        R[i][1] = -cosa * sinq;
        R[i][2] = sina * sinq;
        R[i][3] = sinq;
        R[i][4] = cosa * cosq;
        R[i][5] = -sina * cosq;
        R[i][6] = 0.0f;
        R[i][7] = sina;
        R[i][8] = cosa;
        //遍历每个关节，计算修正后的关节角度，以及每个关节的旋转矩阵。
    }

    MatMultiply(R[0], R[1], R02, 3, 3, 3);
    MatMultiply(R02, R[2], R03, 3, 3, 3);
    MatMultiply(R03, R[3], R04, 3, 3, 3);
    MatMultiply(R04, R[4], R05, 3, 3, 3);
    MatMultiply(R05, R[5], R06, 3, 3, 3);
    //使用 MatMultiply 函数计算相邻关节之间的旋转矩阵，最终得到机械臂末端执行器的旋转矩阵 R06。

    MatMultiply(R[0], L1_base, L0_bs, 3, 3, 1);
    MatMultiply(R02, L2_arm, L0_se, 3, 3, 1);
    MatMultiply(R03, L3_elbow, L0_ew, 3, 3, 1);
    MatMultiply(R06, L6_wrist, L0_wt, 3, 3, 1);
    //使用 MatMultiply 函数计算基座、第一关节、第二关节和末端执行器相对于基座坐标系的位移。

    for (int i = 0; i < 3; i++)
        P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];

    RotMatToEulerAngle(R06, &(P06[3]));

    _outputPose6D.X = P06[0];
    _outputPose6D.Y = P06[1];
    _outputPose6D.Z = P06[2];
    _outputPose6D.A = P06[3] * RAD_TO_DEG;
    _outputPose6D.B = P06[4] * RAD_TO_DEG;
    _outputPose6D.C = P06[5] * RAD_TO_DEG;
    //将位移和旋转矩阵转换为欧拉角，存储到数组 P06 中。
    memcpy(_outputPose6D.R, R06, 9 * sizeof(float));
    //将计算得到的位姿信息分别赋值给 _outputPose6D 结构体的成员变量

    return true;
    //返回 true 表示正运动学求解成功。
}

bool DOF6Kinematic::SolveIK(const DOF6Kinematic::Pose6D_t &_inputPose6D, const Joint6D_t &_lastJoint6D,
                            DOF6Kinematic::IKSolves_t &_outputSolves)
{//逆解
//// 定义变量
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

    int ind_arm, ind_elbow, ind_wrist;
    int i;

    if (0 == l_ew)
    {
        l_ew = sqrtf(l_ew_2);
        atan_e = atanf(armConfig.D_ELBOW / armConfig.L_FOREARM);
    }

    P06[0] = _inputPose6D.X / 1000.0f;
    P06[1] = _inputPose6D.Y / 1000.0f;
    P06[2] = _inputPose6D.Z / 1000.0f;
    if (!_inputPose6D.hasR)
    {
        P06[3] = _inputPose6D.A / RAD_TO_DEG;
        P06[4] = _inputPose6D.B / RAD_TO_DEG;
        P06[5] = _inputPose6D.C / RAD_TO_DEG;
        EulerAngleToRotMat(&(P06[3]), R06);
    } else
    {
        memcpy(R06, _inputPose6D.R, 9 * sizeof(float));
    }
    for (i = 0; i < 2; i++)
    {
        qs[i] = _lastJoint6D.a[0];
        qa[i][0] = _lastJoint6D.a[1];
        qa[i][1] = _lastJoint6D.a[2];
        qw[i][0] = _lastJoint6D.a[3];
        qw[i][1] = _lastJoint6D.a[4];
        qw[i][2] = _lastJoint6D.a[5];
    }
    MatMultiply(R06, L6_wrist, L0_wt, 3, 3, 1);
    for (i = 0; i < 3; i++)
    {
        P0_w[i] = P06[i] - L0_wt[i];
    }
    if (sqrt(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= 0.000001)
    {
        qs[0] = _lastJoint6D.a[0];
        qs[1] = _lastJoint6D.a[0];
        for (i = 0; i < 4; i++)
        {
            _outputSolves.solFlag[0 + i][0] = -1;
            _outputSolves.solFlag[4 + i][0] = -1;
        }
    } else
    {
        qs[0] = atan2f(P0_w[1], P0_w[0]);
        qs[1] = atan2f(-P0_w[1], -P0_w[0]);
        for (i = 0; i < 4; i++)
        {
            _outputSolves.solFlag[0 + i][0] = 1;
            _outputSolves.solFlag[4 + i][0] = 1;
        }
    }
    for (ind_arm = 0; ind_arm < 2; ind_arm++)
    {
        cosqs = cosf(qs[ind_arm] + DH_matrix[0][0]);
        sinqs = sinf(qs[ind_arm] + DH_matrix[0][0]);

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
        for (i = 0; i < 3; i++)
        {
            L1_sw[i] = P1_w[i] - L1_base[i];
        }
        l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        l_sw = sqrtf(l_sw_2);

        if (fabs(l_se + l_ew - l_sw) <= 0.000001)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            qa[0][1] = 0.0f;
            qa[1][1] = 0.0f;
            if (l_sw > l_se + l_ew)
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 0;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else if (fabs(l_sw - fabs(l_se - l_ew)) <= 0.000001)
        {
            qa[0][0] = atan2f(L1_sw[1], L1_sw[0]);
            qa[1][0] = qa[0][0];
            if (0 == ind_arm)
            {
                qa[0][1] = (float) M_PI;
                qa[1][1] = -(float) M_PI;
            } else
            {
                qa[0][1] = -(float) M_PI;
                qa[1][1] = (float) M_PI;
            }
            if (l_sw < fabs(l_se - l_ew))
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 0;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 0;
                }
            } else
            {
                for (i = 0; i < 2; i++)
                {
                    _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                    _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
                }
            }
        } else
        {
            atan_a = atan2f(L1_sw[1], L1_sw[0]);
            acos_a = 0.5f * (l_se_2 + l_sw_2 - l_ew_2) / (l_se * l_sw);
            if (acos_a >= 1.0f) acos_a = 0.0f;
            else if (acos_a <= -1.0f) acos_a = (float) M_PI;
            else acos_a = acosf(acos_a);
            acos_e = 0.5f * (l_se_2 + l_ew_2 - l_sw_2) / (l_se * l_ew);
            if (acos_e >= 1.0f) acos_e = 0.0f;
            else if (acos_e <= -1.0f) acos_e = (float) M_PI;
            else acos_e = acosf(acos_e);
            if (0 == ind_arm)
            {
                qa[0][0] = atan_a - acos_a + (float) M_PI_2;
                qa[0][1] = atan_e - acos_e + (float) M_PI;
                qa[1][0] = atan_a + acos_a + (float) M_PI_2;
                qa[1][1] = atan_e + acos_e - (float) M_PI;

            } else
            {
                qa[0][0] = atan_a + acos_a + (float) M_PI_2;
                qa[0][1] = atan_e + acos_e - (float) M_PI;
                qa[1][0] = atan_a - acos_a + (float) M_PI_2;
                qa[1][1] = atan_e - acos_e + (float) M_PI;
            }
            for (i = 0; i < 2; i++)
            {
                _outputSolves.solFlag[4 * ind_arm + 0 + i][1] = 1;
                _outputSolves.solFlag[4 * ind_arm + 2 + i][1] = 1;
            }
        }
        for (ind_elbow = 0; ind_elbow < 2; ind_elbow++)
        {
            cosqa[0] = cosf(qa[ind_elbow][0] + DH_matrix[1][0]);
            sinqa[0] = sinf(qa[ind_elbow][0] + DH_matrix[1][0]);
            cosqa[1] = cosf(qa[ind_elbow][1] + DH_matrix[2][0]);
            sinqa[1] = sinf(qa[ind_elbow][1] + DH_matrix[2][0]);

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

            if (R36[8] >= 1.0 - 0.000001)
            {
                cosqw = 1.0f;
                qw[0][1] = 0.0f;
                qw[1][1] = 0.0f;
            } else if (R36[8] <= -1.0 + 0.000001)
            {
                cosqw = -1.0f;
                if (0 == ind_arm)
                {
                    qw[0][1] = (float) M_PI;
                    qw[1][1] = -(float) M_PI;
                } else
                {
                    qw[0][1] = -(float) M_PI;
                    qw[1][1] = (float) M_PI;
                }
            } else
            {
                cosqw = R36[8];
                if (0 == ind_arm)
                {
                    qw[0][1] = acosf(cosqw);
                    qw[1][1] = -acosf(cosqw);
                } else
                {
                    qw[0][1] = -acosf(cosqw);
                    qw[1][1] = acosf(cosqw);
                }
            }
            if (1.0f == cosqw || -1.0f == cosqw)
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = _lastJoint6D.a[3];
                    cosqw = cosf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    qw[0][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][2] = _lastJoint6D.a[5];
                    cosqw = cosf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    qw[1][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                } else
                {
                    qw[0][2] = _lastJoint6D.a[5];
                    cosqw = cosf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    sinqw = sinf(_lastJoint6D.a[5] + DH_matrix[5][0]);
                    qw[0][0] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                    qw[1][0] = _lastJoint6D.a[3];
                    cosqw = cosf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    sinqw = sinf(_lastJoint6D.a[3] + DH_matrix[3][0]);
                    qw[1][2] = atan2f(cosqw * R36[3] - sinqw * R36[0], cosqw * R36[0] + sinqw * R36[3]);
                }
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
            } else
            {
                if (0 == ind_arm)
                {
                    qw[0][0] = atan2f(R36[5], R36[2]);
                    qw[1][0] = atan2f(-R36[5], -R36[2]);
                    qw[0][2] = atan2f(R36[7], -R36[6]);
                    qw[1][2] = atan2f(-R36[7], R36[6]);
                } else
                {
                    qw[0][0] = atan2f(-R36[5], -R36[2]);
                    qw[1][0] = atan2f(R36[5], R36[2]);
                    qw[0][2] = atan2f(-R36[7], R36[6]);
                    qw[1][2] = atan2f(R36[7], -R36[6]);
                }
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
                _outputSolves.solFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
            }
            for (ind_wrist = 0; ind_wrist < 2; ind_wrist++)
            {
                if (qs[ind_arm] > (float) M_PI)
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] =
                        qs[ind_arm] - (float) M_PI;
                else if (qs[ind_arm] < -(float) M_PI)
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] =
                        qs[ind_arm] + (float) M_PI;
                else
                    _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[0] = qs[ind_arm];

                for (i = 0; i < 2; i++)
                {
                    if (qa[ind_elbow][i] > (float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                            qa[ind_elbow][i] - (float) M_PI;
                    else if (qa[ind_elbow][i] < -(float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                            qa[ind_elbow][i] + (float) M_PI;
                    else
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[1 + i] =
                            qa[ind_elbow][i];
                }

                for (i = 0; i < 3; i++)
                {
                    if (qw[ind_wrist][i] > (float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                            qw[ind_wrist][i] - (float) M_PI;
                    else if (qw[ind_wrist][i] < -(float) M_PI)
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                            qw[ind_wrist][i] + (float) M_PI;
                    else
                        _outputSolves.config[4 * ind_arm + 2 * ind_elbow + ind_wrist].a[3 + i] =
                            qw[ind_wrist][i];
                }
            }
        }
    }

    for (i = 0; i < 8; i++)
        for (float &j: _outputSolves.config[i].a)
            j *= RAD_TO_DEG;

    return true;
}

DOF6Kinematic::Joint6D_t
operator-(const DOF6Kinematic::Joint6D_t &_joints1, const DOF6Kinematic::Joint6D_t &_joints2)
{
    DOF6Kinematic::Joint6D_t tmp{};
    for (int i = 0; i < 6; i++)
        tmp.a[i] = _joints1.a[i] - _joints2.a[i];

    return tmp;
}
