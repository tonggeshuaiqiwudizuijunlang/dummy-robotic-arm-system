#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H

#include "stm32f405xx.h"
#include "arm_math.h"
#include "memory.h"

class DOF6Kinematic
//这段代码定义了一个名为 DOF6Kinematic 的类，表示一个六自由度机械臂的运动学模型。以下是该类的主要部分的解释：
{
private:
//这标志着以下的成员变量和方法都是私有的，只能在类的内部访问。
    const float RAD_TO_DEG = 57.295777754771045f;
//定义一个常量，用于将弧度转换为角度的比例因子。
    // DH parameters
    struct ArmConfig_t
    // 这是一个结构体类型，用于存储机械臂的 DH 参数
    {
        float L_BASE;
        //机械臂基座的长度。
        float D_BASE;
        //机械臂基座到第一关节的垂直距离。
        float L_ARM;
        //: 第一关节到第二关节的长度
        float L_FOREARM;
        //第二关节到第三关节的长度
        float D_ELBOW;
        //第三关节到第四关节的垂直距离
        float L_WRIST;
        //第四关节到末端执行器的长度
    };
    ArmConfig_t armConfig;

    float DH_matrix[6][4] = {0}; // home,d,a,alpha
    //存储 DH 参数的矩阵。矩阵的每一行代表一个关节，包含了该关节的 home、d、a、alpha 四个参数
    float L1_base[3] = {0};
    float L2_arm[3] = {0};
    float L3_elbow[3] = {0};
    float L6_wrist[3] = {0};
    //float L1_base[3] = {0};, float L2_arm[3] = {0};, float L3_elbow[3] = {0};, float L6_wrist[3] = {0};: 
    //分别存储基座、第一关节、第三关节（肘部）和第六关节（腕部）的位置坐标。

    float l_se_2;
    float l_se;
    float l_ew_2;
    float l_ew;
    float atan_e;
    //一些用于计算的额外变量，可能涉及到臂的长度和角度。
   //这段代码定义了机械臂的运动学参数和相关的变量。类中可能还有其他的成员函数和方法，用于计算正运动学、逆运动学等操作。

public:
//这一部分是 DOF6Kinematic 类的 public 部分，其中包括两个嵌套的 struct 定义，分别是 Joint6D_t 和 Pose6D_t。
    struct Joint6D_t
    //Joint6D_t结构体表示六个关节的角度信息
    {
        Joint6D_t()
        = default;

        Joint6D_t(float a1, float a2, float a3, float a4, float a5, float a6)
            : a{a1, a2, a3, a4, a5, a6}
        {}
        // 通过给定的六个角度值构造对象

        float a[6];
        // 存储六个关节的角度值

        friend Joint6D_t operator-(const Joint6D_t &_joints1, const Joint6D_t &_joints2);
    };
    //通过重载 - 运算符，实现两个 Joint6D_t 对象的减法操作。这个函数返回一个新的 Joint6D_t 对象，其每个关节的值等于第一个对象的对应关节减去第二个对象的对应关节。

    struct Pose6D_t
    //Pose6D_t 结构体表示六自由度机械臂的姿态信息：
    {
        Pose6D_t()
        = default;
        //默认构造函数。

        Pose6D_t(float x, float y, float z, float a, float b, float c)
            : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false)
        {}
        //通过给定的位置坐标和欧拉角构造对象。hasR 初始化为 false，表示尚未计算旋转矩阵。

        float X{}, Y{}, Z{};
        //: 三维空间中的位置坐标。
        float A{}, B{}, C{};
        //三个欧拉角，分别表示绕 X、Y、Z 轴的旋转。
        float R[9]{};
        //三维旋转矩阵

        // if Pose was calculated by FK then it's true automatically (so that no need to do extra calc),
        // otherwise if manually set params then it should be set to false.
        bool hasR{};
        //一个布尔值，用于标识旋转矩阵是否已经计算。如果通过正运动学计算得到姿态，则此值为 true；如果手动设置参数，则应将其设置为 false。
    };

    struct IKSolves_t
    //这是 DOF6Kinematic 类的剩余部分，包括 IKSolves_t 结构体和类的构造函数以及两个函数的声明。
    //struct IKSolves_t: 这个结构体用于存储逆运动学求解的结果
    {
        Joint6D_t config[8];    
        //存储八个可能的关节角度配置，每个配置由一个 Joint6D_t 对象表示。
        char solFlag[8][3];
    };
    // 每个配置的解标志，用于表示是否有解。solFlag[i][0] 表示第 i 个配置是否有解，solFlag[i][1] 和 solFlag[i][2] 可以用于其他信息。

    DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);
    //DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);: 构造函数，用于初始化 DOF6Kinematic 类的实例。接受六个参数，分别表示机械臂的 DH 参数。

//DH参数（Denavit-Hartenberg参数）是一种用于描述刚体连接关节之间几何关系的参数体系。它是由Jacques Denavit和Richard S. Hartenberg提出的，用于建模和分析机械臂或其他多关节系统的运动学链。

//在DH参数中，每个关节都被描述为一个坐标系的转换过程，包括以下四个参数：

//连杆长度（a）： 表示前一关节与当前关节之间的连杆长度。这是沿着前一关节z轴测量的。

//连杆偏移（d）： 表示前一关节与当前关节之间的连杆偏移，即前一关节z轴到当前关节z轴之间的距离。这是沿着前一关节x轴测量的。

//关节角度（θ）： 表示前一关节z轴绕着z轴的旋转角度，即绕着z轴的旋转。

//关节旋转（α）： 表示当前关节x轴绕着z轴的旋转角度，即绕着z轴的旋转。

//这四个参数定义了相邻两个关节之间的坐标系转换。使用DH参数，可以通过组合这些转换来建立整个机械臂的运动学模型。

//DH参数的使用简化了机械臂运动学的描述和计算，使得可以更方便地进行运动学分析和控制。


    bool SolveFK(const Joint6D_t &_inputJoint6D, Pose6D_t &_outputPose6D);
    //: 正运动学求解函数，用于根据给定的关节角度计算末端执行器的姿态。

    bool SolveIK(const Pose6D_t &_inputPose6D, const Joint6D_t &_lastJoint6D, IKSolves_t &_outputSolves);
};
// 逆运动学求解函数，用于根据给定的姿态计算关节角度配置。同时，通过 _lastJoint6D 参数，可以提供上一个关节角度配置，以帮助解决多解的情况。

#endif //DOF6_KINEMATIC_SOLVER_H
//这些函数的具体实现可能在类的实现文件中。
