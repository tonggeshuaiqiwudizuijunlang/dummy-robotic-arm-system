#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0
//在robot.h头文件中，你会看到类的声明，包括构造函数、析构函数、
//私有成员变量和其他公共和私有成员函数的声明。

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */
//current_limit：电流限制，以某个单位为基础（可能是安培）。
//acceleration：关节运动的加速度，以某个单位为基础（可能是度每秒平方）。
//dce_kp：位置控制器的比例增益。
//dce_kv：位置控制器的速度增益。
//dce_ki：位置控制器的积分增益。
//dce_kd：位置控制器的微分增益。
//然后，表格中每一行表示机器人的一个关节，具体如下：

//Joint1 到 Joint6：分别代表六个关节。
//对于每个关节，有相应的电流限制、加速度、比例


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;
    //nodeID：表示手的节点ID，默认为7。
    //maxCurrent：表示手的最大电流，默认为0.7。


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);
//接受一个CAN_HandleTypeDef类型的指针 _hcan 和一个uint8_t类型的节点ID _id 作为参数

    void SetAngle(float _angle);
    //SetAngle(float _angle)：设置手的角度。
    void SetMaxCurrent(float _val);
    //SetMaxCurrent(float _val)：设置手的最大电流。
    void SetEnable(bool _enable);
    //SetEnable(bool _enable)：设置手的使能状态


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
        //set_angle：通过调用 SetAngle 函数设置手的角度。
       //set_enable：通过调用 SetEnable 函数设置手的使能状态。
      //set_current_limit：通过调用 SetMaxCurrent 函数设置手的最大电流。
    }


private:
    CAN_HandleTypeDef* hcan;
    //hcan：指向 CAN_HandleTypeDef 结构体的指针，表示与手相关的CAN总线句柄。
    uint8_t canBuf[8];
    //canBuf：长度为8的字节数组，用于存储CAN消息的数据。
    CAN_TxHeaderTypeDef txHeader;
    //txHeader：CAN_TxHeaderTypeDef 类型的结构体，用于存储CAN消息的头部信息。
    float minAngle = 0;
    //minAngle：手的最小角度，默认为0
    float maxAngle = 45;
    //maxAngle：手的最大角度，默认为45。
};


class DummyRobot
{
public:
    // 构造函数
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    // 析构函数
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        //顺序运动到目标点的命令模式。
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        //可中断的目标点运动模式。
        COMMAND_CONTINUES_TRAJECTORY,
        //连续轨迹运动模式。
        COMMAND_MOTOR_TUNING
        //电机调试模式。
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }
        //TuningHelper 类包含用于调整的功能。
        //在 MakeProtocolDefinitions 函数中，它定义了两个通信协议函数：

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);
        //set_tuning_freq_amp：允许设置调整的频率和振幅。
       //"set_tuning_freq_amp"：字符串，表示协议函数的名称。
      //*this：表示该协议函数与当前对象（TuningHelper 的实例）相关联。
     //&TuningHelper::SetFreqAndAmp：表示协议函数要调用的成员函数，即 TuningHelper 类中的 SetFreqAndAmp 函数。
    //"freq", "amp"：字符串参数，用于标识在通信中要设置的频率和振幅。
   //set_tuning_flag：允许设置调整的标志。
//"set_tuning_flag"：字符串，表示协议函数的名称。
//*this：表示该协议函数与当前对象（TuningHelper 的实例）相关联。
//&TuningHelper::SetTuningFlag：表示协议函数要调用的成员函数，即 TuningHelper 类中的 SetTuningFlag 函数。
//"flag"：字符串参数，用于标识在通信中要设置的标志。

        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }
        //这样，通过这两个协议函数，可以通过通信设置调整的频率和振幅，以及调整的标志。
        //这提供了一种通过通信接口对 TuningHelper 的功能进行远程控制的方式。


    private:
        DummyRobot* context;
        //用于访问机器人的状态和方法。
        float time = 0;
        //记录调谐的时间，初始值为 0。
        uint8_t tuningFlag = 0;
        //调谐标志，用于控制调谐的特定行为
        float frequency = 1;
        float amplitude = 1;
        //frequency 和 amplitude：用于控制调谐的频率和振幅的参数
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -73, 180, 0, 0, 0};
    //机器人上电时的初始姿势，由六个关节的角度组成
    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    //关节的默认速度，以度每秒为单位。
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    //关节的默认基础加速度，由六个关节的加速度组成。
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    //关节的默认低速和高速加速度。这两个值用于定义一个范围，可以在实际应用中根据需要调整关节的加速度。
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;
    //DEFAULT_COMMAND_MODE：机器人的默认命令模式，
    //这里设置为 COMMAND_TARGET_POINT_INTERRUPTABLE。这个值定义了机器人在上电时执行的默认操作模式。


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    //当前关节角度，初始值为 REST_POSE，即机器人上电时的初始姿势。
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    //：目标关节角度，用于指定机器人应该移动到的目标位置。
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    //机器人的初始姿势，初始值同样为 REST_POSE
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    //机器人当前的六自由度位姿，初始值为空
    volatile uint8_t jointsStateFlag = 0b00000000;
    //关节状态标志，用于标识关节的运动状态
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    //机器人的命令模式，初始值为 DEFAULT_COMMAND_MODE，即默认的目标点可中断命令模式
    CtrlStepMotor* motorJ[7] = {nullptr};
    //包含七个关节的电机控制对象的数组，初始化时所有元素都为 nullptr。
    DummyHand* hand = {nullptr};
    //一个 DummyHand 对象的指针，用于控制机器人手部分的功能。
    //nullptr 它表示一个没有指向任何对象的空指针,
    //表示在初始状态下没有为这些关节分配实际的电机控制对象。这通常在构造函数中进行初始化。


    void Init();
    //Init: 初始化虚拟机器人
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    //MoveJ: 控制机器人沿指定关节角度移动
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    //MoveL: 控制机器人沿指定的直线路径移动。
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    //MoveJoints: 直接设置机器人的关节角度
    void SetJointSpeed(float _speed);
    // /SetJointSpeed: 设置机器人关节的运动速度。
    void SetJointAcceleration(float _acc);
    //SetJointAcceleration: 设置机器人关节的运动加速度
    void UpdateJointAngles();
    //UpdateJointAngles: 更新机器人关节的角度
    void UpdateJointAnglesCallback();
    //UpdateJointAnglesCallback: 更新机器人关节角度的回调函数
    void UpdateJointPose6D();
    //UpdateJointPose6D: 更新机器人的六维位姿。
    void Reboot();
    //Reboot: 重启机器人。
    void SetEnable(bool _enable);
    //SetEnable: 启用或禁用机器人。
    void SetRGBEnable(bool _enable);
    //SetRGBEnable: 启用或禁用机器人上的RGB功能。
    bool GetRGBEnabled();
    //GetRGBEnabled: 获取RGB功能的启用状态。
    void SetRGBMode(uint32_t mode);
    //SetRGBMode: 设置机器人上RGB的模式。
    uint32_t GetRGBMode();
    //GetRGBMode: 获取机器人上RGB的模式。
    void CalibrateHomeOffset();
    //CalibrateHomeOffset: 校准机器人的初始位置
    void Homing();
    //Homing: 启动机器人的回零过程。
    void Resting();
    //Resting: 将机器人置于休息状态。
    bool IsMoving();
    //IsMoving: 检查机器人是否在移动。
    bool IsEnabled();
    //IsEnabled: 检查机器人是否已启用。
    void SetCommandMode(uint32_t _mode);
    //SetCommandMode: 设置机器人的命令模式。


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    //DummyRobot::MakeProtocolDefinitions 函数定义了虚拟机器人的通信协议，
    //包括关节控制、运动、校准、重启等功能。以下是该函数的详细说明：
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("set_rgb_enable", *this, &DummyRobot::SetRGBEnable, "enable"),
            make_protocol_function("set_rgb_mode", *this, &DummyRobot::SetRGBMode, "mode"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }
       // 构造函数：接受 DummyRobot 对象作为上下文，并创建一个消息队列用于存储命令。

        uint32_t Push(const std::string &_cmd);
        //Push 函数：将命令推送到消息队列中。
        std::string Pop(uint32_t timeout);
        //Pop 函数：从消息队列中弹出命令。
        uint32_t ParseCommand(const std::string &_cmd);
        //ParseCommand 函数：解析命令。
        uint32_t GetSpace();
        //GetSpace 函数：获取消息队列中可用空间的数量。
        void ClearFifo();
        //ClearFifo 函数：清空消息队列。
        void EmergencyStop();
        //EmergencyStop 函数：执行紧急停止操作。


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);
//CommandHandler 对象作为 DummyRobot 类的成员：

private:
    CAN_HandleTypeDef* hcan;
    //指向 CAN 处理结构体的指针，用于与 CAN 总线进行通信。
    float jointSpeed = DEFAULT_JOINT_SPEED;
    //关节速度，初始化为默认值。
    float jointSpeedRatio = 1;
    //关节速度比率，初始化为 1。
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    //动态关节速度，初始化为 {1, 1, 1, 1, 1, 1}。
    DOF6Kinematic* dof6Solver;
    //对象的指针，用于处理六自由度运动学。
    bool isEnabled = false;
    //表示机器人是否启用，初始化为 false。
    bool isRGBEnabled = false;
    //表示 RGB 功能是否启用，初始化为 false。
    uint32_t rgbMode = 0;
    //RGB 模式，初始化为 0。
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
