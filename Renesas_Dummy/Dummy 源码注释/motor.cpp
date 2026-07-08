#include "configurations.h"
#include "motor.h"

#include <cmath>

//这是一个用于电机每20kHz执行一次的主要函数。
//在每次调用时，它首先更新编码器数据，然后执行电机的闭环控制。
void Motor::Tick20kHz()
{
    // 1.Encoder data Update  1. 更新编码器数据
    encoder->UpdateAngle();

    // 2.Motor Control Update 2. 执行电机闭环控制
    CloseLoopControlTick();
}

//将传入的编码器对象与电机关联起来
void Motor::AttachEncoder(EncoderBase* _encoder)
{
    // 将传入的编码器对象赋值给电机的成员变量
    encoder = _encoder;
}

//将传入的驱动器对象与电机关联起来
void Motor::AttachDriver(DriverBase* _driver)
{
    // 将传入的驱动器对象赋值给电机的成员变量
    driver = _driver;
}


void Motor::CloseLoopControlTick()
{
    /************************************ First Called 第一次调用************************************/
     // 静态变量，用于标记是否是第一次调用该函数
    static bool isFirstCalled = true;
     // 如果是第一次调用该函数
    if (isFirstCalled)
    {
        int32_t angle;
        // 根据配置的编码器家位置偏移，初始化电机的实际位置
        if (config.motionParams.encoderHomeOffset < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2)
        {
            // 如果家位置偏移小于半个步进电机圈数，根据当前角度判断初始化位置
            angle =
                encoder->angleData.rectifiedAngle >
                config.motionParams.encoderHomeOffset + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        } else
        {
            // 如果家位置偏移大于半个步进电机圈数，根据当前角度判断初始化位置
            angle =
                encoder->angleData.rectifiedAngle <
                config.motionParams.encoderHomeOffset - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        }

        // 初始化电机控制器的实际圈数位置和实际位置
        controller->realLapPosition = angle;
        controller->realLapPositionLast = angle;
        controller->realPosition = angle;
        controller->realPositionLast = angle;

        // 标记第一次调用已经完成
        isFirstCalled = false;
        return;
    }
    //这段代码是电机的闭环控制函数。在首次调用时，它通过使用编码器的数据和配置的编码器家位置偏移，
    //初始化了电机控制器的实际位置信息。静态变量 isFirstCalled 用于标记是否是第一次调用函数，
    //确保只在第一次调用时执行初始化操作。

    /********************************* Update Data 更新数据*********************************/
    // 定义变量以存储圈数位置变化
    int32_t deltaLapPosition;

    // Read Encoder data  读取编码器的当前角度数据
    // 保存上一个周期的圈数位置
    controller->realLapPositionLast = controller->realLapPosition;
    // 获取当前圈数位置
    controller->realLapPosition = encoder->angleData.rectifiedAngle;

    // Lap-Position calculate 计算圈数位置的变化量
    deltaLapPosition = controller->realLapPosition - controller->realLapPositionLast;
     // 若变化量大于一圈的一半
    if (deltaLapPosition > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
    // 减去一圈的值，确保变化量在一圈内
        deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        // 若变化量小于负一圈的一半
    else if (deltaLapPosition < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
     // 加上一圈的值，确保变化量在一圈内
        deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

    // Naive-Position calculate 更新简单位置信息
    // 保存上一个周期的简单位置
    controller->realPositionLast = controller->realPosition;
    // 加上圈数位置的变化量，更新简单位置
    controller->realPosition += deltaLapPosition;
    //这段代码负责更新电机控制器的位置信息。
    //首先，通过读取编码器的角度数据，计算出圈数位置的变化量 deltaLapPosition。
    //然后，根据圈数位置的溢出情况进行处理，确保变化量在一个圈内。
    //最后，利用变化量更新电机的简单位置信息，以反映电机当前的位置。

    /********************************* Estimate Data 估算数据 *********************************/
    // Estimate Velocity
    // 估算速度
    // 首先，将上一个周期到当前周期的位置变化量乘以控制频率，得到积分项。
    // 其次，通过对当前速度的估算值进行更新，该估算值由上一个周期的速度值进行修正。
    // 最后，通过右移 5 位来得到实际的速度估算值，并更新速度积分项。
    controller->estVelocityIntegral += (
        (controller->realPosition - controller->realPositionLast) * motionPlanner.CONTROL_FREQUENCY
        + ((controller->estVelocity << 5) - controller->estVelocity)
    );
    // 通过右移 5 位得到实际速度
    controller->estVelocity = controller->estVelocityIntegral >> 5;
    // 更新速度积分项
    controller->estVelocityIntegral -= (controller->estVelocity << 5);


    // Estimate Position
    // 估算位置
    // 通过补偿速度提前角度，估算出实际位置。
    // 这是一种对速度提前角度进行补偿的方法，用于更精准地估算位置。
    controller->estLeadPosition = Controller::CompensateAdvancedAngle(controller->estVelocity);
    // 计算补偿的提前角度
    controller->estPosition = controller->realPosition + controller->estLeadPosition;
    // 估算实际位置
    // Estimate Error
    // 估算误差
    // 通过目标位置和估算位置的差值，得到估算误差。
    controller->estError = controller->softPosition - controller->estPosition;
    //这段代码用于估算电机的速度、位置和误差。
    //首先，通过积分上一个周期到当前周期的位置变化量，
    //结合修正后的速度估算值，更新速度的实际估算值，并更新速度积分项。
    //然后，通过对速度进行提前角度的补偿，计算出实际位置。
    //最后，通过目标位置和估算位置的差值，得到位置估算的误差。

    /************************************ Ctrl Loop 控制循环 ************************************/
    // 检查是否电机被卡住、软禁用或者编码器未校准
    if (controller->isStalled ||
        controller->softDisable ||
        !encoder->IsCalibrated())
    {
        controller->ClearIntegral();    // clear integrals // 清除积分项
        controller->focPosition = 0;    // clear outputs  // 清除输出项
        controller->focCurrent = 0;
        driver->Sleep();                // 使驱动器进入休眠状态
    } else if (controller->softBrake)
    {
        controller->ClearIntegral();    // 清除积分项
        controller->focPosition = 0;    // 清除输出项
        controller->focCurrent = 0;
        driver->Brake();                // 施加软刹车
    } else
    // 如果没有异常情况，则根据运行模式进行相应的控制
    {
        // 根据运行模式进行不同的控制
        switch (controller->modeRunning)
        {
            //控制器使用直流电调制 (DCE) 控制，计算并输出给定位置和速度的电流。
            case MODE_STEP_DIR:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            //使驱动器进入休眠状态，停止电机运动
            case MODE_STOP:
                driver->Sleep();
                break;
                //控制器使用直流电调制 (DCE) 控制，计算并输出给定位置和速度的电流。
            case MODE_COMMAND_Trajectory:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
                //控制器计算并输出给定电流
            case MODE_COMMAND_CURRENT:
                controller->CalcCurrentToOutput(controller->softCurrent);
                break;
                //控制器使用比例积分微分 (PID) 控制，计算并输出给定速度的电流。
            case MODE_COMMAND_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
                //控制器使用直流电调制 (DCE) 控制，计算并输出给定位置和速度的电流。
            case MODE_COMMAND_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
                //控制器计算并输出给定电流，使用脉宽调制 (PWM) 控制。
            case MODE_PWM_CURRENT:
                controller->CalcCurrentToOutput(controller->softCurrent);
                break;
                //控制器使用比例积分微分 (PID) 控制，计算并输出给定速度的电流，使用脉宽调制 (PWM) 控制。
            case MODE_PWM_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
                //控制器使用直流电调制 (DCE) 控制，计算并输出给定位置和速度的电流，使用脉宽调制 (PWM) 控制。
            case MODE_PWM_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
                //如果controller->modeRunning不匹配上述任何一个值，不执行任何具体操作
            default:
                break;
                //这段代码是电机的控制循环。首先，检查电机是否被卡住、软禁用或者编码器未校准，
                //如果是，则清除积分项、输出项，并使驱动器进入休眠状态。
                //如果启用了软刹车，则同样清除积分项、输出项，并施加软刹车。
                //最后，如果没有异常情况，则根据不同的运行模式选择相应的控制方法。
        }
    }

    /******************************* Mode Change Handle *******************************/
    // 检查控制器的运行模式是否需要切换
    if (controller->modeRunning != controller->requestMode)
    {
        // 更新运行模式为请求的模式
        controller->modeRunning = controller->requestMode;
        // 标记需要重新计算曲线或执行特殊操作的标志
        controller->softNewCurve = true;
    }
    //模式比较： 检查当前运行模式 controller->modeRunning
    // 是否与请求的模式 controller->requestMode 相同。
    //模式切换： 如果当前运行模式与请求的模式不同，表示需要进行模式切换。
    //更新模式： 将当前运行模式更新为请求的模式，以确保控制器处于请求的工作模式。
    //标志设置： 设置 controller->softNewCurve 标志为 true，
    //可能表示在模式切换后需要进行一些特殊的操作或重新计算曲线。

    /******************************* Update Hard-Goal 更新控制器的硬性指标*******************************/
    // 更新硬目标值，确保目标值在合理范围内
    if (controller->goalVelocity > config.motionParams.ratedVelocity)
     // 如果目标速度超过额定速度上限，将其限制为额定速度上限
        controller->goalVelocity = config.motionParams.ratedVelocity;
    else if (controller->goalVelocity < -config.motionParams.ratedVelocity)
     // 如果目标速度低于额定速度下限，将其限制为额定速度下限
        controller->goalVelocity = -config.motionParams.ratedVelocity;
    if (controller->goalCurrent > config.motionParams.ratedCurrent)
     // 如果目标电流超过额定电流上限，将其限制为额定电流上限
        controller->goalCurrent = config.motionParams.ratedCurrent;
    else if (controller->goalCurrent < -config.motionParams.ratedCurrent)
    // 如果目标电流低于额定电流下限，将其限制为额定电流下限
        controller->goalCurrent = -config.motionParams.ratedCurrent;

    /******************************** Motion Plan *********************************/
    // 如果软禁用被启用，而硬禁用被禁用；或者软刹车被启用，而硬刹车被禁用，则生成新的曲线
    if ((controller->softDisable && !controller->goalDisable) ||
        (controller->softBrake && !controller->goalBrake))
    {
        controller->softNewCurve = true;
    }

         // 如果需要生成新的曲线
    if (controller->softNewCurve)
    {
         // 重置软积分和清除堵转标志
        controller->softNewCurve = false;
        controller->ClearIntegral();
        controller->ClearStallFlag();

         // 根据运行模式生成新的运动任务
        switch (controller->modeRunning)
        {
            // 停止模式无需生成新的任务
            case MODE_STOP:
                break;
            // 位置模式下，生成新的位置追踪任务
            case MODE_COMMAND_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            // 速度模式下，生成新的速度追踪任务
            case MODE_COMMAND_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            // 电流模式下，生成新的电流追踪任务   
            case MODE_COMMAND_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            // 轨迹模式下，生成新的轨迹追踪任务
            case MODE_COMMAND_Trajectory:
                motionPlanner.trajectoryTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
             // PWM位置模式下，生成新的位置追踪任务
            case MODE_PWM_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            // PWM速度模式下，生成新的速度追踪任务
            case MODE_PWM_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            // PWM电流模式下，生成新的电流追踪任务
            case MODE_PWM_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            // Step/Dir模式下，生成新的位置插补任务
            case MODE_STEP_DIR:
                motionPlanner.positionInterpolator.NewTask(controller->estPosition, controller->estVelocity);
            // step/dir mode uses delta-position, so stay where we are
            // Step/Dir模式使用增量位置，因此保持当前位置不变
                controller->goalPosition = controller->estPosition;
                break;
            default:
                break;
        }
    }
    //这段代码处理运动规划的逻辑。首先检查软禁用和软刹车的状态，如果发生状态变化，
    //则标志需要生成新的曲线。如果需要生成新的曲线，根据当前运行模式选择性地生成新的运动任务，
    //包括位置追踪、速度追踪、电流追踪等不同的任务。这有助于在不同的运行模式下实现灵活的运动规划和控制。

    /******************************* Update Soft Goal *******************************/
    // 根据当前运行模式更新软目标
    switch (controller->modeRunning)
    {
        // 停止模式下无需更新软目标
        case MODE_STOP:
            break;
             // 位置模式下，使用位置追踪器计算软目标
        case MODE_COMMAND_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
            // 速度模式下，使用速度追踪器计算软目标
        case MODE_COMMAND_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
            // 电流模式下，使用电流追踪器计算软目标
        case MODE_COMMAND_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
            // 轨迹模式下，使用轨迹追踪器计算软目标
        case MODE_COMMAND_Trajectory:
            motionPlanner.trajectoryTracker.CalcSoftGoal(controller->goalPosition, controller->goalVelocity);
            controller->softPosition = motionPlanner.trajectoryTracker.goPosition;
            controller->softVelocity = motionPlanner.trajectoryTracker.goVelocity;
            break;
            // PWM位置模式下，使用位置追踪器计算软目标
        case MODE_PWM_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
            // PWM速度模式下，使用速度追踪器计算软目标
        case MODE_PWM_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
            // PWM电流模式下，使用电流追踪器计算软目标
        case MODE_PWM_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
            // Step/Dir模式下，使用位置插补器计算软目标
        case MODE_STEP_DIR:
            motionPlanner.positionInterpolator.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionInterpolator.goPosition;
            controller->softVelocity = motionPlanner.positionInterpolator.goVelocity;
            break;
        default:
            break;
    }

// 更新软禁用和软刹车状态
    controller->softDisable = controller->goalDisable;
    controller->softBrake = controller->goalBrake;
    //这段代码根据当前运行模式更新软目标。根据不同的模式，
    //使用相应的追踪器或插补器计算软目标的位置、速度、电流等值。
    //然后将软禁用和软刹车状态更新为硬目标的对应状态。
    //这有助于实现软目标与硬目标之间的平滑过渡。


    /******************************** State Check ********************************/
    int32_t current = abs(controller->focCurrent);

    // Stall detect 堵转检测
    // 如果启用了堵转保护
    if (controller->config->stallProtectSwitch)
    {
        if (// Current Mode 电流模式
           (
            ((controller->modeRunning == MODE_COMMAND_CURRENT ||
              controller->modeRunning == MODE_PWM_CURRENT) &&
             (current != 0))
            || // Other Mode 其他模式
            // 如果当前运行模式是电流模式或PWM电流模式且当前电流不为零
            // 或者当前电流等于额定电流
            current == config.motionParams.ratedCurrent)
        {
             // 如果估计速度的绝对值小于电机一圈的步数的五分之一
            if (abs(controller->estVelocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)
            {
                // 如果堵转时间超过了一秒
                if (controller->stalledTime >= 1000 * 1000)
                // 将堵转标志设置为true
                    controller->isStalled = true;
                // 否则增加堵转时间
                else
                    controller->stalledTime += motionPlanner.CONTROL_PERIOD;
            }
              //else 只能手动清除堵转标志
        } else // can ONLY clear stall flag  MANUALLY
        {
             // 否则将堵转时间清零
            controller->stalledTime = 0;
        }
    }

    // Overload detect
    if ((controller->modeRunning != MODE_COMMAND_CURRENT) &&
        (controller->modeRunning != MODE_PWM_CURRENT) &&
        (current == config.motionParams.ratedCurrent))
    {
        if (controller->overloadTime >= 1000 * 1000)
            controller->overloadFlag = true;
        else
            controller->overloadTime += motionPlanner.CONTROL_PERIOD;
    } else // auto clear overload flag when released
    {
        controller->overloadTime = 0;
        controller->overloadFlag = false;
    }
    //这段代码实现了堵转检测的逻辑。首先检查是否启用了堵转保护。
    //然后，根据当前运行模式和电流值进行判断，
    //如果满足一定条件（在电流模式或PWM电流模式且电流不为零，或者电流等于额定电流），
    //则进行速度估计绝对值的比较。如果估计速度的绝对值小于电机一圈的步数的五分之一，
    //且堵转时间超过一秒，则将堵转标志设置为true；否则，增加堵转时间。如果不满足堵转条件，
    //则手动将堵转时间清零。

    /******************************** Update State ********************************/
    if (!encoder->IsCalibrated())
        controller->state = STATE_NO_CALIB;
    else if (controller->modeRunning == MODE_STOP)
        controller->state = STATE_STOP;
    else if (controller->isStalled)
        controller->state = STATE_STALL;
    else if (controller->overloadFlag)
        controller->state = STATE_OVERLOAD;
    else
    {
        if (controller->modeRunning == MODE_COMMAND_POSITION)
        {
            if ((controller->softPosition == controller->goalPosition)
                && (controller->softVelocity == 0))
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_VELOCITY)
        {
            if (controller->softVelocity == controller->goalVelocity)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_CURRENT)
        {
            if (controller->softCurrent == controller->goalCurrent)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else
        {
            controller->state = STATE_FINISH;
        }
    }
}


void Motor::Controller::CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;

    if (focCurrent > 0)
        focPosition = estPosition + context->SOFT_DIVIDE_NUM; // ahead phase of 90°
    else if (focCurrent < 0)
        focPosition = estPosition - context->SOFT_DIVIDE_NUM; // behind phase of 90°
    else focPosition = estPosition;

    context->driver->SetFocCurrentVector(focPosition, focCurrent);
}


void Motor::Controller::CalcPidToOutput(int32_t _speed)
{
    config->pid.vErrorLast = config->pid.vError;
    config->pid.vError = _speed - estVelocity;
    if (config->pid.vError > (1024 * 1024)) config->pid.vError = (1024 * 1024);
    if (config->pid.vError < (-1024 * 1024)) config->pid.vError = (-1024 * 1024);
    config->pid.outputKp = ((config->pid.kp) * (config->pid.vError));

    config->pid.integralRound += (config->pid.ki * config->pid.vError);
    config->pid.integralRemainder = config->pid.integralRound >> 10;
    config->pid.integralRound -= (config->pid.integralRemainder << 10);
    config->pid.outputKi += config->pid.integralRemainder;
    // integralRound limitation is  ratedCurrent*1024
    if (config->pid.outputKi > context->config.motionParams.ratedCurrent << 10)
        config->pid.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->pid.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->pid.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->pid.outputKd = config->pid.kd * (config->pid.vError - config->pid.vErrorLast);

    config->pid.output = (config->pid.outputKp + config->pid.outputKi + config->pid.outputKd) >> 10;
    if (config->pid.output > context->config.motionParams.ratedCurrent)
        config->pid.output = context->config.motionParams.ratedCurrent;
    else if (config->pid.output < -context->config.motionParams.ratedCurrent)
        config->pid.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->pid.output);
}


void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed)
{
    config->dce.pError = _location - estPosition;
    if (config->dce.pError > (3200)) config->dce.pError = (3200);   // limited pError to 1/16r (51200/16)
    if (config->dce.pError < (-3200)) config->dce.pError = (-3200);
    config->dce.vError = (_speed - estVelocity) >> 7;
    if (config->dce.vError > (4000)) config->dce.vError = (4000);   // limited vError
    if (config->dce.vError < (-4000)) config->dce.vError = (-4000);

    config->dce.outputKp = config->dce.kp * config->dce.pError;

    config->dce.integralRound += (config->dce.ki * config->dce.pError + config->dce.kv * config->dce.vError);
    config->dce.integralRemainder = config->dce.integralRound >> 7;
    config->dce.integralRound -= (config->dce.integralRemainder << 7);
    config->dce.outputKi += config->dce.integralRemainder;
    // limited to ratedCurrent * 1024, should be TUNED when use different scene
    if (config->dce.outputKi > context->config.motionParams.ratedCurrent << 10)
        config->dce.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->dce.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->dce.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->dce.outputKd = ((config->dce.kd) * (config->dce.vError));

    config->dce.output = (config->dce.outputKp + config->dce.outputKi + config->dce.outputKd) >> 10;
    if (config->dce.output > context->config.motionParams.ratedCurrent)
        config->dce.output = context->config.motionParams.ratedCurrent;
    else if (config->dce.output < -context->config.motionParams.ratedCurrent)
        config->dce.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->dce.output);
}


void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)
{
    requestMode = _mode;
}


void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel)
{
    SetPositionSetPoint(_pos);
    SetVelocitySetPoint(_vel);
}


void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    goalPosition = _pos + context->config.motionParams.encoderHomeOffset;
}


bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    int32_t deltaPos = abs(_pos - realPosition + context->config.motionParams.encoderHomeOffset);

    float pMax = (float) context->config.motionParams.ratedVelocityAcc * _time * _time / 4;
    if ((float) deltaPos > pMax)
    {
        context->config.motionParams.ratedVelocity = boardConfig.velocityLimit;
        SetPositionSetPoint(_pos);

        return false;
    } else
    {
        float vMax = _time * (float) context->config.motionParams.ratedVelocityAcc;
        vMax -= (float) context->config.motionParams.ratedVelocityAcc *
                (sqrtf(_time * _time - 4 * (float) deltaPos /
                                       (float) context->config.motionParams.ratedVelocityAcc));
        vMax /= 2;

        context->config.motionParams.ratedVelocity = (int32_t) vMax;
        SetPositionSetPoint(_pos);

        return true;
    }
}


void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if ((_vel >= -context->config.motionParams.ratedVelocity) &&
        (_vel <= context->config.motionParams.ratedVelocity))
    {
        goalVelocity = _vel;
    }
}


float Motor::Controller::GetPosition(bool _isLap)
{
    return _isLap ?
           (float) (realLapPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  :
           (float) (realPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}


float Motor::Controller::GetVelocity()
{
    return (float) estVelocity / (float) context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


float Motor::Controller::GetFocCurrent()
{
    return (float) focCurrent / 1000.f;
}


void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    if (_cur > context->config.motionParams.ratedCurrent)
        goalCurrent = context->config.motionParams.ratedCurrent;
    else if (_cur < -context->config.motionParams.ratedCurrent)
        goalCurrent = -context->config.motionParams.ratedCurrent;
    else
        goalCurrent = _cur;
}


void Motor::Controller::SetDisable(bool _disable)
{
    goalDisable = _disable;
}


void Motor::Controller::SetBrake(bool _brake)
{
    goalBrake = _brake;

}


void Motor::Controller::ClearStallFlag()
{
    stalledTime = 0;
    isStalled = false;
}


int32_t Motor::Controller::CompensateAdvancedAngle(int32_t _vel)
{
    /*
     * The code is for DPS series sensors, need to measured and renew for TLE5012/MT6816.
     */

    int32_t compensate;

    if (_vel < 0)
    {
        if (_vel > -100000) compensate = 0;
        else if (_vel > -1300000) compensate = (((_vel + 100000) * 262) >> 20) - 0;
        else if (_vel > -2200000) compensate = (((_vel + 1300000) * 105) >> 20) - 300;
        else compensate = (((_vel + 2200000) * 52) >> 20) - 390;

        if (compensate < -430) compensate = -430;
    } else
    {
        if (_vel < 100000) compensate = 0;
        else if (_vel < 1300000) compensate = (((_vel - 100000) * 262) >> 20) + 0;
        else if (_vel < 2200000) compensate = (((_vel - 1300000) * 105) >> 20) + 300;
        else compensate = (((_vel - 2200000) * 52) >> 20) + 390;

        if (compensate > 430) compensate = 430;
    }

    return compensate;
}


void Motor::Controller::Init()
{
    requestMode = boardConfig.enableMotorOnBoot ? static_cast<Mode_t>(boardConfig.defaultMode) : MODE_STOP;

    modeRunning = MODE_STOP;
    state = STATE_STOP;

    realLapPosition = 0;
    realLapPositionLast = 0;
    realPosition = 0;
    realPositionLast = 0;

    estVelocityIntegral = 0;
    estVelocity = 0;
    estLeadPosition = 0;
    estPosition = 0;
    estError = 0;

    goalPosition = context->config.motionParams.encoderHomeOffset;
    goalVelocity = 0;
    goalCurrent = 0;
    goalDisable = false;
    goalBrake = false;

    softPosition = 0;
    softVelocity = 0;
    softCurrent = 0;
    softDisable = false;
    softBrake = false;
    softNewCurve = false;

    focPosition = 0;
    focCurrent = 0;

    stalledTime = 0;
    isStalled = false;

    overloadTime = 0;
    overloadFlag = false;

    config->pid.vError = 0;
    config->pid.vErrorLast = 0;
    config->pid.outputKp = 0;
    config->pid.outputKi = 0;
    config->pid.outputKd = 0;
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.output = 0;

    config->dce.pError = 0;
    config->dce.vError = 0;
    config->dce.outputKp = 0;
    config->dce.outputKi = 0;
    config->dce.outputKd = 0;
    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.output = 0;
}


void Motor::Controller::ApplyPosAsHomeOffset()
// 使用当前位置（realPosition）作为编码器的零位偏移
{
    //计算当前位置对一个电机圆周细分的余数，即计算当前位置在一个电机圆周中的偏移
    context->config.motionParams.encoderHomeOffset = realPosition %
    // 计算当前位置对一个电机圆周细分的余数，即计算当前位置在一个电机圆周中的偏移
                                                     context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
} // 将计算得到的偏移作为编码器的零位偏移
//realPosition 是当前电机位置。
//context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS 表示一个电机圆周的细分步数。
//通过取余操作，计算当前位置在一个电机圆周中的偏移，这个偏移将被用作编码器的零位偏移。
//最终，context->config.motionParams.encoderHomeOffset 
//将被设置为当前位置相对于一个电机圆周的偏移值，作为零位偏移。这在一些应用中用于校准编码器零位。

void Motor::Controller::AttachConfig(Motor::Controller::Config_t* _config)
{
    config = _config;
}


void Motor::Controller::ClearIntegral() const
{
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.outputKi = 0;

    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.outputKi = 0;
}


