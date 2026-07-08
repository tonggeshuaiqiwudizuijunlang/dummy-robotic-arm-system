/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "bsp_dwt.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

#define PID_MIN_DT 1e-6f

static float PID_Safe_Dt(float dt)
{
    if (!isfinite(dt) || dt < PID_MIN_DT)
        return PID_MIN_DT;
    return dt;
}

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        float abs_err = fabsf(pid->Err);

        // 积分呈累积趋势
        if (abs_err <= pid->CoefB)
            return; // Full integral
        if (abs_err <= (pid->CoefA + pid->CoefB) && fabsf(pid->CoefA) > PID_MIN_DT)
            pid->ITerm *= (pid->CoefA - abs_err + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (fabsf(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    float denominator = pid->Derivative_LPF_RC + pid->dt;
    if (!isfinite(denominator) || denominator < PID_MIN_DT)
        denominator = PID_MIN_DT;

    pid->Dout = pid->Dout * pid->dt / denominator +
                pid->Last_Dout * pid->Derivative_LPF_RC / denominator;
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    float denominator = pid->Output_LPF_RC + pid->dt;
    if (!isfinite(denominator) || denominator < PID_MIN_DT)
        denominator = PID_MIN_DT;

    pid->Output = pid->Output * pid->dt / denominator +
                  pid->Last_Output * pid->Output_LPF_RC / denominator;
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
    // 后续修改为逐个赋值
    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // set rest of memory to 0
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = PID_Safe_Dt(DWT_GetDeltaT(&pid->DWT_CNT)); // 获取两次pid计算的时间间隔,用于积分和微分

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm; // 累加积分

        pid->Fout = (pid->Measure - pid->Last_Measure) * pid->kf;    // 计算输出前馈
        pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Fout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief          增量式PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate1(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = PID_Safe_Dt(DWT_GetDeltaT(&pid->DWT_CNT)); // 获取两次PID计算的时间间隔, 用于积分和微分

    // 保存当前测量值和参考值, 计算当前误差
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外, 则计算增量式PID
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的增量式PID计算
        float deltaP = pid->Kp * (pid->Err - pid->Last_Err);                                    // 比例增量
        float deltaI = pid->Ki * pid->Err * pid->dt;                                            // 积分增量
        float deltaD = pid->Kd * (pid->Err - 2 * pid->Last_Err + pid->Last_Last_Err) / pid->dt; // 微分增量

        // 总增量
        float deltaOutput = deltaP + deltaI + deltaD;

        // 更新输出值
        pid->Output = pid->Last_Output + deltaOutput;

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则保持当前输出
    {
        pid->Output = pid->Last_Output;
    }

    // 保存当前数据, 用于下次计算
    pid->Last_Last_Err = pid->Last_Err; // 保存前两次误差
    pid->Last_Err = pid->Err;           // 保存上一次误差
    pid->Last_Output = pid->Output;     // 保存当前输出

    return pid->Output;
}

float PID_increment(PIDInstance *PID, float measure, float ref)
{
    // 保存上次的测量值和误差,计算当前error
    PID->Measure = measure;
    PID->Ref = ref;
    PID->Err = PID->Ref - PID->Measure;
    PID->Output += PID->Kp * (PID->Err - PID->Err1) + PID->Ki * PID->Err + PID->Kd * (PID->Err - 2 * PID->Err1 + PID->Err2);
    //	PID->out+=PID->kp*PID->ek-PID->ki*PID->ek1+PID->kd*PID->ek2;
    PID->Err2 = PID->Err1;
    PID->Err1 = PID->Err;
    f_Output_Limit(PID);
    return PID->Output;
}

// 带约束的版本（将输出限制在目标范围内）
float map_float_clamp(float x, float in_min, float in_max, float out_min, float out_max) 
{
    // 先约束输入值在输入范围内
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    
    // 执行映射
    float result = out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    
    // 约束输出值在输出范围内（防止浮点误差）
    if (result < out_min) result = out_min;
    if (result > out_max) result = out_max;
    
    return result;
}