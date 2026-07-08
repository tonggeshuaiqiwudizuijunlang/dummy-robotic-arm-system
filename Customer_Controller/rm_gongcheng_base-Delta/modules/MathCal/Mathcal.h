#ifndef __MATH_CAL_H
#define __MATH_CAL_H
#include "arm_math.h"
#define pi 3.14159265358979323846f
#define Degree_2_Rad 0.01745329252f
#define ERROR_THRESHOLD 5.f

#define POINT 200 //* 轨迹规划200个点
void DH_Init(void);
void Transformation_Init(void);
void robot_ik(float target[5][5], float *q_val, float *live_error);

#endif // !__MATH_CAL_H