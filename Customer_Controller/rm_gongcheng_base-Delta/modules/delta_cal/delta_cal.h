#ifndef __DELTA_CAL_H
#define __DELTA_CAL_H
#include "arm_math.h"
#define pi 3.14159265358979323846f

void Gravity_compensation(float *theta, float *tor, float *xyz);
void Delta_ik(float *target);
void Delta_fk(float *theta, float *target_xyz);
void Rotation_ZYX_Matrix(float rotation[4][4], float yaw, float pitch, float roll);

#endif // !__DELTA_CAL_H