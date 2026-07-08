#include "dummy_kinematic_v2.h"
#include <float.h>
#include <string.h>

#define IK_SOLVE_COUNT 8
#define KINEMATIC_EPSILON 1e-6f
#define CONTROLLER_POSE_SCALE_MM 1000.0f

typedef struct {
    double theta[6];
} JointAngles;

typedef struct {
    double d[6];
    double a[6];
    double alpha[6];
} DHParameters;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} EulerAngles;

typedef struct {
    double x;
    double y;
    double z;
} Vector3;

typedef struct {
    double m[4][4];
} Matrix4x4;

typedef struct {
    double m[3][3];
} Matrix3x3;

typedef struct {
    Vector3 position;
    Matrix3x3 rotation;
    Matrix4x4 transform;
} EndEffectorState;

typedef struct {
    JointAngles angles[8];
    int count;
} InverseKinematicsSolutions;

static void Apply_Affine_Map(const AffineAngleMap_t *map, const Joint6D_t *input, Joint6D_t *output);

static double Deg_To_Rad(double deg)
{
    return deg * M_PI / 180.0;
}

static double Rad_To_Deg(double rad)
{
    return rad * 180.0 / M_PI;
}

static double Normalize_Angle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

static bool Kinematic_Double_Is_Finite(double value)
{
    return isfinite(value) != 0;
}

static bool Kinematic_Float_Is_Finite(float value)
{
    return isfinite(value) != 0;
}

static bool Kinematic_Vector_Is_Finite(const Vector3 *position)
{
    return position != NULL &&
           Kinematic_Double_Is_Finite(position->x) &&
           Kinematic_Double_Is_Finite(position->y) &&
           Kinematic_Double_Is_Finite(position->z);
}

static bool Kinematic_Rotation_Is_Finite(const Matrix3x3 *rotation)
{
    if (rotation == NULL)
        return false;

    for (uint8_t r = 0; r < 3U; r++)
    {
        for (uint8_t c = 0; c < 3U; c++)
        {
            if (!Kinematic_Double_Is_Finite(rotation->m[r][c]))
                return false;
        }
    }
    return true;
}

static bool Kinematic_Safe_Sqrt(double value, double *result)
{
    const double eps = 1e-9;

    if (result == NULL || !Kinematic_Double_Is_Finite(value))
        return false;
    if (value < -eps)
        return false;
    if (value < 0.0)
        value = 0.0;

    *result = sqrt(value);
    return Kinematic_Double_Is_Finite(*result);
}

static bool Kinematic_Is_Angle_In_Limit(float angle_deg, const AngleLimit_t *limit)
{
    const float eps = 1e-4f;

    if (!Kinematic_Float_Is_Finite(angle_deg))
        return false;
    if (limit == NULL || !limit->enable)
        return true;
    return angle_deg >= (limit->min_deg - eps) && angle_deg <= (limit->max_deg + eps);
}

static bool Kinematic_Is_Motor_In_Limits(const ArmConfig_t *config, const Joint6D_t *motor_joint_deg)
{
    if (config == NULL || motor_joint_deg == NULL)
        return false;

    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (!Kinematic_Is_Angle_In_Limit(motor_joint_deg->a[i], &config->motor_limit[i]))
            return false;
    }
    return true;
}

static bool Kinematic_Is_Coupling_In_Limits(const ArmConfig_t *config, const Joint6D_t *motor_joint_deg)
{
    const float eps = 1e-4f;

    if (config == NULL || motor_joint_deg == NULL)
        return false;

    for (uint8_t i = 0U; i < config->coupling_limit_count && i < 4U; i++)
    {
        const MotorCouplingLimit_t *limit = &config->coupling_limits[i];
        if (!limit->enable || limit->lhs_motor_index >= 6U || limit->rhs_motor_index >= 6U)
            continue;

        float delta = motor_joint_deg->a[limit->lhs_motor_index] - motor_joint_deg->a[limit->rhs_motor_index];
        if (!Kinematic_Float_Is_Finite(delta) ||
            delta < (limit->min_delta_deg - eps) ||
            delta > (limit->max_delta_deg + eps))
            return false;
    }
    return true;
}

static Matrix4x4 Identity_Matrix4x4(void)
{
    Matrix4x4 I;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            I.m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    return I;
}

static Matrix4x4 Matrix4x4_Multiply(const Matrix4x4 *A, const Matrix4x4 *B)
{
    Matrix4x4 C;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            C.m[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
            {
                C.m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }
    return C;
}

static Matrix3x3 Euler_To_Rotation_Matrix(double roll, double pitch, double yaw)
{
    Matrix3x3 R;
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    R.m[0][0] = cy * cp;
    R.m[0][1] = cy * sp * sr - sy * cr;
    R.m[0][2] = cy * sp * cr + sy * sr;

    R.m[1][0] = sy * cp;
    R.m[1][1] = sy * sp * sr + cy * cr;
    R.m[1][2] = sy * sp * cr - cy * sr;

    R.m[2][0] = -sp;
    R.m[2][1] = cp * sr;
    R.m[2][2] = cp * cr;
    return R;
}

static EulerAngles Rotation_Matrix_To_Euler(const Matrix3x3 *R)
{
    EulerAngles euler;
    double sy = sqrt(R->m[0][0] * R->m[0][0] + R->m[1][0] * R->m[1][0]);

    if (sy > 1e-6)
    {
        euler.roll = atan2(R->m[2][1], R->m[2][2]);
        euler.pitch = atan2(-R->m[2][0], sy);
        euler.yaw = atan2(R->m[1][0], R->m[0][0]);
    }
    else
    {
        euler.roll = atan2(-R->m[1][2], R->m[1][1]);
        euler.pitch = atan2(-R->m[2][0], sy);
        euler.yaw = 0.0;
    }
    return euler;
}

static Matrix4x4 DH_Transform(double theta, double d, double a, double alpha)
{
    Matrix4x4 T;
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);

    T.m[0][0] = ct;
    T.m[0][1] = -st * ca;
    T.m[0][2] = st * sa;
    T.m[0][3] = a * ct;
    T.m[1][0] = st;
    T.m[1][1] = ct * ca;
    T.m[1][2] = -ct * sa;
    T.m[1][3] = a * st;
    T.m[2][0] = 0.0;
    T.m[2][1] = sa;
    T.m[2][2] = ca;
    T.m[2][3] = d;
    T.m[3][0] = 0.0;
    T.m[3][1] = 0.0;
    T.m[3][2] = 0.0;
    T.m[3][3] = 1.0;
    return T;
}

static void Config_To_DH(const ArmConfig_t *config, DHParameters *dh)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        dh->d[i] = config->dh.d[i];
        dh->a[i] = config->dh.a[i];
        dh->alpha[i] = config->dh.alpha[i];
    }
}

static EndEffectorState Forward_Kinematics(const JointAngles *angles, const DHParameters *dh)
{
    EndEffectorState state;
    Matrix4x4 T = Identity_Matrix4x4();

    for (int i = 0; i < 6; i++)
    {
        Matrix4x4 Ti = DH_Transform(angles->theta[i], dh->d[i], dh->a[i], dh->alpha[i]);
        T = Matrix4x4_Multiply(&T, &Ti);
    }

    state.transform = T;
    state.position.x = T.m[0][3];
    state.position.y = T.m[1][3];
    state.position.z = T.m[2][3];

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            state.rotation.m[i][j] = T.m[i][j];
        }
    }
    return state;
}

static bool Kinematic_Add_IK_Solution(InverseKinematicsSolutions *solutions,
                                      double q1,
                                      double q2,
                                      double q3,
                                      double q4,
                                      double q5,
                                      double q6,
                                      int *flag_beyond_180,
                                      bool enable_q3_patch,
                                      bool wrist_flip)
{
    double values[6] = {q1, q2, q3, q4, q5, q6};
    for (uint8_t i = 0; i < 6U; i++)
    {
        if (!Kinematic_Double_Is_Finite(values[i]))
            return false;
    }

    if (solutions == NULL || solutions->count >= 8)
        return false;

    double q3_tmp = Normalize_Angle(q3);
    if (enable_q3_patch && flag_beyond_180 != NULL)
    {
        if (q3_tmp * 180.0 / M_PI > 160.0)
            *flag_beyond_180 = 1;
        else if (q3_tmp * 180.0 / M_PI < 160.0 && q3_tmp > 0.0)
            *flag_beyond_180 = 0;
        if (*flag_beyond_180 && q3_tmp * 180.0 / M_PI < 0.0)
            q3_tmp = 2.0 * M_PI + q3_tmp;
    }

    JointAngles *sol = &solutions->angles[solutions->count];
    sol->theta[0] = Normalize_Angle(q1);
    sol->theta[1] = Normalize_Angle(q2);
    sol->theta[2] = q3_tmp;
    sol->theta[3] = Normalize_Angle(wrist_flip ? q4 + M_PI : q4);
    sol->theta[4] = Normalize_Angle(wrist_flip ? -q5 : q5);
    sol->theta[5] = Normalize_Angle(wrist_flip ? q6 + M_PI : q6);
    solutions->count++;
    return true;
}

static InverseKinematicsSolutions Inverse_Kinematics_From_Rotation_Matrix(const Vector3 *position,
                                                                          const Matrix3x3 *rotation,
                                                                          const DHParameters *dh,
                                                                          int *flag_beyond_180,
                                                                          bool enable_q3_patch)
{
    InverseKinematicsSolutions solutions;
    solutions.count = 0;

    if (!Kinematic_Vector_Is_Finite(position) || !Kinematic_Rotation_Is_Finite(rotation) || dh == NULL)
        return solutions;

    Matrix3x3 R06 = *rotation;
    double ax = R06.m[0][2];
    double ay = R06.m[1][2];
    double az = R06.m[2][2];
    double d2 = dh->d[1];
    double d4 = dh->d[3];
    double a2 = dh->a[2];
    double a3 = dh->a[3];
    double nx = R06.m[0][0];
    double ny = R06.m[1][0];
    double nz = R06.m[2][0];

    if (fabs(a2) < 1e-9 || !Kinematic_Double_Is_Finite(a2))
        return solutions;

    double px = position->x - dh->d[5] * ax;
    double py = position->y - dh->d[5] * ay;
    double pz = position->z - dh->d[5] * az;

    double temp = px * px + py * py - d2 * d2;
    double sqrt_temp = 0.0;
    if (!Kinematic_Safe_Sqrt(temp, &sqrt_temp))
        return solutions;

    double theta1_1 = atan2(py, px) - atan2(d2, sqrt_temp);
    double theta1_2 = atan2(py, px) - atan2(d2, -sqrt_temp);

    for (int i1 = 0; i1 < 2; i1++)
    {
        double q1 = (i1 == 0) ? theta1_1 : theta1_2;

        double k = (px * px + py * py + pz * pz - a2 * a2 - a3 * a3 - d2 * d2 - d4 * d4) /
                   (2.0 * a2);

        temp = a3 * a3 + d4 * d4 - k * k;
        if (!Kinematic_Safe_Sqrt(temp, &sqrt_temp))
            continue;

        for (int i3 = 0; i3 < 2; i3++)
        {
            double q3 = (i3 == 0) ?
                            atan2(a3, d4) - atan2(k, sqrt_temp) :
                            atan2(a3, d4) - atan2(k, -sqrt_temp);

            double q23 = atan2(-(a3 + a2 * cos(q3)) * pz + (cos(q1) * px + sin(q1) * py) * (a2 * sin(q3) - d4),
                               (a2 * sin(q3) - d4) * pz + (cos(q1) * px + sin(q1) * py) * (a2 * cos(q3) + a3));
            double q2 = q23 - q3;
            if (q3 < 0.0)
                q2 += M_PI / 2.0;
            else if (q3 > 0.0)
                q2 -= M_PI * 3.0 / 2.0;

            double q4 = atan2(-ax * sin(q1) + ay * cos(q1),
                              -ax * cos(q1) * cos(q23) - ay * sin(q1) * cos(q23) + az * sin(q23));

            double s5 = -(ax * (cos(q1) * cos(q23) * cos(q4) + sin(q1) * sin(q4)) +
                          ay * (sin(q1) * cos(q23) * cos(q4) - cos(q1) * sin(q4)) -
                          az * (sin(q23) * cos(q4)));

            double c5 = ax * (-cos(q1) * sin(q23)) +
                        ay * (-sin(q1) * sin(q23)) +
                        az * (-cos(q23));

            double q5 = atan2(s5, c5);

            double s6 = -nx * (cos(q1) * cos(q23) * sin(q4) - sin(q1) * cos(q4)) -
                        ny * (sin(q1) * cos(q23) * sin(q4) + cos(q1) * cos(q4)) +
                        nz * (sin(q23) * sin(q4));

            double c6 = nx * ((cos(q1) * cos(q23) * cos(q4) + sin(q1) * sin(q4)) * cos(q5) -
                              cos(q1) * sin(q23) * sin(q5)) +
                        ny * ((sin(q1) * cos(q23) * cos(q4) - cos(q1) * sin(q4)) * cos(q5) -
                              sin(q1) * sin(q23) * sin(q5)) -
                        nz * (sin(q23) * cos(q4) * cos(q5) + cos(q23) * sin(q5));

            double q6 = atan2(s6, c6);

            Kinematic_Add_IK_Solution(&solutions, q1, q2, q3, q4, q5, q6,
                                      flag_beyond_180, enable_q3_patch, false);
            Kinematic_Add_IK_Solution(&solutions, q1, q2, q3, q4, q5, q6,
                                      flag_beyond_180, enable_q3_patch, true);
        }
    }

    return solutions;
}

static void Fit_Angle_Solution(JointAngles *solution, const JointAngles *reference)
{
    for (int j = 3; j < 6; j++)
    {
        double delta = solution->theta[j] - reference->theta[j];
        if (delta > M_PI)
            solution->theta[j] -= 2.0 * M_PI;
        else if (delta < -M_PI)
            solution->theta[j] += 2.0 * M_PI;
    }
}

float Kinematic_Clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void Kinematic_Get_Default_Config(ArmConfig_t *config)
{
    if (config == NULL)
        return;
    memset(config, 0, sizeof(ArmConfig_t));
}

void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config)
{
    if (handle == NULL || config == NULL)
        return;

    memset(handle, 0, sizeof(DOF6Kinematic_Handle_t));
    memcpy(&handle->config, config, sizeof(ArmConfig_t));

    for (uint8_t i = 0; i < 6; i++)
    {
        handle->current_ik_rad[i] = Deg_To_Rad(config->ik_reference_init_deg[i]);
        handle->last_answer_rad[i] = handle->current_ik_rad[i];
    }
    handle->initialized = true;
}

static bool Controller_DeltaFK_Internal(const ControllerKinematicConfig_t *cfg, const float theta_rad[3], float xyz_m[3])
{
    float theta1 = theta_rad[0];
    float theta2 = theta_rad[1];
    float theta3 = theta_rad[2];
    float sqrt3 = sqrtf(3.0f);

    float R = cfg->delta_R_m;
    float r = cfg->delta_r_m;
    float L = cfg->delta_L_m;
    float La = cfg->delta_La_m;

    float A1 = R + L * cosf(theta1) - r;
    float C1 = L * sinf(theta1);

    float A2 = -0.5f * (R + L * cosf(theta2) - r);
    float B2 = (sqrt3 / 2.0f) * (R + L * cosf(theta2) - r);
    float C2 = L * sinf(theta2);

    float A3 = -0.5f * (R + L * cosf(theta3) - r);
    float B3 = -(sqrt3 / 2.0f) * (R + L * cosf(theta3) - r);
    float C3 = L * sinf(theta3);

    float D1 = 0.5f * (A2 * A2 - A1 * A1 + C2 * C2 - C1 * C1 + B2 * B2);
    float A21 = A2 - A1;
    float C21 = C2 - C1;
    float D2 = 0.5f * (A3 * A3 - A1 * A1 + C3 * C3 - C1 * C1 + B3 * B3);
    float A31 = A3 - A1;
    float C31 = C3 - C1;

    float denom1 = A21 * B3 - A31 * B2;
    float denom2 = A31 * B2 - A21 * B3;
    if (fabsf(denom1) < KINEMATIC_EPSILON || fabsf(denom2) < KINEMATIC_EPSILON)
        return false;

    float E1 = (B3 * C21 - B2 * C31) / denom1;
    float F1 = (B3 * D1 - B2 * D2) / denom1;
    float E2 = (A31 * C21 - A21 * C31) / denom2;
    float F2 = (A31 * D1 - A21 * D2) / denom2;

    float a = E1 * E1 + E2 * E2 + 1.0f;
    float b = 2.0f * E2 * F2 + 2.0f * C1 - 2.0f * E1 * (A1 - F1);
    float c = (A1 - F1) * (A1 - F1) + F2 * F2 + C1 * C1 - La * La;
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
        return false;

    float z = (-b - sqrtf(discriminant)) / (2.0f * a);
    xyz_m[0] = E1 * z + F1;
    xyz_m[1] = E2 * z + F2;
    xyz_m[2] = z;
    return true;
}

static void Matrix_3_Cross_Product_3(float a[4][4], float b[4][4], float res[4][4])
{
    memset(res, 0, 4U * 4U * sizeof(float));
    for (uint8_t i = 1; i <= 3; i++)
    {
        for (uint8_t j = 1; j <= 3; j++)
        {
            for (uint8_t k = 1; k <= 3; k++)
                res[i][j] += a[i][k] * b[k][j];
        }
    }
}

static void Matrix_Reduce_3x1(float *a, float *b, float *res)
{
    res[1] = a[1] - b[1];
    res[2] = a[2] - b[2];
    res[3] = a[3] - b[3];
}

static void Matrix_T_3x3(float a[4][4], float res[4][4])
{
    for (uint8_t i = 1; i <= 3; i++)
    {
        for (uint8_t j = 1; j <= 3; j++)
            res[i][j] = a[j][i];
    }
}

static void Matrix_1x3_Cross_Product_3x1(float *a, float *b, float *res)
{
    *res = a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

static bool Matrix_3x3_Det(float in[4][4], float *res)
{
    float a = in[1][1], b = in[1][2], c = in[1][3];
    float d = in[2][1], e = in[2][2], f = in[2][3];
    float g = in[3][1], h = in[3][2], i = in[3][3];

    *res = a * (e * i - f * h) -
           b * (d * i - f * g) +
           c * (d * h - e * g);
    return fabsf(*res) >= 0.001f;
}

static void Matrix_3x3_Inv(float in[4][4], float det, float inv[4][4])
{
    float a = in[1][1], b = in[1][2], c = in[1][3];
    float d = in[2][1], e = in[2][2], f = in[2][3];
    float g = in[3][1], h = in[3][2], i = in[3][3];

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

static void Matrix_3x3_Take_Negative(float in[4][4], float out[4][4])
{
    for (uint8_t i = 1; i <= 3; i++)
    {
        for (uint8_t j = 1; j <= 3; j++)
            out[i][j] = -in[i][j];
    }
}

static void Matrix_3x3_Cross_Product_3x1(const float a[4][4], const float *b, float *res)
{
    res[0] = 0.0f;
    for (uint8_t i = 1; i <= 3; i++)
    {
        res[i] = 0.0f;
        for (uint8_t j = 1; j <= 3; j++)
            res[i] += a[i][j] * b[j];
    }
}

bool Kinematic_ControllerDeltaFK(const DOF6Kinematic_Handle_t *handle, const float theta_rad[3], float xyz_m[3])
{
    if (handle == NULL || theta_rad == NULL || xyz_m == NULL)
        return false;
    return Controller_DeltaFK_Internal(&handle->config.controller_config, theta_rad, xyz_m);
}

bool Kinematic_ControllerGravityCompensation(const DOF6Kinematic_Handle_t *handle,
                                             const float theta_rad[3],
                                             float torque[3],
                                             float xyz_m[3])
{
    if (handle == NULL || theta_rad == NULL || torque == NULL || xyz_m == NULL)
        return false;

    const ControllerKinematicConfig_t *cfg = &handle->config.controller_config;
    float target[3] = {0.0f};

    if (!Controller_DeltaFK_Internal(cfg, theta_rad, target))
        return false;

    xyz_m[0] = target[0];
    xyz_m[1] = target[1];
    xyz_m[2] = target[2];

    float target_1based[4] = {0.0f, target[0], target[1], target[2]};
    float phi[4] = {0.0f, 0.0f, 2.0f * (float)M_PI / 3.0f, 4.0f * (float)M_PI / 3.0f};
    float gravity[4] = {0.0f, -13.0f, 0.0f, 0.0f};
    float Jacobi[4][4] = {{0.0f}};

    float R1[4][4] = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, cosf(phi[1]), -sinf(phi[1]), 0.0f},
        {0.0f, sinf(phi[1]), cosf(phi[1]), 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}};
    float R2[4][4] = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, cosf(phi[2]), -sinf(phi[2]), 0.0f},
        {0.0f, sinf(phi[2]), cosf(phi[2]), 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}};
    float R3[4][4] = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, cosf(phi[3]), -sinf(phi[3]), 0.0f},
        {0.0f, sinf(phi[3]), cosf(phi[3]), 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}};

    float temp1[4] = {0.0f, cfg->delta_L_m * cosf(theta_rad[0]) + cfg->delta_R_m - cfg->delta_r_m, 0.0f, -cfg->delta_L_m * sinf(theta_rad[0])};
    float temp2[4] = {0.0f, cfg->delta_L_m * cosf(theta_rad[1]) + cfg->delta_R_m - cfg->delta_r_m, 0.0f, -cfg->delta_L_m * sinf(theta_rad[1])};
    float temp3[4] = {0.0f, cfg->delta_L_m * cosf(theta_rad[2]) + cfg->delta_R_m - cfg->delta_r_m, 0.0f, -cfg->delta_L_m * sinf(theta_rad[2])};
    float t1[4] = {0.0f};
    float t2[4] = {0.0f};
    float t3[4] = {0.0f};
    float S1[4] = {0.0f};
    float S2[4] = {0.0f};
    float S3[4] = {0.0f};

    Matrix_3x3_Cross_Product_3x1(R1, temp1, t1);
    Matrix_3x3_Cross_Product_3x1(R2, temp2, t2);
    Matrix_3x3_Cross_Product_3x1(R3, temp3, t3);
    Matrix_Reduce_3x1(target_1based, t1, S1);
    Matrix_Reduce_3x1(target_1based, t2, S2);
    Matrix_Reduce_3x1(target_1based, t3, S3);

    float b1_src[4] = {0.0f, -cfg->delta_L_m * sinf(theta_rad[0]), 0.0f, -cfg->delta_L_m * cosf(theta_rad[0])};
    float b2_src[4] = {0.0f, -cfg->delta_L_m * sinf(theta_rad[1]), 0.0f, -cfg->delta_L_m * cosf(theta_rad[1])};
    float b3_src[4] = {0.0f, -cfg->delta_L_m * sinf(theta_rad[2]), 0.0f, -cfg->delta_L_m * cosf(theta_rad[2])};
    float b1[4] = {0.0f};
    float b2[4] = {0.0f};
    float b3[4] = {0.0f};
    Matrix_3x3_Cross_Product_3x1(R1, b1_src, b1);
    Matrix_3x3_Cross_Product_3x1(R2, b2_src, b2);
    Matrix_3x3_Cross_Product_3x1(R3, b3_src, b3);

    float S1b1 = 0.0f;
    float S2b2 = 0.0f;
    float S3b3 = 0.0f;
    Matrix_1x3_Cross_Product_3x1(S1, b1, &S1b1);
    Matrix_1x3_Cross_Product_3x1(S2, b2, &S2b2);
    Matrix_1x3_Cross_Product_3x1(S3, b3, &S3b3);

    float Sb[4][4] = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, S1b1, 0.0f, 0.0f},
        {0.0f, 0.0f, S2b2, 0.0f},
        {0.0f, 0.0f, 0.0f, S3b3}};
    float S_T[4][4] = {
        {0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, S1[1], S1[2], S1[3]},
        {0.0f, S2[1], S2[2], S2[3]},
        {0.0f, S3[1], S3[2], S3[3]},
    };

    float S_det = 0.0f;
    float S_T_inv[4][4] = {{0.0f}};
    if (Matrix_3x3_Det(S_T, &S_det))
    {
        float Jacobi_temp[4][4] = {{0.0f}};
        Matrix_3x3_Inv(S_T, S_det, S_T_inv);
        Matrix_3_Cross_Product_3(S_T_inv, Sb, Jacobi_temp);
        Matrix_3x3_Take_Negative(Jacobi_temp, Jacobi);
    }

    float Jacobo_T[4][4] = {{0.0f}};
    float torque_1based[4] = {0.0f};
    Matrix_T_3x3(Jacobi, Jacobo_T);
    Matrix_3x3_Cross_Product_3x1(Jacobo_T, gravity, torque_1based);

    torque[0] = -torque_1based[1];
    torque[1] = -torque_1based[2];
    torque[2] = -torque_1based[3];
    return true;
}

float Kinematic_Scale_Controller_Pitch(const DOF6Kinematic_Handle_t *handle, float pitch_deg)
{
    float limit = 60.0f;
    if (handle != NULL && handle->config.controller_config.pitch_limit_deg > KINEMATIC_EPSILON)
        limit = handle->config.controller_config.pitch_limit_deg;

    pitch_deg = Kinematic_Clamp(pitch_deg, -limit, limit);
    return pitch_deg / limit * 90.0f;
}

void Kinematic_Build_Rotation_ZYX(float yaw_deg, float pitch_deg, float roll_deg, float R[9])
{
    if (R == NULL)
        return;

    float yaw = yaw_deg * KINEMATIC_DEG_TO_RAD;
    float pitch = pitch_deg * KINEMATIC_DEG_TO_RAD;
    float roll = roll_deg * KINEMATIC_DEG_TO_RAD;

    float cy = cosf(yaw);
    float sy = sinf(yaw);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cr = cosf(roll);
    float sr = sinf(roll);

    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;
    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;
    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}

void Kinematic_Build_Pose(float x_mm,
                          float y_mm,
                          float z_mm,
                          float roll_deg,
                          float pitch_deg,
                          float yaw_deg,
                          Pose6D_t *pose)
{
    if (pose == NULL)
        return;

    pose->X = x_mm;
    pose->Y = y_mm;
    pose->Z = z_mm;
    pose->A = roll_deg;
    pose->B = pitch_deg;
    pose->C = yaw_deg;
    pose->hasR = true;
    Kinematic_Build_Rotation_ZYX(yaw_deg, pitch_deg, roll_deg, pose->R);
}

static void Map_DeltaXYZ_To_Pose(const ControllerKinematicConfig_t *cfg, const float xyz_m[3], float pose_xyz_m[3])
{
    float x = -(xyz_m[2] - cfg->xyz_delta_error_m[2]);
    float y = xyz_m[1] - cfg->xyz_delta_error_m[1];
    float z = xyz_m[0] - cfg->xyz_delta_error_m[0];

    x += cfg->xyz_arm_error_m[0];
    y += cfg->xyz_arm_error_m[1];
    z += cfg->xyz_arm_error_m[2];

    x *= cfg->xyz_scale[0];
    y *= cfg->xyz_scale[1];
    z *= cfg->xyz_scale[2];
    y *= -1.0f;

    pose_xyz_m[0] = x;
    pose_xyz_m[1] = y;
    pose_xyz_m[2] = z;
}

bool Kinematic_ControllerFK(const DOF6Kinematic_Handle_t *handle,
                            const ControllerInput_t *input,
                            Pose6D_t *pose,
                            float torque[3],
                            float xyz_m[3])
{
    if (handle == NULL || input == NULL || pose == NULL || !handle->initialized)
        return false;

    const ControllerKinematicConfig_t *cfg = &handle->config.controller_config;
    float theta_rad[3] = {
        -input->q1_rad - cfg->q1_offset_deg * KINEMATIC_DEG_TO_RAD,
        input->q2_rad - cfg->q2_initial_deg * KINEMATIC_DEG_TO_RAD - cfg->q2_offset_deg * KINEMATIC_DEG_TO_RAD,
        -input->q3_rad + cfg->q3_initial_deg * KINEMATIC_DEG_TO_RAD - cfg->q3_offset_deg * KINEMATIC_DEG_TO_RAD,
    };
    float local_torque[3] = {0.0f};
    float local_xyz_m[3] = {0.0f};

    if (!Kinematic_ControllerGravityCompensation(handle, theta_rad, local_torque, local_xyz_m))
        return false;

    if (theta_rad[0] > cfg->q1_max_rad - cfg->q1_offset_deg * KINEMATIC_DEG_TO_RAD)
        local_torque[0] = cfg->q1_limit_torque;
    if (theta_rad[1] > (cfg->q2_max_deg - cfg->q2_initial_deg - cfg->q2_offset_deg) * KINEMATIC_DEG_TO_RAD)
        local_torque[1] *= 0.3f;
    if (theta_rad[2] > (-(cfg->q3_max_deg - cfg->q3_initial_deg) - cfg->q3_offset_deg) * KINEMATIC_DEG_TO_RAD)
        local_torque[2] *= 0.3f;

    float pose_xyz_m[3] = {0.0f};
    Map_DeltaXYZ_To_Pose(cfg, local_xyz_m, pose_xyz_m);

    float roll = input->imu_roll_deg + 180.0f;
    float pitch = -Kinematic_Scale_Controller_Pitch(handle, input->imu_pitch_deg) + 90.0f;
    float yaw = input->imu_yaw_deg;
    Kinematic_Build_Pose(pose_xyz_m[0] * CONTROLLER_POSE_SCALE_MM,
                         pose_xyz_m[1] * CONTROLLER_POSE_SCALE_MM,
                         pose_xyz_m[2] * CONTROLLER_POSE_SCALE_MM,
                         roll,
                         pitch,
                         yaw,
                         pose);

    if (torque != NULL)
    {
        torque[0] = local_torque[0];
        torque[1] = local_torque[1];
        torque[2] = local_torque[2];
    }
    if (xyz_m != NULL)
    {
        xyz_m[0] = local_xyz_m[0];
        xyz_m[1] = local_xyz_m[1];
        xyz_m[2] = local_xyz_m[2];
    }

    return true;
}

bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle,
                       const Joint6D_t *inputJoints,
                       Pose6D_t *outputPose)
{
    if (handle == NULL || inputJoints == NULL || outputPose == NULL || !handle->initialized)
        return false;

    DHParameters dh;
    JointAngles joint;
    Config_To_DH(&handle->config, &dh);
    for (uint8_t i = 0; i < 6; i++)
        joint.theta[i] = Deg_To_Rad(inputJoints->a[i]);

    EndEffectorState state = Forward_Kinematics(&joint, &dh);
    EulerAngles euler = Rotation_Matrix_To_Euler(&state.rotation);

    outputPose->X = (float)(state.position.x * handle->config.dh_to_pose_scale);
    outputPose->Y = (float)(state.position.y * handle->config.dh_to_pose_scale);
    outputPose->Z = (float)(state.position.z * handle->config.dh_to_pose_scale);
    outputPose->A = (float)Rad_To_Deg(euler.roll);
    outputPose->B = (float)Rad_To_Deg(euler.pitch);
    outputPose->C = (float)Rad_To_Deg(euler.yaw);
    outputPose->hasR = true;
    for (uint8_t r = 0; r < 3; r++)
    {
        for (uint8_t c = 0; c < 3; c++)
            outputPose->R[r * 3 + c] = (float)state.rotation.m[r][c];
    }
    return true;
}

static void Pose_To_Vector_Rotation(const DOF6Kinematic_Handle_t *handle, const Pose6D_t *pose, Vector3 *position, Matrix3x3 *rotation)
{
    position->x = pose->X * handle->config.pose_to_dh_scale;
    position->y = pose->Y * handle->config.pose_to_dh_scale;
    position->z = pose->Z * handle->config.pose_to_dh_scale;

    if (pose->hasR)
    {
        for (uint8_t r = 0; r < 3; r++)
        {
            for (uint8_t c = 0; c < 3; c++)
                rotation->m[r][c] = pose->R[r * 3 + c];
        }
    }
    else
    {
        *rotation = Euler_To_Rotation_Matrix(Deg_To_Rad(pose->A), Deg_To_Rad(pose->B), Deg_To_Rad(pose->C));
    }
}

bool Kinematic_SolveIK(const DOF6Kinematic_Handle_t *handle,
                       const Pose6D_t *inputPose,
                       const Joint6D_t *lastJoints,
                       IKSolves_t *outputSolves)
{
    (void)lastJoints;

    if (handle == NULL || inputPose == NULL || outputSolves == NULL || !handle->initialized)
        return false;

    memset(outputSolves, 0, sizeof(IKSolves_t));

    DHParameters dh;
    Vector3 position;
    Matrix3x3 rotation;
    int flag = handle->flag_beyond_180;
    Config_To_DH(&handle->config, &dh);
    Pose_To_Vector_Rotation(handle, inputPose, &position, &rotation);

    InverseKinematicsSolutions solves = Inverse_Kinematics_From_Rotation_Matrix(&position,
                                                                                &rotation,
                                                                                &dh,
                                                                                &flag,
                                                                                handle->config.enable_q3_beyond_180_patch);
    if (solves.count <= 0)
        return false;

    for (int i = 0; i < solves.count && i < IK_SOLVE_COUNT; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
            outputSolves->config[i].a[j] = (float)Rad_To_Deg(solves.angles[i].theta[j]);
        outputSolves->solFlag[i][0] = 1;
    }
    return true;
}

static void Joint6D_To_JointAngles(const Joint6D_t *joint_deg, JointAngles *joint_rad)
{
    for (uint8_t i = 0; i < 6; i++)
        joint_rad->theta[i] = Deg_To_Rad(joint_deg->a[i]);
}

static void IKSolves_To_Internal(const IKSolves_t *solves, InverseKinematicsSolutions *internal)
{
    internal->count = 0;
    for (uint8_t i = 0; i < IK_SOLVE_COUNT; i++)
    {
        if (solves->solFlag[i][0] <= 0)
            continue;
        for (uint8_t j = 0; j < 6; j++)
            internal->angles[internal->count].theta[j] = Deg_To_Rad(solves->config[i].a[j]);
        internal->count++;
    }
}

static bool Kinematic_Is_Candidate_In_Limits(const DOF6Kinematic_Handle_t *handle, const JointAngles *candidate)
{
    if (handle == NULL || candidate == NULL)
        return false;

    Joint6D_t ik_joint_deg;
    Joint6D_t motor_joint_deg;
    for (uint8_t j = 0U; j < 6U; j++)
    {
        if (!Kinematic_Double_Is_Finite(candidate->theta[j]))
            return false;

        ik_joint_deg.a[j] = (float)Rad_To_Deg(candidate->theta[j]);
        if (!Kinematic_Is_Angle_In_Limit(ik_joint_deg.a[j], &handle->config.joint_limit[j]))
            return false;
    }

    Apply_Affine_Map(&handle->config.joint_to_motor, &ik_joint_deg, &motor_joint_deg);
    return Kinematic_Is_Motor_In_Limits(&handle->config, &motor_joint_deg) &&
           Kinematic_Is_Coupling_In_Limits(&handle->config, &motor_joint_deg);
}

int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle,
                              const IKSolves_t *solves,
                              const Joint6D_t *current_joints)
{
    if (handle == NULL || solves == NULL || current_joints == NULL)
        return -1;

    InverseKinematicsSolutions internal;
    JointAngles reference;
    IKSolves_To_Internal(solves, &internal);
    Joint6D_To_JointAngles(current_joints, &reference);

    if (internal.count == 0)
        return -1;

    double min_cost = DBL_MAX;
    int best_index = -1;
    double max_jump = handle->config.max_single_joint_jump_rad > 0.0f ? handle->config.max_single_joint_jump_rad : 6.0;
    double max_total_cost = handle->config.max_total_cost > 0.0f ? handle->config.max_total_cost : 20.0;

    for (int i = 0; i < internal.count; i++)
    {
        if (!Kinematic_Is_Candidate_In_Limits(handle, &internal.angles[i]))
            continue;

        double cost = 0.0;
        for (int j = 0; j < 6; j++)
        {
            double diff = internal.angles[i].theta[j] - reference.theta[j];
            if (internal.angles[i].theta[j] > Deg_To_Rad(300.0))
            {
                cost = DBL_MAX;
                break;
            }
            if (diff > M_PI)
                diff -= 2.0 * M_PI;
            if (diff < -M_PI)
                diff += 2.0 * M_PI;

            double weight = handle->config.select_weight[j] > 0.0f ? handle->config.select_weight[j] : (7.0 - j);
            cost += weight * fabs(diff);
            if (diff >= max_jump || diff <= -max_jump)
                cost = DBL_MAX;
        }

        if (cost < min_cost)
        {
            min_cost = cost;
            best_index = i;
        }
    }

    if (min_cost > max_total_cost)
        return 255;
    return best_index;
}

static void Apply_Affine_Map(const AffineAngleMap_t *map, const Joint6D_t *input, Joint6D_t *output)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        float value = map->bias_deg[i];
        for (uint8_t j = 0; j < 6; j++)
            value += map->map[i][j] * input->a[j];
        output->a[i] = value;
    }
}

void Kinematic_IKAngle_To_Motor(const DOF6Kinematic_Handle_t *handle,
                                const Joint6D_t *ik_joint_deg,
                                Joint6D_t *motor_joint_deg)
{
    if (handle == NULL || ik_joint_deg == NULL || motor_joint_deg == NULL)
        return;

    Apply_Affine_Map(&handle->config.joint_to_motor, ik_joint_deg, motor_joint_deg);
}

void Kinematic_Motor_To_IKAngle(const DOF6Kinematic_Handle_t *handle,
                                const Joint6D_t *motor_joint_deg,
                                Joint6D_t *ik_joint_deg)
{
    if (handle == NULL || motor_joint_deg == NULL || ik_joint_deg == NULL)
        return;

    Apply_Affine_Map(&handle->config.motor_to_joint, motor_joint_deg, ik_joint_deg);
}

static void Apply_Motor_Limit_By_Index(const ArmConfig_t *config, Joint6D_t *motor_joint_deg, uint8_t index)
{
    if (index >= 6 || !config->motor_limit[index].enable)
        return;

    motor_joint_deg->a[index] = Kinematic_Clamp(motor_joint_deg->a[index],
                                                config->motor_limit[index].min_deg,
                                                config->motor_limit[index].max_deg);
}

void Kinematic_Limit_Motor_Angle(const DOF6Kinematic_Handle_t *handle, Joint6D_t *motor_joint_deg)
{
    if (handle == NULL || motor_joint_deg == NULL)
        return;

    const ArmConfig_t *config = &handle->config;

    Apply_Motor_Limit_By_Index(config, motor_joint_deg, 1U);
    Apply_Motor_Limit_By_Index(config, motor_joint_deg, 2U);

    for (uint8_t i = 0; i < config->coupling_limit_count && i < 4U; i++)
    {
        const MotorCouplingLimit_t *limit = &config->coupling_limits[i];
        if (!limit->enable || limit->lhs_motor_index >= 6 || limit->rhs_motor_index >= 6)
            continue;

        float delta = motor_joint_deg->a[limit->lhs_motor_index] - motor_joint_deg->a[limit->rhs_motor_index];
        if (delta > limit->max_delta_deg)
            motor_joint_deg->a[limit->lhs_motor_index] = limit->max_delta_deg + motor_joint_deg->a[limit->rhs_motor_index];
        if (delta < limit->min_delta_deg)
            motor_joint_deg->a[limit->lhs_motor_index] = limit->min_delta_deg + motor_joint_deg->a[limit->rhs_motor_index];
    }

    Apply_Motor_Limit_By_Index(config, motor_joint_deg, 4U);
    for (uint8_t i = 0; i < 6; i++)
    {
        if (i != 1U && i != 2U && i != 4U)
            Apply_Motor_Limit_By_Index(config, motor_joint_deg, i);
    }
}

bool Kinematic_SolveIK_To_Motor(DOF6Kinematic_Handle_t *handle,
                                const Pose6D_t *target_pose,
                                Joint6D_t *last_valid_ik_joint_deg,
                                Joint6D_t *motor_joint_deg,
                                int *best_index)
{
    if (handle == NULL || target_pose == NULL || last_valid_ik_joint_deg == NULL || motor_joint_deg == NULL || !handle->initialized)
        return false;

    DHParameters dh;
    Vector3 position;
    Matrix3x3 rotation;
    Config_To_DH(&handle->config, &dh);
    Pose_To_Vector_Rotation(handle, target_pose, &position, &rotation);

    InverseKinematicsSolutions internal_solves = Inverse_Kinematics_From_Rotation_Matrix(&position,
                                                                                         &rotation,
                                                                                         &dh,
                                                                                         &handle->flag_beyond_180,
                                                                                         handle->config.enable_q3_beyond_180_patch);
    IKSolves_t public_solves;
    memset(&public_solves, 0, sizeof(public_solves));
    for (int i = 0; i < internal_solves.count && i < IK_SOLVE_COUNT; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
            public_solves.config[i].a[j] = (float)Rad_To_Deg(internal_solves.angles[i].theta[j]);
        public_solves.solFlag[i][0] = 1;
    }

    int selected = Kinematic_Select_Best_Sol(handle, &public_solves, last_valid_ik_joint_deg);
    if (best_index != NULL)
        *best_index = selected;

    JointAngles selected_joint;
    bool solved = true;
    if (selected == -1 || selected == 255 || selected >= internal_solves.count)
    {
        for (uint8_t i = 0; i < 6; i++)
            selected_joint.theta[i] = handle->current_ik_rad[i];
        solved = false;
    }
    else
    {
        selected_joint = internal_solves.angles[selected];
        JointAngles reference;
        for (uint8_t i = 0; i < 6; i++)
            reference.theta[i] = handle->current_ik_rad[i];
        Fit_Angle_Solution(&selected_joint, &reference);
        for (uint8_t i = 0; i < 6; i++)
        {
            handle->current_ik_rad[i] = selected_joint.theta[i];
            handle->last_answer_rad[i] = selected_joint.theta[i];
        }
    }

    for (uint8_t i = 0; i < 6; i++)
        last_valid_ik_joint_deg->a[i] = (float)Rad_To_Deg(selected_joint.theta[i]);

    Kinematic_IKAngle_To_Motor(handle, last_valid_ik_joint_deg, motor_joint_deg);
    Kinematic_Limit_Motor_Angle(handle, motor_joint_deg);
    return solved;
}
