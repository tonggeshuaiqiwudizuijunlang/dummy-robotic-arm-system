#ifndef ARM_MOVIET_H
#define ARM_MOVIET_H

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// ==================== 结构体定义 ====================


/** 6自由度关节角度 (弧度) */
typedef struct {
    double theta[6];
} JointAngles;

/** D-H参数 */
typedef struct {
    double d[6];      // 连杆偏距
    double a[6];      // 连杆长度
    double alpha[6];  // 连杆扭角
} DHParameters;

/** 四元数 */
typedef struct {
    double w, x, y, z;
} Quaternion;

/** 欧拉角 (RPY表示) */
typedef struct {
    double roll, pitch, yaw;  // 绕X, Y, Z轴的旋转
} EulerAngles;

/** 3D向量 */
typedef struct {
    double x, y, z;
} Vector3;

/** 4x4齐次变换矩阵 */
typedef struct {
    double m[4][4];
} Matrix4x4;

/** 3x3旋转矩阵 */
typedef struct {
    double m[3][3];
} Matrix3x3;


/** 机器人末端位姿完整描述 */
typedef struct {
    Vector3 position;
    Matrix3x3 rotation;  // 旋转矩阵
    Matrix4x4 transform; // 齐次变换矩阵
} EndEffectorState;

/** 机器人末端位姿 */
typedef struct {
    Vector3 position;     // 位置 (x, y, z)
    EulerAngles orientation;  // 姿态 (欧拉角)
} EndEffectorPose;
// ==================== 逆运动学 ====================

/** 逆运动学解结构 */
typedef struct {
    JointAngles angles[8];  // 最多8组解
    int count;              // 有效解的数量
} InverseKinematicsSolutions;

EndEffectorState forward_kinematics(const JointAngles* angles, const DHParameters* dh);
// InverseKinematicsSolutions inverse_kinematics_from_rotation_matrix(
//     const Vector3* position,
//     const Matrix3x3* rotation,
//     const DHParameters* dh);
InverseKinematicsSolutions inverse_kinematics_from_pose(
    const Vector3* position,
    const EulerAngles* orientation,
    const DHParameters* dh);
InverseKinematicsSolutions inverse_kinematics_from_rotation_matrix(
    const Vector3* position,
    const Matrix3x3* rotation,
    const DHParameters* dh);
int select_best_solution(const InverseKinematicsSolutions* solutions, const JointAngles* reference);
double verify_solution(const JointAngles* angles, 
                       const Vector3* expected_position,
                       const EulerAngles* expected_orientation,
                       const DHParameters* dh);
void fit_angle_solution(JointAngles *solutions, JointAngles *reference);
void Rotation_4x4_To_3x3(float rotation[4][4], Matrix3x3 *__end_rotation);
#endif