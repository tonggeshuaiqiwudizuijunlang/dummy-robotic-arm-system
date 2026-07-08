#ifndef DUMMY_MOTORMATIC_H
#define DUMMY_MOTORMATIC_H

#include "robot_types.h"
#include "dummy_kinematic_v2.h"
#include "flysky.h"

#define REMOTE_JOINT_GAIN 0.0003f
#define REMOTE_KNOB_RANGE ((float)(FS_DATA_DOWN - FS_DATA_MIN))
#define REMOTE_KNOB_HALF (REMOTE_KNOB_RANGE / 2.0f)

#define ARM_HOME_JOINT1 0.0f
#define ARM_HOME_JOINT2 0.0f
#define ARM_HOME_JOINT3 0.0f
#define ARM_HOME_JOINT4 0.0f
#define ARM_HOME_JOINT5 0.0f
#define ARM_HOME_JOINT6 0.0f

#define ARM_SEVEN_JOINT1 0.0f
#define ARM_SEVEN_JOINT2 75.0f
#define ARM_SEVEN_JOINT3 90.0f
#define ARM_SEVEN_JOINT4 0.0f
#define ARM_SEVEN_JOINT5 0.0f
#define ARM_SEVEN_JOINT6 0.0f

#ifndef DUMMY_GRIPPER_CALIB_ENABLE
#define DUMMY_GRIPPER_CALIB_ENABLE 1u
#endif

#ifndef DUMMY_GRIPPER_CALIB_BLOCK_ARM_ENABLE
#define DUMMY_GRIPPER_CALIB_BLOCK_ARM_ENABLE 0u
#endif

#ifndef DUMMY_GRIPPER_STOP_THRESHOLD_CURRENT
#define DUMMY_GRIPPER_STOP_THRESHOLD_CURRENT 0.25f
#endif




void Dummy_Motormatic_Init(void);
void Dummy_Motormatic_Task(void);

DOF6Kinematic_Handle_t *Dummy_Motormatic_GetKinematicHandle(void);

#endif
