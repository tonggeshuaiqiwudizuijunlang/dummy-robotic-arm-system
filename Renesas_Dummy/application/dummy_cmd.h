#ifndef DUMMY_CMD_H
#define DUMMY_CMD_H

#include <stdint.h>
#include "motor_def.h"
#include "robot_types.h"

#define DUMMY_CMD_UART_MODE_BLUETOOTH      0
#define DUMMY_CMD_UART_MODE_SERIAL_DEBUG   1

#ifndef DUMMY_CMD_UART_MODE
#define DUMMY_CMD_UART_MODE DUMMY_CMD_UART_MODE_BLUETOOTH
#endif

#ifndef DUMMY_CMD_FKIK_TEST_ENABLE
#define DUMMY_CMD_FKIK_TEST_ENABLE 0
#endif

#ifndef SERIAL_DEBUG_SEND_DIV
#define SERIAL_DEBUG_SEND_DIV 4U
#endif

#ifndef DUMMY_CMD_FKIK_TEST_SEND_DIV
#define DUMMY_CMD_FKIK_TEST_SEND_DIV SERIAL_DEBUG_SEND_DIV
#endif

#ifndef DUMMY_CMD_REMOTE_POSE_TEST_ENABLE
#define DUMMY_CMD_REMOTE_POSE_TEST_ENABLE 0u
#endif

#ifndef REMOTE_POSE_POS_GAIN
#define REMOTE_POSE_POS_GAIN 0.001f
#endif

#ifndef REMOTE_POSE_ATT_GAIN
#define REMOTE_POSE_ATT_GAIN 0.00005f
#endif

#ifndef BT_POSE_POS_SCALE
#define BT_POSE_POS_SCALE 0.5f
#endif

#ifndef BT_POSE_ATT_FALLBACK_ENABLE
#define BT_POSE_ATT_FALLBACK_ENABLE 1u
#endif

#ifndef DUMMY_CMD_NONBLOCKING_FINISH_FEEDBACK_ENABLE
#define DUMMY_CMD_NONBLOCKING_FINISH_FEEDBACK_ENABLE 1u
#endif

#ifndef DUMMY_CMD_WORKSPACE_X_MIN
#define DUMMY_CMD_WORKSPACE_X_MIN (-450.0f)
#endif

#ifndef DUMMY_CMD_WORKSPACE_X_MAX
#define DUMMY_CMD_WORKSPACE_X_MAX 450.0f
#endif

#ifndef DUMMY_CMD_WORKSPACE_Y_MIN
#define DUMMY_CMD_WORKSPACE_Y_MIN (-450.0f)
#endif

#ifndef DUMMY_CMD_WORKSPACE_Y_MAX
#define DUMMY_CMD_WORKSPACE_Y_MAX 450.0f
#endif

#ifndef DUMMY_CMD_WORKSPACE_Z_MIN
#define DUMMY_CMD_WORKSPACE_Z_MIN (-120.0f)
#endif

#ifndef DUMMY_CMD_WORKSPACE_Z_MAX
#define DUMMY_CMD_WORKSPACE_Z_MAX 650.0f
#endif

#ifndef DUMMY_CMD_POSE_ATT_MIN
#define DUMMY_CMD_POSE_ATT_MIN (-180.0f)
#endif

#ifndef DUMMY_CMD_POSE_ATT_MAX
#define DUMMY_CMD_POSE_ATT_MAX 180.0f
#endif

#define DUMMY_CMD_BT_STATUS_IDLE      0u
#define DUMMY_CMD_BT_STATUS_OK        1u
#define DUMMY_CMD_BT_STATUS_NO_HANDLE 2u
#define DUMMY_CMD_BT_STATUS_NO_FRAME  3u
#define DUMMY_CMD_BT_STATUS_IK_FAIL   4u

typedef struct {
    uint32_t bt_branch_count;
    uint32_t bt_ik_ok_count;
    uint32_t bt_ik_fail_count;
    uint8_t bt_last_status;
    uint8_t bt_last_header;
    uint16_t bt_last_tailer;
    uint16_t last_switch_l2;
    uint16_t last_switch_r1;
    uint16_t last_switch_r2;
    float bt_target_x;
    float bt_target_y;
    float bt_target_z;
} DummyCmd_Debug_s;

extern volatile DummyCmd_Debug_s dummy_cmd_debug;



void DummyCmd_Init(void);
void DummyCmd_Task(void);


#endif
