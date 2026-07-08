#ifndef DUMMY_CMD_H
#define DUMMY_CMD_H

#include "robot_types.h"

#define DUMMY_CMD_OUTPUT_MODE_SERIAL_DEBUG 0U
#define DUMMY_CMD_OUTPUT_MODE_BT_POSE      1U

#ifndef DUMMY_CMD_OUTPUT_MODE
#define DUMMY_CMD_OUTPUT_MODE DUMMY_CMD_OUTPUT_MODE_BT_POSE
#endif

#ifndef SERIAL_DEBUG_SEND_DIV
#define SERIAL_DEBUG_SEND_DIV 5U
#endif

#ifndef DUMMY_CMD_SERIAL_DEBUG_CHANNELS
#define DUMMY_CMD_SERIAL_DEBUG_CHANNELS 12U
#endif

void DummyCmd_Init(void);
void DummyCmd_Task(void);


#endif
