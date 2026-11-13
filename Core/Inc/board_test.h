#ifndef BOARD_TEST_H
#define BOARD_TEST_H

#include "main.h"
#include <stdint.h>

typedef enum {
  BOARD_TOUCH_ACTION_NONE = 0,
  BOARD_TOUCH_ACTION_SPINDLE_TOGGLE,
  BOARD_TOUCH_ACTION_RESET_UNLOCK
} BoardTouchAction;

void BoardTest_Init(void);
void BoardTest_Task(void);
void BoardTest_OnUsbRx(uint8_t *data, uint32_t len);
void BoardTest_SetUsbPortOpen(uint8_t is_open);
uint8_t BoardTest_IsUsbPortOpen(void);
BoardTouchAction BoardTest_TakeTouchAction(void);
void BoardTest_UsbPullupInit(void);
void BoardTest_UsbDisconnect(void);
void BoardTest_UsbConnect(void);
void BoardTest_PwmInit(void);
void BoardTest_SetSpindlePwm(uint16_t permille);
void BoardTest_SetSpindlePwmMinimum(uint16_t permille);
void BoardTest_SetSpindlePwmNeutral(uint16_t permille);
void BoardTest_IdcInit(void);
void BoardTest_SetGrblStatus(const float machine_position[4],
                             float spindle_speed, uint8_t limit_state,
                             uint8_t probe_state, const char *state_text);

#endif
