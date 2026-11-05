/*
  main.c - GRBL application entry adapted to the existing STM32Cube HAL project.
  Based on GRBL 1.1f from usbcnc/grbl (GPLv3).
*/

#include "grbl.h"

system_t sys;
int32_t sys_position[N_AXIS];
int32_t sys_probe_position[N_AXIS];
volatile uint8_t sys_probe_state;
volatile uint8_t sys_rt_exec_state;
volatile uint8_t sys_rt_exec_alarm;
volatile uint8_t sys_rt_exec_motion_override;
volatile uint8_t sys_rt_exec_accessory_override;
#ifdef DEBUG
volatile uint8_t sys_rt_exec_debug;
#endif

void Grbl_Run(void)
{
  grbl_platform_init();
  eeprom_init();
  serial_init();
  settings_init();
  /* Preserve motion calibration while overriding CNC-only pedal settings. */
  settings.spindle_pwm_min_percent = 50U;
  settings.probe_resume_enable = 0U;
  settings.flags &= ~BITFLAG_LASER_MODE;
  stepper_init();
  system_init();

  memset(sys_position, 0, sizeof(sys_position));

#ifdef FORCE_INITIALIZATION_ALARM
  sys.state = STATE_ALARM;
#else
  sys.state = STATE_IDLE;
#endif

#ifdef HOMING_INIT_LOCK
  if (bit_istrue(settings.flags, BITFLAG_HOMING_ENABLE)) {
    sys.state = STATE_ALARM;
  }
#endif

  for (;;) {
    uint8_t prior_state = sys.state;

    memset(&sys, 0, sizeof(system_t));
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;
    sys.r_override = DEFAULT_RAPID_OVERRIDE;
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE;
    memset(sys_probe_position, 0, sizeof(sys_probe_position));
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    serial_reset_read_buffer();
    gc_init();
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset();
    st_reset();
    plan_sync_position();
    gc_sync_position();
    report_init_message();
    protocol_main_loop();
  }
}
