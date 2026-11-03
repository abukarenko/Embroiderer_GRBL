#ifndef GRBL_PLATFORM_H
#define GRBL_PLATFORM_H

#include "main.h"
#include <stdint.h>

void grbl_platform_init(void);
void grbl_platform_task(void);

void grbl_platform_stepper_init(void);
void grbl_platform_stepper_enable(uint8_t disable);
void grbl_platform_step_timer_start(uint16_t period, uint16_t prescaler);
void grbl_platform_step_timer_stop(void);
void grbl_platform_pulse_timer_start(uint16_t period);
void grbl_platform_set_step_outputs(uint8_t bits);
void grbl_platform_set_direction_outputs(uint8_t bits);

uint8_t grbl_platform_limits_state(void);
uint8_t grbl_platform_probe_state(void);
uint8_t grbl_platform_control_state(void);

void grbl_platform_spindle_init(void);
void grbl_platform_spindle_set_enable(uint8_t enabled);
void grbl_platform_spindle_set_direction(uint8_t ccw);
void grbl_platform_spindle_set_pwm(uint16_t pwm);
void grbl_platform_spindle_set_minimum_percent(uint8_t percent);
uint8_t grbl_platform_spindle_get_enable(void);
uint8_t grbl_platform_spindle_get_direction(void);

void grbl_platform_coolant_init(void);
void grbl_platform_coolant_set_flood(uint8_t enabled);
void grbl_platform_coolant_set_mist(uint8_t enabled);
uint8_t grbl_platform_coolant_get_flood(void);
uint8_t grbl_platform_coolant_get_mist(void);

void grbl_serial_receive(const uint8_t *data, uint32_t length);
void grbl_serial_flush(void);

#endif
