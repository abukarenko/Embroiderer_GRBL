#include "grbl_platform.h"
#include "board_test.h"
#include "grbl.h"

static uint32_t grbl_display_sample_tick;
static uint8_t embroidery_waiting;
static uint8_t embroidery_armed;
static uint8_t embroidery_permit;
static volatile uint8_t embroidery_between_stitches;
static volatile uint8_t embroidery_trimming;
static uint32_t embroidery_trim_started;

uint16_t embroidery_speed_for_distance_squared(float distance_squared)
{
  return embroidery_speed_pwm(distance_squared);
}

uint8_t embroidery_needle_down(void)
{
  /* Logical active means needle in fabric. Preserve GRBL's $6 inversion
     so an optocoupler's polarity can be calibrated without rebuilding. */
  return probe_get_state();
}

uint8_t embroidery_hold_status(void)
{
  return ((sys.state == STATE_IDLE) &&
          (embroidery_waiting || embroidery_between_stitches)) ||
         (((sys.state == STATE_CYCLE) || (sys.state == STATE_JOG)) &&
          embroidery_needle_down());
}

uint8_t embroidery_accessory_busy(void)
{
  return embroidery_waiting || embroidery_trimming;
}

static void embroidery_sensor_task(void)
{
  if (embroidery_trimming &&
      ((uint32_t)(HAL_GetTick() - embroidery_trim_started) >= EMBROIDERY_TRIM_MS ||
       sys.state != STATE_IDLE || sys.abort)) {
    grbl_platform_spindle_set_pwm(0U);
    grbl_platform_spindle_set_enable(0U);
  }
  /* Never use needle cycles to release an operator's Feed Hold/M0. */
  if (!embroidery_waiting || sys.state != STATE_IDLE || sys.suspend) {
    embroidery_armed = 0U;
    return;
  }
  if (embroidery_needle_down()) { embroidery_armed = 1U; }
  else if (embroidery_armed) {
    embroidery_permit = 1U;
    embroidery_armed = 0U;
  }
}

uint8_t embroidery_wait_stitch(void)
{
  embroidery_waiting = 1U;
  embroidery_armed = 0U;
  embroidery_permit = 0U;
  while (!embroidery_permit) {
    protocol_execute_realtime();
    if (sys.abort) { embroidery_waiting = 0U; return 0U; }
  }
  embroidery_waiting = 0U;
  embroidery_between_stitches = 0U;
  return 1U;
}

void embroidery_stitch_complete(void)
{
  embroidery_between_stitches = 1U;
}

uint8_t embroidery_finish_stitch(void)
{
  if (embroidery_between_stitches && grbl_platform_spindle_get_enable()) {
    return embroidery_wait_stitch();
  }
  return !sys.abort;
}

void embroidery_trim(void)
{
  uint32_t started;
  if (sys.state == STATE_CHECK_MODE || sys.abort) { return; }
  protocol_buffer_synchronize();
  if (sys.abort) { return; }
  /* The last XY move precedes its needle penetration. Let that needle
     cycle finish before requesting thread trim. */
  if (!embroidery_finish_stitch()) { return; }
  spindle_stop();
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (sys.abort || (sys_rt_exec_state & EXEC_RESET)) {
    if (!primask) { __enable_irq(); }
    return;
  }
  embroidery_trim_started = HAL_GetTick();
  embroidery_trimming = 1U;
  BoardTest_SetSpindlePwmMinimum(0U);
  BoardTest_SetSpindlePwm(EMBROIDERY_TRIM_PWM);
  if (!primask) { __enable_irq(); }
  started = embroidery_trim_started;
  while ((uint32_t)(HAL_GetTick() - started) < EMBROIDERY_TRIM_MS) {
    protocol_execute_realtime();
    if (sys.abort || !embroidery_trimming) { break; }
  }
  embroidery_trimming = 0U;
  grbl_platform_spindle_set_pwm(0U);
  grbl_platform_spindle_set_enable(0U);
}

static const char *grbl_state_text(uint8_t state)
{
  if (embroidery_hold_status()) { return "HOLD"; }
  if (state & STATE_ALARM) { return "ALARM"; }
  if (state & STATE_HOMING) { return "HOME"; }
  if (state & STATE_CYCLE) { return "RUN"; }
  if (state & STATE_HOLD) { return "HOLD"; }
  if (state & STATE_JOG) { return "JOG"; }
  if (state & STATE_SAFETY_DOOR) { return "DOOR"; }
  if (state & STATE_CHECK_MODE) { return "CHECK"; }
  if (state & STATE_SLEEP) { return "SLEEP"; }
  return "IDLE";
}

static void write_axis_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
  HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void grbl_platform_update_display(void)
{
  int32_t position_steps[N_AXIS];
  float machine_position[N_AXIS];
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  memcpy(position_steps, sys_position, sizeof(position_steps));
  if (!primask) { __enable_irq(); }
  system_convert_array_steps_to_mpos(machine_position, position_steps);
  BoardTest_SetGrblStatus(machine_position, sys.spindle_speed,
                          limits_get_state(), probe_get_state(),
                          grbl_state_text(sys.state));
}

void grbl_platform_init(void)
{
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2U, 0U);
}

void grbl_platform_task(void)
{
  uint32_t now = HAL_GetTick();

  embroidery_sensor_task();

  if ((uint32_t)(now - grbl_display_sample_tick) >= 1000U) {
    grbl_display_sample_tick = now;
    grbl_platform_update_display();
  }
  BoardTest_Task();
  grbl_serial_flush();
}

void grbl_platform_stepper_init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  TIM2->CR1 = 0U;
  TIM2->PSC = 0U;
  TIM2->ARR = 0xFFFFU;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->SR = 0U;

  TIM3->CR1 = TIM_CR1_OPM;
  TIM3->PSC = 0U;
  TIM3->ARR = 1U;
  TIM3->DIER = TIM_DIER_UIE;
  TIM3->SR = 0U;

  HAL_NVIC_SetPriority(TIM2_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0U, 1U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  grbl_platform_set_step_outputs(0U);
  grbl_platform_set_direction_outputs(0U);
  grbl_platform_stepper_enable(1U);
}

void grbl_platform_stepper_enable(uint8_t disable)
{
  write_axis_pin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin, disable);
}

void grbl_platform_step_timer_start(uint16_t period, uint16_t prescaler)
{
  if (period == 0U) {
    period = 1U;
  }
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM2->PSC = prescaler;
  TIM2->ARR = (uint16_t)(period - 1U);
  TIM2->CNT = 0U;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void grbl_platform_step_timer_stop(void)
{
  TIM2->CR1 &= ~TIM_CR1_CEN;
  TIM2->SR = 0U;
}

void grbl_platform_pulse_timer_start(uint16_t period)
{
  if (period == 0U) {
    period = 1U;
  }
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->PSC = 0U;
  TIM3->ARR = (uint16_t)(period - 1U);
  TIM3->CNT = 0U;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0U;
  TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
}

void grbl_platform_set_step_outputs(uint8_t bits)
{
  uint32_t gpioa_set = 0U;
  uint32_t gpioa_reset = 0U;

  GPIOB->BSRR = (bits & (1U << 0)) ? X_STEP_Pin : ((uint32_t)X_STEP_Pin << 16U);
  GPIOC->BSRR = (bits & (1U << 1)) ? Y_STEP_Pin : ((uint32_t)Y_STEP_Pin << 16U);
  if (bits & (1U << 2)) { gpioa_set |= Z_STEP_Pin; }
  else { gpioa_reset |= Z_STEP_Pin; }
  if (bits & (1U << 3)) { gpioa_set |= A_STEP_Pin; }
  else { gpioa_reset |= A_STEP_Pin; }
  GPIOA->BSRR = gpioa_set | (gpioa_reset << 16U);
}

void grbl_platform_set_direction_outputs(uint8_t bits)
{
  uint32_t gpioa_set = 0U;
  uint32_t gpioa_reset = 0U;

  GPIOB->BSRR = (bits & (1U << 0)) ? X_DIR_Pin : ((uint32_t)X_DIR_Pin << 16U);
  GPIOC->BSRR = (bits & (1U << 1)) ? Y_DIR_Pin : ((uint32_t)Y_DIR_Pin << 16U);
  if (bits & (1U << 2)) { gpioa_set |= Z_DIR_Pin; }
  else { gpioa_reset |= Z_DIR_Pin; }
  if (bits & (1U << 3)) { gpioa_set |= A_DIR_Pin; }
  else { gpioa_reset |= A_DIR_Pin; }
  GPIOA->BSRR = gpioa_set | (gpioa_reset << 16U);
}

uint8_t grbl_platform_limits_state(void)
{
  uint8_t state = 0U;
  if (HAL_GPIO_ReadPin(X_LIMIT_GPIO_Port, X_LIMIT_Pin) == GPIO_PIN_SET) {
    state |= (1U << 0);
  }
  if (HAL_GPIO_ReadPin(Y_LIMIT_GPIO_Port, Y_LIMIT_Pin) == GPIO_PIN_SET) {
    state |= (1U << 1);
  }
  if (HAL_GPIO_ReadPin(Z_LIMIT_GPIO_Port, Z_LIMIT_Pin) == GPIO_PIN_SET) {
    state |= (1U << 2);
  }
  return state;
}

uint8_t grbl_platform_probe_state(void)
{
  return HAL_GPIO_ReadPin(PROBE_GPIO_Port, PROBE_Pin) == GPIO_PIN_SET;
}

uint8_t grbl_platform_control_state(void)
{
  return 0U;
}

void grbl_platform_spindle_init(void)
{
  embroidery_waiting = embroidery_armed = embroidery_permit = 0U;
  embroidery_between_stitches = embroidery_trimming = 0U;
  grbl_platform_spindle_set_minimum_percent(settings.spindle_pwm_min_percent);
  BoardTest_SetSpindlePwm(SPINDLE_PWM_OFF_VALUE);
  grbl_platform_spindle_set_enable(0U);
  grbl_platform_spindle_set_direction(0U);
}

void grbl_platform_spindle_set_enable(uint8_t enabled)
{
  write_axis_pin(OUT1_GPIO_Port, OUT1_Pin, enabled);
}

void grbl_platform_spindle_set_direction(uint8_t ccw)
{
  write_axis_pin(OUT2_GPIO_Port, OUT2_Pin, ccw);
}

void grbl_platform_spindle_set_pwm(uint16_t pwm)
{
  /* A stop/reset always exits trim and restores neutral, even from an ISR. */
  if (pwm == 0U) {
    embroidery_trimming = 0U;
    embroidery_between_stitches = 0U;
    pwm = EMBROIDERY_NEUTRAL_PWM;
  }
  BoardTest_SetSpindlePwmMinimum(EMBROIDERY_NEUTRAL_PWM);
  if (pwm > 860U) { pwm = 860U; }
  BoardTest_SetSpindlePwm(pwm);
}

void grbl_platform_spindle_set_minimum_percent(uint8_t percent)
{
  (void)percent;
  BoardTest_SetSpindlePwmMinimum(EMBROIDERY_NEUTRAL_PWM);
}

uint8_t grbl_platform_spindle_get_enable(void)
{
  return HAL_GPIO_ReadPin(OUT1_GPIO_Port, OUT1_Pin) == GPIO_PIN_SET;
}

uint8_t grbl_platform_spindle_get_direction(void)
{
  return HAL_GPIO_ReadPin(OUT2_GPIO_Port, OUT2_Pin) == GPIO_PIN_SET;
}

void grbl_platform_coolant_init(void)
{
  grbl_platform_coolant_set_flood(0U);
  grbl_platform_coolant_set_mist(0U);
}

void grbl_platform_coolant_set_flood(uint8_t enabled)
{
  write_axis_pin(OUT3_GPIO_Port, OUT3_Pin, enabled);
}

void grbl_platform_coolant_set_mist(uint8_t enabled)
{
  write_axis_pin(OUT4_GPIO_Port, OUT4_Pin, enabled);
}

uint8_t grbl_platform_coolant_get_flood(void)
{
  return HAL_GPIO_ReadPin(OUT3_GPIO_Port, OUT3_Pin) == GPIO_PIN_SET;
}

uint8_t grbl_platform_coolant_get_mist(void)
{
  return HAL_GPIO_ReadPin(OUT4_GPIO_Port, OUT4_Pin) == GPIO_PIN_SET;
}
