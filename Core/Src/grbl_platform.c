#include "grbl_platform.h"
#include "board_test.h"
#include "grbl.h"

static uint32_t grbl_display_sample_tick;

#define PROBE_RESUME_DEBOUNCE_MS 30U

static uint8_t probe_resume_armed;
static uint8_t probe_resume_debouncing;
static uint32_t probe_resume_debounce_tick;

static void grbl_platform_probe_resume_task(void)
{
  uint8_t active;
  uint8_t m0_hold;

  if (!settings.probe_resume_enable) {
    probe_resume_armed = 0U;
    probe_resume_debouncing = 0U;
    return;
  }

  active = probe_get_state();
  m0_hold = (sys.state == STATE_HOLD) &&
            (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) &&
            (sys.suspend & SUSPEND_HOLD_COMPLETE);

  if (!m0_hold) {
    probe_resume_armed = 0U;
    probe_resume_debouncing = 0U;
    return;
  }

  /* Require release after entering HOLD. A permanently active input must not
     immediately resume a newly reached M0 pause. */
  if (!active) {
    probe_resume_armed = 1U;
    probe_resume_debouncing = 0U;
    return;
  }

  if (!probe_resume_armed) { return; }

  if (!probe_resume_debouncing) {
    probe_resume_debouncing = 1U;
    probe_resume_debounce_tick = HAL_GetTick();
    return;
  }

  if ((uint32_t)(HAL_GetTick() - probe_resume_debounce_tick) >=
      PROBE_RESUME_DEBOUNCE_MS) {
    probe_resume_armed = 0U;
    probe_resume_debouncing = 0U;
    system_set_exec_state_flag(EXEC_CYCLE_START);
  }
}

static const char *grbl_state_text(uint8_t state)
{
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

  grbl_platform_probe_resume_task();

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
  BoardTest_SetSpindlePwm(pwm);
}

void grbl_platform_spindle_set_minimum_percent(uint8_t percent)
{
  if (percent > 100U) { percent = 100U; }
  BoardTest_SetSpindlePwmMinimum((uint16_t)percent * 10U);
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
