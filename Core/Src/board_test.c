#include "board_test.h"
#include "grbl.h"
#include "xpt2046.h"

#include "usbd_cdc_if.h"
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  const char *name;
} BoardPin;

typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t step_pin;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  char axis;
} AxisPins;

typedef struct {
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  const char *label;
  uint32_t color;
} TftButton;

#define USB_PULLUP_GPIO_Port GPIOC
#define USB_PULLUP_Pin GPIO_PIN_11
#define USB_PULLUP_ACTIVE_STATE GPIO_PIN_RESET
#define USB_PULLUP_INACTIVE_STATE GPIO_PIN_SET

#define LCD_CLK_GPIO_Port GPIOB
#define LCD_CLK_Pin GPIO_PIN_8
#define LCD_DIN_GPIO_Port GPIOB
#define LCD_DIN_Pin GPIO_PIN_9
#define LCD_CE_GPIO_Port GPIOB
#define LCD_CE_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_2

#define LCD_WIDTH 84U
#define LCD_ROWS 6U
#define LCD_DIAGNOSTIC_PHASE_MS 1500U
#define LCD_PIN_DIAGNOSTIC_MODE 0U
#define LCD_PIN_PULSE_MS 100U
#define LCD_PIN_GROUP_GAP_MS 1000U
#define LCD_SPI_DELAY_LOOPS 100U
#define TFT_ILI9488_MODE 1U
#define TFT_WIDTH 480U
#define TFT_HEIGHT 320U
#define TOUCH_POLL_PERIOD_MS 25U

#define HEARTBEAT_PERIOD_MS 1000U
#define HEARTBEAT_FIRST_PULSE_END_MS 80U
#define HEARTBEAT_SECOND_PULSE_START_MS 180U
#define HEARTBEAT_SECOND_PULSE_END_MS 260U

static const BoardPin outputs[] = {
    {OUT1_GPIO_Port, OUT1_Pin, "OUT1"},
    {OUT2_GPIO_Port, OUT2_Pin, "OUT2"},
    {OUT3_GPIO_Port, OUT3_Pin, "OUT3"},
    {OUT4_GPIO_Port, OUT4_Pin, "OUT4"},
};

static const BoardPin inputs[] = {
    {X_LIMIT_GPIO_Port, X_LIMIT_Pin, "X_LIMIT/IN1"},
    {Y_LIMIT_GPIO_Port, Y_LIMIT_Pin, "Y_LIMIT/IN2"},
    {Z_LIMIT_GPIO_Port, Z_LIMIT_Pin, "Z_LIMIT/IN3"},
    {PROBE_GPIO_Port, PROBE_Pin, "PROBE/IN4"},
    {GPIOB, GPIO_PIN_4, "IDC7/PB4"},
    {GPIOB, GPIO_PIN_3, "IDC8/PB3"},
    {GPIOC, GPIO_PIN_12, "IDC10/PC12"},
};

static const AxisPins axes[] = {
    {X_STEP_GPIO_Port, X_STEP_Pin, X_DIR_GPIO_Port, X_DIR_Pin, 'x'},
    {Y_STEP_GPIO_Port, Y_STEP_Pin, Y_DIR_GPIO_Port, Y_DIR_Pin, 'y'},
    {Z_STEP_GPIO_Port, Z_STEP_Pin, Z_DIR_GPIO_Port, Z_DIR_Pin, 'z'},
    {A_STEP_GPIO_Port, A_STEP_Pin, A_DIR_GPIO_Port, A_DIR_Pin, 'a'},
};

/* Fixed touch-button hit areas and their enabled colors. */
static const TftButton tft_buttons[] = {
    {10U, 246U, 119U, 316U, "FLUID", 0x86C7D8U},
    {127U, 246U, 236U, 316U, "SPINDLE", 0xD99A94U},
    {244U, 246U, 353U, 316U, "PAUSE", 0xFFC000U},
    {361U, 246U, 470U, 316U, "UNLOCK", 0xFF1010U},
};

static const BoardPin lcd_diagnostic_pins[] = {
    {LCD_RST_GPIO_Port, LCD_RST_Pin, "RST"},
    {LCD_CE_GPIO_Port, LCD_CE_Pin, "CE"},
    {LCD_DC_GPIO_Port, LCD_DC_Pin, "DC"},
    {LCD_DIN_GPIO_Port, LCD_DIN_Pin, "DIN"},
    {LCD_CLK_GPIO_Port, LCD_CLK_Pin, "CLK"},
};

static char rx_line[96];
static char command_line[96];
static uint32_t rx_len;
static volatile uint8_t command_ready;
static volatile uint8_t usb_port_open;
static uint16_t spindle_pwm_permille;
static uint16_t spindle_pwm_requested_permille;
static uint16_t spindle_pwm_minimum_permille =
    DEFAULT_SPINDLE_PWM_MIN_PERCENT * 10U;
static uint8_t lcd_usb_state = 0xFFU;
static uint8_t lcd_diagnostic_phase = 3U;
static uint8_t lcd_diagnostic_byte;
static uint32_t lcd_diagnostic_tick;
static float grbl_machine_position[4];
static float tft_last_machine_position[4] = {
    1.0e30f, 1.0e30f, 1.0e30f, 1.0e30f};
static float grbl_spindle_speed;
static uint8_t grbl_limit_state;
static uint8_t grbl_probe_state;
static char grbl_state_text[12] = "START";
static char tft_last_axis_text[4][10] = {
    "    +0.00", "    +0.00", "    +0.00", "    +0.00"};
static char tft_last_spindle_text[19];
static char tft_last_state_text[12] = "START";
static char tft_state_redraw_text[12];
static uint32_t tft_state_redraw_color;
static uint8_t tft_state_redraw_phase;
static uint8_t tft_state_redraw_slice;
static uint8_t tft_state_redraw_character;
static uint8_t tft_last_input_state;
static volatile uint8_t grbl_status_pending;
static uint32_t tft_grbl_update_tick;
static uint8_t tft_grbl_update_field;
static uint8_t lcd_pin_diagnostic_index;
static uint8_t lcd_pin_diagnostic_count;
static uint8_t lcd_pin_diagnostic_state;
static uint32_t lcd_pin_diagnostic_tick;
static uint8_t tft_usb_state = 0xFFU;
static uint16_t tft_last_pwm;
static uint32_t touch_poll_tick;
static int8_t touch_active_button = -1;
static uint8_t touch_press_latched;
static volatile BoardTouchAction touch_action_pending;
static uint8_t tft_button_state_mask;
static uint8_t tft_button_redraw_active;
static uint8_t tft_button_redraw_index;
static uint8_t tft_button_redraw_target;
static uint8_t tft_button_redraw_slice;
static uint8_t tft_button_redraw_character;
static uint32_t tft_button_update_tick;

static const uint8_t lcd_font_digits[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
};

static const uint8_t lcd_font_upper[26][5] = {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U},
};

static void lcd_spi_delay(void) {
  for (volatile uint32_t delay = 0U; delay < LCD_SPI_DELAY_LOOPS; delay++) {
    __NOP();
  }
}

static void lcd_write_byte(uint8_t is_data, uint8_t value) {
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin,
                    is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);

  for (uint8_t bit = 0U; bit < 8U; bit++) {
    HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DIN_GPIO_Port, LCD_DIN_Pin,
                      (value & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    lcd_spi_delay();
    HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_SET);
    lcd_spi_delay();
    value <<= 1U;
  }

  HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
}

static void lcd_command(uint8_t command) {
  lcd_write_byte(0U, command);
}

static void lcd_data(uint8_t data) {
  lcd_write_byte(1U, data);
}

static void lcd_set_position(uint8_t x, uint8_t row) {
  lcd_command((uint8_t)(0x80U | x));
  lcd_command((uint8_t)(0x40U | row));
}

static void lcd_clear(void) {
  lcd_set_position(0U, 0U);
  for (uint16_t i = 0U; i < (LCD_WIDTH * LCD_ROWS); i++) {
    lcd_data(0x00U);
  }
  lcd_set_position(0U, 0U);
}

static void lcd_fill_checkerboard(void) {
  lcd_command(0x0CU);
  lcd_set_position(0U, 0U);

  for (uint8_t row = 0U; row < LCD_ROWS; row++) {
    for (uint8_t x = 0U; x < LCD_WIDTH; x++) {
      lcd_data(((x + row) & 1U) ? 0xAAU : 0x55U);
    }
  }

  lcd_set_position(0U, 0U);
}

static const uint8_t *lcd_get_glyph(char c) {
  static const uint8_t glyph_space[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t glyph_colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
  static const uint8_t glyph_dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t glyph_dot[5] = {0U, 0x60U, 0x60U, 0U, 0U};
  static const uint8_t glyph_plus[5] = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
  static const uint8_t glyph_question[5] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};

  if (c >= 'a' && c <= 'z') {
    c = (char)(c - ('a' - 'A'));
  }
  if (c >= '0' && c <= '9') {
    return lcd_font_digits[(uint8_t)(c - '0')];
  }
  if (c >= 'A' && c <= 'Z') {
    return lcd_font_upper[(uint8_t)(c - 'A')];
  }

  switch (c) {
    case ' ':
      return glyph_space;
    case ':':
      return glyph_colon;
    case '-':
      return glyph_dash;
    case '.':
      return glyph_dot;
    case '+':
      return glyph_plus;
    default:
      return glyph_question;
  }
}

static void lcd_write_line(uint8_t row, const char *text) {
  lcd_set_position(0U, row);

  for (uint8_t column = 0U; column < 14U; column++) {
    char c = (*text != '\0') ? *text++ : ' ';
    const uint8_t *glyph = lcd_get_glyph(c);
    for (uint8_t i = 0U; i < 5U; i++) {
      lcd_data(glyph[i]);
    }
    lcd_data(0x00U);
  }
}

static void lcd_show_status(void) {
  lcd_write_line(0U, "MASH3 GRBL");
  lcd_write_line(1U, "NOKIA 5110");
  lcd_write_line(2U, "PCD8544 READY");
  lcd_write_line(3U, usb_port_open ? "USB: OPEN" : "USB: WAIT");
  lcd_write_line(4U, usb_port_open ? "LED: ON" : "LED: HEART");
  lcd_write_line(5U, "DISPLAY TEST");
  lcd_usb_state = usb_port_open;
}

static void lcd_init(void) {
  HAL_Delay(250U);

  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DIN_GPIO_Port, LCD_DIN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);

  lcd_command(0x21U);
  lcd_command(0xBFU);
  lcd_command(0x04U);
  lcd_command(0x14U);
  lcd_command(0x20U);
  lcd_command(0x0CU);

  lcd_clear();
  lcd_show_status();
  lcd_diagnostic_phase = 3U;
  lcd_diagnostic_byte = 0x55U;
  lcd_diagnostic_tick = HAL_GetTick();
}

static void lcd_screen_diagnostic_task(void) {
  uint32_t now = HAL_GetTick();

  if (lcd_diagnostic_phase == 2U) {
    lcd_data(lcd_diagnostic_byte);
    lcd_diagnostic_byte ^= 0xFFU;
  }

  if ((now - lcd_diagnostic_tick) < LCD_DIAGNOSTIC_PHASE_MS) {
    return;
  }

  lcd_diagnostic_tick = now;
  lcd_diagnostic_phase = (uint8_t)((lcd_diagnostic_phase + 1U) & 0x03U);

  switch (lcd_diagnostic_phase) {
    case 0U:
      lcd_command(0x09U);
      break;
    case 1U:
      lcd_command(0x08U);
      break;
    case 2U:
      lcd_fill_checkerboard();
      break;
    default:
      lcd_command(0x0CU);
      lcd_clear();
      lcd_show_status();
      break;
  }
}

static void lcd_pin_diagnostic_init(void) {
  for (uint8_t i = 0U;
       i < (sizeof(lcd_diagnostic_pins) / sizeof(lcd_diagnostic_pins[0]));
       i++) {
    HAL_GPIO_WritePin(lcd_diagnostic_pins[i].port,
                      lcd_diagnostic_pins[i].pin,
                      GPIO_PIN_RESET);
  }

  lcd_pin_diagnostic_index = 0U;
  lcd_pin_diagnostic_count = 0U;
  lcd_pin_diagnostic_state = 0U;
  lcd_pin_diagnostic_tick = HAL_GetTick();
}

static void lcd_pin_diagnostic_task(void) {
  const BoardPin *pin = &lcd_diagnostic_pins[lcd_pin_diagnostic_index];
  uint32_t now = HAL_GetTick();

  if (lcd_pin_diagnostic_state == 0U) {
    if ((now - lcd_pin_diagnostic_tick) < LCD_PIN_GROUP_GAP_MS) {
      return;
    }

    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_SET);
    lcd_pin_diagnostic_state = 1U;
    lcd_pin_diagnostic_tick = now;
    return;
  }

  if ((now - lcd_pin_diagnostic_tick) < LCD_PIN_PULSE_MS) {
    return;
  }

  if (lcd_pin_diagnostic_state == 1U) {
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_RESET);
    lcd_pin_diagnostic_state = 2U;
    lcd_pin_diagnostic_tick = now;
    return;
  }

  lcd_pin_diagnostic_count++;
  if (lcd_pin_diagnostic_count >= (lcd_pin_diagnostic_index + 1U)) {
    lcd_pin_diagnostic_count = 0U;
    lcd_pin_diagnostic_index++;
    if (lcd_pin_diagnostic_index >=
        (sizeof(lcd_diagnostic_pins) / sizeof(lcd_diagnostic_pins[0]))) {
      lcd_pin_diagnostic_index = 0U;
    }
    lcd_pin_diagnostic_state = 0U;
  } else {
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_SET);
    lcd_pin_diagnostic_state = 1U;
  }
  lcd_pin_diagnostic_tick = now;
}

#if defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

#if defined(__GNUC__)
#define TFT_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define TFT_ALWAYS_INLINE inline
#endif

/* PB8 (CLK) and PB9 (MOSI) share GPIOB. Updating both pins with one BSRR
   write and unrolling the byte removes the O0 loop and delay overhead from
   the display path. The XPT2046 keeps its separate, slower SPI timing. */
static TFT_ALWAYS_INLINE void tft_write_bit(uint8_t value, uint8_t mask) {
  uint32_t clock_low_and_data = (uint32_t)LCD_CLK_Pin << 16U;

  if ((value & mask) != 0U) {
    clock_low_and_data |= LCD_DIN_Pin;
  } else {
    clock_low_and_data |= (uint32_t)LCD_DIN_Pin << 16U;
  }

  LCD_CLK_GPIO_Port->BSRR = clock_low_and_data;
  LCD_CLK_GPIO_Port->BSRR = LCD_CLK_Pin;
}

static TFT_ALWAYS_INLINE void tft_write8(uint8_t value) {
  tft_write_bit(value, 0x80U);
  tft_write_bit(value, 0x40U);
  tft_write_bit(value, 0x20U);
  tft_write_bit(value, 0x10U);
  tft_write_bit(value, 0x08U);
  tft_write_bit(value, 0x04U);
  tft_write_bit(value, 0x02U);
  tft_write_bit(value, 0x01U);
  LCD_CLK_GPIO_Port->BSRR = (uint32_t)LCD_CLK_Pin << 16U;
}

static void tft_write_command(uint8_t command) {
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);
  tft_write8(command);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
}

static void tft_write_data(uint8_t data) {
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);
  tft_write8(data);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
}

static void tft_set_window(uint16_t x1, uint16_t y1,
                           uint16_t x2, uint16_t y2) {
  tft_write_command(0x2AU);
  tft_write_data((uint8_t)(x1 >> 8U));
  tft_write_data((uint8_t)x1);
  tft_write_data((uint8_t)(x2 >> 8U));
  tft_write_data((uint8_t)x2);

  tft_write_command(0x2BU);
  tft_write_data((uint8_t)(y1 >> 8U));
  tft_write_data((uint8_t)y1);
  tft_write_data((uint8_t)(y2 >> 8U));
  tft_write_data((uint8_t)y2);

  tft_write_command(0x2CU);
}

static void tft_fill_rectangle(uint16_t x1, uint16_t y1,
                               uint16_t x2, uint16_t y2,
                               uint8_t red, uint8_t green, uint8_t blue) {
  uint32_t pixels = (uint32_t)(x2 - x1) * (uint32_t)(y2 - y1);

  tft_set_window(x1, y1, (uint16_t)(x2 - 1U), (uint16_t)(y2 - 1U));
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);

  while (pixels-- != 0U) {
    tft_write8(red);
    tft_write8(green);
    tft_write8(blue);
  }

  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
}

static void tft_init(void) {
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DIN_GPIO_Port, LCD_DIN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(500U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(500U);

  tft_write_command(0xE0U);
  const uint8_t gamma_positive[] = {
      0x00U, 0x13U, 0x18U, 0x04U, 0x0FU, 0x06U, 0x3AU, 0x56U,
      0x4DU, 0x03U, 0x0AU, 0x06U, 0x30U, 0x3EU, 0x0FU};
  for (uint8_t i = 0U; i < sizeof(gamma_positive); i++) {
    tft_write_data(gamma_positive[i]);
  }

  tft_write_command(0xE1U);
  const uint8_t gamma_negative[] = {
      0x00U, 0x13U, 0x18U, 0x01U, 0x11U, 0x06U, 0x38U, 0x34U,
      0x4DU, 0x06U, 0x0DU, 0x0BU, 0x31U, 0x37U, 0x0FU};
  for (uint8_t i = 0U; i < sizeof(gamma_negative); i++) {
    tft_write_data(gamma_negative[i]);
  }

  tft_write_command(0xC0U);
  tft_write_data(0x18U);
  tft_write_data(0x16U);
  tft_write_command(0xC1U);
  tft_write_data(0x45U);
  tft_write_command(0xC5U);
  tft_write_data(0x00U);
  tft_write_data(0x63U);
  tft_write_data(0x01U);
  tft_write_command(0x36U);
  tft_write_data(0x38U);
  tft_write_command(0x3AU);
  tft_write_data(0x66U);
  tft_write_command(0xB0U);
  tft_write_data(0x80U);
  tft_write_command(0xB1U);
  /* ILI9488 normal-mode frame rate: about 68-70 Hz.
     The previous 0x00/0x10 setting was close to 30 Hz and visibly flickered. */
  tft_write_data(0xB0U);
  tft_write_data(0x11U);
  tft_write_command(0xB4U);
  tft_write_data(0x02U);
  tft_write_command(0xB6U);
  tft_write_data(0x02U);
  tft_write_command(0xE9U);
  tft_write_data(0x00U);
  tft_write_command(0xF7U);
  tft_write_data(0xA9U);
  tft_write_data(0x51U);
  tft_write_data(0x2CU);
  tft_write_data(0x82U);
  tft_write_command(0x11U);
  HAL_Delay(120U);
  tft_write_command(0x20U);
  HAL_Delay(120U);
  tft_write_command(0x29U);
}

static TFT_ALWAYS_INLINE void tft_stream_rgb(uint32_t rgb) {
  tft_write8((uint8_t)((rgb >> 16U) & 0xFCU));
  tft_write8((uint8_t)((rgb >> 8U) & 0xFCU));
  tft_write8((uint8_t)(rgb & 0xFCU));
}

static void tft_draw_char(uint16_t x, uint16_t y, char c, uint8_t scale,
                          uint32_t foreground, uint32_t background) {
  const uint8_t *glyph = lcd_get_glyph(c);
  uint16_t width = (uint16_t)(6U * scale);
  uint16_t height = (uint16_t)(8U * scale);

  tft_set_window(x, y, (uint16_t)(x + width - 1U),
                 (uint16_t)(y + height - 1U));
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_RESET);

  for (uint8_t row = 0U; row < 8U; row++) {
    for (uint8_t repeat_y = 0U; repeat_y < scale; repeat_y++) {
      for (uint8_t column = 0U; column < 6U; column++) {
        uint8_t pixel_on =
            (column < 5U) && ((glyph[column] & (1U << row)) != 0U);
        for (uint8_t repeat_x = 0U; repeat_x < scale; repeat_x++) {
          tft_stream_rgb(pixel_on ? foreground : background);
        }
      }
    }
  }

  HAL_GPIO_WritePin(LCD_CE_GPIO_Port, LCD_CE_Pin, GPIO_PIN_SET);
}

static void tft_draw_text(uint16_t x, uint16_t y, const char *text,
                          uint8_t scale, uint32_t foreground,
                          uint32_t background) {
  while (*text != '\0') {
    tft_draw_char(x, y, *text++, scale, foreground, background);
    x = (uint16_t)(x + (6U * scale));
  }
}

static void tft_fill_rgb(uint16_t x1, uint16_t y1,
                         uint16_t x2, uint16_t y2, uint32_t color) {
  tft_fill_rectangle(x1, y1, x2, y2,
                     (uint8_t)(color >> 16U),
                     (uint8_t)(color >> 8U),
                     (uint8_t)color);
}

static void tft_draw_box(uint16_t x1, uint16_t y1,
                         uint16_t x2, uint16_t y2, uint32_t color) {
  tft_fill_rgb(x1, y1, x2, y2, 0x000000U);
  tft_fill_rgb((uint16_t)(x1 + 2U), (uint16_t)(y1 + 2U),
               (uint16_t)(x2 - 2U), (uint16_t)(y2 - 2U), color);
}

static void tft_draw_centered_text(uint16_t x1, uint16_t y1,
                                   uint16_t x2, uint16_t y2,
                                   const char *text, uint8_t scale,
                                   uint32_t foreground, uint32_t background) {
  uint16_t width = (uint16_t)(strlen(text) * 6U * scale);
  uint16_t height = (uint16_t)(8U * scale);
  uint16_t x = (uint16_t)(x1 + ((x2 - x1 - width) / 2U));
  uint16_t y = (uint16_t)(y1 + ((y2 - y1 - height) / 2U));
  tft_draw_text(x, y, text, scale, foreground, background);
}

static void tft_draw_button_border(uint8_t button, uint32_t color) {
  const TftButton *item = &tft_buttons[button];

  tft_fill_rgb(item->x1, item->y1, item->x2,
               (uint16_t)(item->y1 + 2U), color);
  tft_fill_rgb(item->x1, (uint16_t)(item->y2 - 2U), item->x2,
               item->y2, color);
  tft_fill_rgb(item->x1, (uint16_t)(item->y1 + 2U),
               (uint16_t)(item->x1 + 2U), (uint16_t)(item->y2 - 2U), color);
  tft_fill_rgb((uint16_t)(item->x2 - 2U), (uint16_t)(item->y1 + 2U),
               item->x2, (uint16_t)(item->y2 - 2U), color);
}

static uint32_t tft_get_button_color(uint8_t button, uint8_t enabled) {
  uint32_t color = tft_buttons[button].color;
  if (enabled) {
    return color;
  }
  return (((color >> 16U) & 0xFFU) / 4U << 16U) |
         (((color >> 8U) & 0xFFU) / 4U << 8U) |
         ((color & 0xFFU) / 4U);
}

static const char *tft_get_button_label(uint8_t button, uint8_t enabled) {
  if (button == 2U && enabled) {
    return "RESUME";
  }
  return tft_buttons[button].label;
}

static uint8_t tft_get_button_enabled(uint8_t button) {
  switch (button) {
    case 0U:
      return HAL_GPIO_ReadPin(OUT3_GPIO_Port, OUT3_Pin) == GPIO_PIN_SET;
    case 1U:
      return HAL_GPIO_ReadPin(OUT1_GPIO_Port, OUT1_Pin) == GPIO_PIN_SET;
    case 2U:
      return (sys.state & STATE_HOLD) != 0U;
    case 3U:
      return touch_active_button == 3;
    default:
      return 0U;
  }
}

static void tft_draw_button_face(uint8_t button, uint8_t enabled) {
  const TftButton *item = &tft_buttons[button];
  uint32_t color = tft_get_button_color(button, enabled);
  const char *label = tft_get_button_label(button, enabled);

  tft_draw_box(item->x1, item->y1, item->x2, item->y2, color);
  tft_draw_centered_text((uint16_t)(item->x1 + 2U),
                         (uint16_t)(item->y1 + 2U),
                         (uint16_t)(item->x2 - 2U),
                         (uint16_t)(item->y2 - 2U),
                         label, 2U, 0xFFFFFFU, color);
}

static void tft_update_button_faces(void) {
  const TftButton *item;
  const char *label;
  uint32_t color;
  uint16_t y1;
  uint16_t y2;

  if (!tft_button_redraw_active) {
    for (uint8_t button = 0U;
         button < (uint8_t)(sizeof(tft_buttons) / sizeof(tft_buttons[0]));
         button++) {
      uint8_t enabled = tft_get_button_enabled(button);
      uint8_t stored = (tft_button_state_mask >> button) & 1U;
      if (enabled != stored) {
        tft_button_redraw_active = 1U;
        tft_button_redraw_index = button;
        tft_button_redraw_target = enabled;
        tft_button_redraw_slice = 0U;
        tft_button_redraw_character = 0U;
        break;
      }
    }
    if (!tft_button_redraw_active) {
      return;
    }
  }

  item = &tft_buttons[tft_button_redraw_index];
  color = tft_get_button_color(tft_button_redraw_index,
                               tft_button_redraw_target);
  y1 = (uint16_t)(item->y1 + 2U + 4U * tft_button_redraw_slice);
  if (y1 < (uint16_t)(item->y2 - 2U)) {
    y2 = (uint16_t)(y1 + 4U);
    if (y2 > (uint16_t)(item->y2 - 2U)) {
      y2 = (uint16_t)(item->y2 - 2U);
    }
    tft_fill_rgb((uint16_t)(item->x1 + 2U), y1,
                 (uint16_t)(item->x2 - 2U), y2, color);
    tft_button_redraw_slice++;
    return;
  }

  label = tft_get_button_label(tft_button_redraw_index,
                               tft_button_redraw_target);
  if (tft_button_redraw_character < strlen(label)) {
    uint16_t width = (uint16_t)(strlen(label) * 12U);
    uint16_t x = (uint16_t)(item->x1 + (item->x2 - item->x1 - width) / 2U);
    uint16_t y = (uint16_t)(item->y1 + (item->y2 - item->y1 - 16U) / 2U);
    tft_draw_char((uint16_t)(x + 12U * tft_button_redraw_character), y,
                  label[tft_button_redraw_character], 2U,
                  0xFFFFFFU, color);
    tft_button_redraw_character++;
    return;
  }

  if (tft_button_redraw_target) {
    tft_button_state_mask |= (uint8_t)(1U << tft_button_redraw_index);
  } else {
    tft_button_state_mask &= (uint8_t)~(1U << tft_button_redraw_index);
  }
  tft_draw_button_border(tft_button_redraw_index,
                         touch_active_button == (int8_t)tft_button_redraw_index ?
                         0xFFFFFFU : 0x000000U);
  tft_button_redraw_active = 0U;
}

static int8_t tft_find_button(uint16_t x, uint16_t y) {
  for (uint8_t button = 0U;
       button < (uint8_t)(sizeof(tft_buttons) / sizeof(tft_buttons[0]));
       button++) {
    const TftButton *item = &tft_buttons[button];
    if (x >= item->x1 && x < item->x2 && y >= item->y1 && y < item->y2) {
      return (int8_t)button;
    }
  }
  return -1;
}

static void tft_execute_button(uint8_t button) {
  switch (button) {
    case 0U:
      system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE);
      break;
    case 1U:
      touch_action_pending = BOARD_TOUCH_ACTION_SPINDLE_TOGGLE;
      break;
    case 2U:
      if (sys.state & STATE_HOLD) {
        system_set_exec_state_flag(EXEC_CYCLE_START);
      } else if (sys.state & (STATE_CYCLE | STATE_JOG)) {
        system_set_exec_state_flag(EXEC_FEED_HOLD);
      }
      break;
    case 3U:
      touch_action_pending = BOARD_TOUCH_ACTION_RESET_UNLOCK;
      system_set_exec_state_flag(EXEC_RESET);
      break;
    default:
      break;
  }
}

static void tft_touch_task(void) {
  uint32_t now = HAL_GetTick();
  uint16_t x;
  uint16_t y;
  int8_t button;

  if ((uint32_t)(now - touch_poll_tick) < TOUCH_POLL_PERIOD_MS) {
    return;
  }
  touch_poll_tick = now;

  if (!XPT2046_IsTouched()) {
    if (touch_active_button >= 0) {
      tft_draw_button_border((uint8_t)touch_active_button, 0x000000U);
      touch_active_button = -1;
    }
    touch_press_latched = 0U;
    return;
  }

  if (!XPT2046_GetScreenPoint(&x, &y)) {
    return;
  }

  button = tft_find_button(x, y);
  if (!touch_press_latched) {
    touch_press_latched = 1U;
    if (button >= 0) {
      tft_execute_button((uint8_t)button);
    }
  }
  if (button == touch_active_button) {
    return;
  }
  if (touch_active_button >= 0) {
    tft_draw_button_border((uint8_t)touch_active_button, 0x000000U);
  }
  touch_active_button = button;
  if (touch_active_button >= 0) {
    tft_draw_button_border((uint8_t)touch_active_button, 0xFFFFFFU);
  }
}

static void tft_draw_usb_status(void) {
  uint32_t box_color = usb_port_open ? 0xC6D9A3U : 0xA96565U;
  tft_draw_box(288U, 8U, 470U, 47U, box_color);
  tft_draw_centered_text(290U, 10U, 468U, 45U,
                         usb_port_open ? "USB OPEN" : "USB WAIT",
                         2U, 0xE39A00U, box_color);
  tft_usb_state = usb_port_open;
}

static void tft_format_spindle(char text[19], float speed) {
  char value[24];
  uint32_t rpm = speed > 0.0f ? (uint32_t)(speed + 0.5f) : 0U;
  size_t length;
  size_t offset;

  snprintf(value, sizeof(value), "SPINDLE %lu RPM", (unsigned long)rpm);
  memset(text, ' ', 18U);
  text[18] = '\0';
  length = strlen(value);
  if (length > 18U) { length = 18U; }
  offset = (18U - length) / 2U;
  memcpy(&text[offset], value, length);
}

static uint8_t tft_update_spindle_status(void) {
  char text[19];
  const uint32_t panel = 0x3C3C3CU;
  const uint32_t text_color = 0xF7C79EU;

  tft_format_spindle(text, grbl_spindle_speed);
  for (uint8_t character = 0U; character < 18U; character++) {
    if (text[character] != tft_last_spindle_text[character]) {
      tft_draw_char((uint16_t)(249U + 12U * character), 165U,
                    text[character], 2U, text_color, panel);
      tft_last_spindle_text[character] = text[character];
      return 0U;
    }
  }

  if (tft_last_pwm != spindle_pwm_permille) {
    uint16_t old_width = (uint16_t)((219UL * tft_last_pwm) / 1000UL);
    uint16_t new_width = (uint16_t)((219UL * spindle_pwm_permille) / 1000UL);
    if (new_width > old_width) {
      tft_fill_rgb((uint16_t)(248U + old_width), 202U,
                   (uint16_t)(248U + new_width), 210U, 0xFFC000U);
    } else if (new_width < old_width) {
      tft_fill_rgb((uint16_t)(248U + new_width), 202U,
                   (uint16_t)(248U + old_width), 210U, 0x303030U);
    }
  }
  tft_last_pwm = spindle_pwm_permille;
  return 1U;
}

static void tft_format_axis(char *text, size_t size, float value) {
  int32_t scaled;
  uint32_t magnitude;
  char number[16];
  size_t number_length;
  size_t padding;

  if (value > 9999.99f) {
    value = 9999.99f;
  } else if (value < -9999.99f) {
    value = -9999.99f;
  }

  scaled = (int32_t)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
  magnitude = (uint32_t)(scaled < 0 ? -scaled : scaled);
  snprintf(number, sizeof(number), "%c%lu.%02lu",
           scaled < 0 ? '-' : '+',
           (unsigned long)(magnitude / 100U),
           (unsigned long)(magnitude % 100U));

  number_length = strlen(number);
  padding = number_length < 9U ? 9U - number_length : 0U;
  if (size < (1U + padding + number_length)) {
    if (size != 0U) { text[0] = '\0'; }
    return;
  }

  memset(text, ' ', padding);
  memcpy(&text[padding], number, number_length + 1U);
}

static void tft_draw_limit_box(uint8_t index, uint8_t active) {
  static const uint16_t limit_x[4] = {91U, 126U, 161U, 196U};
  static const char limit_name[4][2] = {"X", "Y", "Z", "P"};
  uint32_t color = active ? 0xF20D18U : 0x465158U;

  tft_draw_box(limit_x[index], 160U,
               (uint16_t)(limit_x[index] + 32U), 194U, color);
  tft_draw_centered_text((uint16_t)(limit_x[index] + 2U), 162U,
                         (uint16_t)(limit_x[index] + 30U), 192U,
                         limit_name[index], 2U, 0xFFFFFFU, color);
}

static uint8_t tft_update_input_status(void) {
  uint8_t current = (uint8_t)((grbl_limit_state & 0x07U) |
                              (grbl_probe_state ? 0x08U : 0U));
  for (uint8_t index = 0U; index < 4U; index++) {
    uint8_t mask = (uint8_t)(1U << index);
    if ((current & mask) != (tft_last_input_state & mask)) {
      tft_draw_limit_box(index, (current & mask) != 0U);
      tft_last_input_state ^= mask;
      return 0U;
    }
  }
  return 1U;
}

static uint32_t tft_get_state_color(const char *state) {
  if (strcmp(state, "ALARM") == 0) { return 0xF20D18U; }
  if (strcmp(state, "HOLD") == 0 || strcmp(state, "DOOR") == 0) {
    return 0xFFC000U;
  }
  if (strcmp(state, "RUN") == 0 || strcmp(state, "JOG") == 0) {
    return 0x00A8E8U;
  }
  if (strcmp(state, "IDLE") == 0) { return 0x00B84FU; }
  if (strcmp(state, "HOME") == 0) { return 0x00A896U; }
  return 0x777777U;
}

static uint8_t tft_update_state_status(void) {
  if (tft_state_redraw_phase == 0U) {
    if (strcmp(grbl_state_text, tft_last_state_text) == 0) { return 1U; }
    strncpy(tft_state_redraw_text, grbl_state_text,
            sizeof(tft_state_redraw_text));
    tft_state_redraw_text[sizeof(tft_state_redraw_text) - 1U] = '\0';
    tft_state_redraw_color = tft_get_state_color(tft_state_redraw_text);
    tft_state_redraw_slice = 0U;
    tft_state_redraw_character = 0U;
    tft_state_redraw_phase = 1U;
  }

  if (tft_state_redraw_phase == 1U) {
    uint16_t y1 = (uint16_t)(201U + 4U * tft_state_redraw_slice);
    tft_fill_rgb(57U, y1, 208U, (uint16_t)(y1 + 4U),
                 tft_state_redraw_color);
    tft_state_redraw_slice++;
    if (tft_state_redraw_slice >= 9U) { tft_state_redraw_phase = 2U; }
    return 0U;
  }

  if (tft_state_redraw_phase == 2U) {
    size_t length = strlen(tft_state_redraw_text);
    uint16_t start_x = (uint16_t)(57U + (151U - length * 18U) / 2U);
    if (tft_state_redraw_character < length) {
      tft_draw_char((uint16_t)(start_x + 18U * tft_state_redraw_character),
                    207U, tft_state_redraw_text[tft_state_redraw_character],
                    3U, 0x000000U, tft_state_redraw_color);
      tft_state_redraw_character++;
      return 0U;
    }
    strncpy(tft_last_state_text, tft_state_redraw_text,
            sizeof(tft_last_state_text));
    tft_last_state_text[sizeof(tft_last_state_text) - 1U] = '\0';
    tft_state_redraw_phase = 0U;
  }
  return 1U;
}

static void tft_draw_grbl_status_field(void) {
  static const uint16_t axis_x[4] = {57U, 292U, 57U, 292U};
  static const uint16_t axis_y[4] = {65U, 65U, 116U, 116U};
  const uint32_t panel = 0x3C3C3CU;
  const uint32_t cyan = 0xBDEFFFU;
  char text[20];

  if (tft_grbl_update_field < 4U) {
    uint8_t axis = tft_grbl_update_field;
    if (grbl_machine_position[axis] != tft_last_machine_position[axis]) {
      tft_format_axis(text, sizeof(text), grbl_machine_position[axis]);
      for (uint8_t character = 0U; character < 9U; character++) {
        if (text[character] != tft_last_axis_text[axis][character]) {
          tft_draw_char((uint16_t)(axis_x[axis] + 18U * character),
                        axis_y[axis], text[character], 3U, cyan, panel);
          tft_last_axis_text[axis][character] = text[character];
          return;
        }
      }
      tft_last_machine_position[axis] = grbl_machine_position[axis];
    }
  } else if (tft_grbl_update_field == 4U) {
    if (!tft_update_input_status()) { return; }
  } else if (tft_grbl_update_field == 5U) {
    if (!tft_update_spindle_status()) { return; }
  } else {
    if (!tft_update_state_status()) { return; }
  }

  tft_grbl_update_field++;
  if (tft_grbl_update_field >= 7U) {
    tft_grbl_update_field = 0U;
    grbl_status_pending = 0U;
  }
}

static void tft_draw_dashboard(void) {
  static const uint16_t axis_box_x[4] = {10U, 245U, 10U, 245U};
  static const uint16_t axis_box_y[4] = {54U, 54U, 105U, 105U};
  static const uint32_t axis_color[4] = {
      0x00A8E8U, 0x00B85AU, 0xD00000U, 0x987900U};
  static const char axis_name[4][2] = {"X", "Y", "Z", "A"};
  const uint32_t background = 0x3B3B3BU;
  const uint32_t panel = 0x3C3C3CU;
  char spindle_text[19];

  tft_fill_rgb(0U, 0U, TFT_WIDTH, TFT_HEIGHT, background);
  tft_draw_box(10U, 8U, 288U, 47U, 0x929292U);
  tft_draw_text(17U, 15U, "MACH3 GRBL", 3U, 0xBDEFFFU, 0x929292U);
  tft_draw_usb_status();

  for (uint8_t axis = 0U; axis < 4U; axis++) {
    uint16_t x = axis_box_x[axis];
    uint16_t y = axis_box_y[axis];
    tft_draw_box(x, y, (uint16_t)(x + 225U), (uint16_t)(y + 45U), panel);
    tft_fill_rgb((uint16_t)(x + 2U), (uint16_t)(y + 2U),
                 (uint16_t)(x + 35U), (uint16_t)(y + 43U), axis_color[axis]);
    tft_draw_centered_text((uint16_t)(x + 2U), (uint16_t)(y + 2U),
                           (uint16_t)(x + 35U), (uint16_t)(y + 43U),
                           axis_name[axis], 3U, 0xFFFFFFU, axis_color[axis]);
    tft_draw_text((uint16_t)(x + 47U), (uint16_t)(y + 11U),
                  "    +0.00", 3U, 0xBDEFFFU, panel);
  }

  tft_draw_box(10U, 158U, 235U, 196U, panel);
  tft_draw_text(16U, 167U, "LIMITS", 2U, 0xBDEFFFU, panel);
  for (uint8_t input = 0U; input < 4U; input++) {
    tft_draw_limit_box(input, 0U);
  }
  tft_last_input_state = 0U;

  tft_draw_box(245U, 158U, 470U, 196U, panel);
  tft_format_spindle(spindle_text, 0.0f);
  tft_draw_text(249U, 165U, spindle_text, 2U, 0xF7C79EU, panel);
  memcpy(tft_last_spindle_text, spindle_text, sizeof(tft_last_spindle_text));
  tft_draw_box(245U, 199U, 470U, 213U, 0x303030U);
  tft_last_pwm = 0U;

  tft_draw_box(55U, 199U, 210U, 239U, 0x777777U);
  tft_draw_centered_text(57U, 201U, 208U, 237U,
                         "START", 3U, 0x000000U, 0x777777U);

  for (uint8_t button = 0U;
       button < (uint8_t)(sizeof(tft_buttons) / sizeof(tft_buttons[0]));
       button++) {
    tft_draw_button_face(button, 0U);
  }
  tft_button_state_mask = 0U;
  tft_button_redraw_active = 0U;
}

#if defined(__GNUC__)
#pragma GCC pop_options
#endif

static void update_status_led(void) {
  GPIO_PinState led_state = GPIO_PIN_RESET;

  if (usb_port_open) {
    led_state = GPIO_PIN_SET;
  } else {
    uint32_t phase = HAL_GetTick() % HEARTBEAT_PERIOD_MS;
    if (phase < HEARTBEAT_FIRST_PULSE_END_MS ||
        (phase >= HEARTBEAT_SECOND_PULSE_START_MS &&
         phase < HEARTBEAT_SECOND_PULSE_END_MS)) {
      led_state = GPIO_PIN_SET;
    }
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_state);
}

static void usb_write(const char *buf, uint16_t len) {
  uint32_t start = HAL_GetTick();

  while (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY) {
    if (HAL_GetTick() - start > 50U) {
      break;
    }
  }
}

static void usb_printf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) {
    return;
  }
  if (n >= (int)sizeof(buf)) {
    n = sizeof(buf) - 1;
  }

  usb_write(buf, (uint16_t)n);
}

static const AxisPins *find_axis(char c) {
  c = (char)tolower((unsigned char)c);
  for (uint32_t i = 0; i < sizeof(axes) / sizeof(axes[0]); i++) {
    if (axes[i].axis == c) {
      return &axes[i];
    }
  }

  return NULL;
}

static void print_help(void) {
  usb_printf("\r\nMash3_grbl board test\r\n");
  usb_printf("? | s | en 0/1 | out 1..4 0/1\r\n");
  usb_printf("dir x/y/z/a 0/1 | step x/y/z/a N | spindle 0/1\r\n");
  usb_printf("pwm 0..1000 | idc | outtest 1..4 | usb 0/1 | cycle\r\n\r\n");
}

static void print_status(void) {
  usb_printf("\r\nSTATUS\r\n");
  usb_printf("LED=%u EN=%u PWM=%u/1000 CCR1=%lu USB_PULLUP_PC11=%u\r\n",
             HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET,
             HAL_GPIO_ReadPin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin) == GPIO_PIN_SET,
             spindle_pwm_permille,
             (unsigned long)TIM1->CCR1,
             HAL_GPIO_ReadPin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin) == GPIO_PIN_SET);

  for (uint32_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
    usb_printf("%s=%u ", outputs[i].name,
               HAL_GPIO_ReadPin(outputs[i].port, outputs[i].pin) == GPIO_PIN_SET);
  }
  usb_printf("\r\n");

  for (uint32_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
    usb_printf("%-12s=%u\r\n", inputs[i].name,
               HAL_GPIO_ReadPin(inputs[i].port, inputs[i].pin) == GPIO_PIN_SET);
  }
  usb_printf("\r\n");
}

static void print_idc_status(void) {
  usb_printf("\r\nIDC/LCD connector, top view\r\n");
  usb_printf("1=+5V 2=GND\r\n");
  usb_printf("3 PB8  LCD CLK\r\n");
  usb_printf("4 PB9  LCD DIN\r\n");
  usb_printf("5 PB6  LCD CE\r\n");
  usb_printf("6 PB7  LCD DC\r\n");
  usb_printf("7 PB4  TOUCH DO=%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET);
  usb_printf("8 PB3  TOUCH CS=%u\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET);
  usb_printf("9 PD2  LCD RST\r\n");
  usb_printf("10 PC12 TOUCH IRQ=%u\r\n\r\n", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET);
}

static void set_output(uint32_t out_no, uint32_t value) {
  if (out_no < 1 || out_no > 4) {
    usb_printf("ERR output must be 1..4\r\n");
    return;
  }

  const BoardPin *pin = &outputs[out_no - 1];
  HAL_GPIO_WritePin(pin->port, pin->pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  usb_printf("OK %s=%lu\r\n", pin->name, (unsigned long)(value ? 1 : 0));
}

static void set_dir(char axis, uint32_t value) {
  const AxisPins *a = find_axis(axis);
  if (a == NULL) {
    usb_printf("ERR axis must be x,y,z,a\r\n");
    return;
  }

  HAL_GPIO_WritePin(a->dir_port, a->dir_pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  usb_printf("OK DIR %c=%lu\r\n", a->axis, (unsigned long)(value ? 1 : 0));
}

static void step_axis(char axis, uint32_t count) {
  const AxisPins *a = find_axis(axis);
  if (a == NULL) {
    usb_printf("ERR axis must be x,y,z,a\r\n");
    return;
  }
  if (count > 10000U) {
    count = 10000U;
  }

  for (uint32_t i = 0; i < count; i++) {
    HAL_GPIO_WritePin(a->step_port, a->step_pin, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 600; d++) {
    }
    HAL_GPIO_WritePin(a->step_port, a->step_pin, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 600; d++) {
    }
  }

  usb_printf("OK STEP %c %lu\r\n", a->axis, (unsigned long)count);
}

static void cycle_outputs(void) {
  for (uint32_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
    HAL_GPIO_WritePin(outputs[i].port, outputs[i].pin, GPIO_PIN_SET);
    usb_printf("%s ON\r\n", outputs[i].name);
    HAL_Delay(300);
    HAL_GPIO_WritePin(outputs[i].port, outputs[i].pin, GPIO_PIN_RESET);
    usb_printf("%s OFF\r\n", outputs[i].name);
    HAL_Delay(100);
  }
  usb_printf("OK cycle\r\n");
}

static void test_output(uint32_t out_no) {
  if (out_no < 1 || out_no > 4) {
    usb_printf("ERR output must be 1..4\r\n");
    return;
  }

  const BoardPin *pin = &outputs[out_no - 1];
  usb_printf("Testing %s: GPIO HIGH/LOW 10 times\r\n", pin->name);

  for (uint32_t i = 0; i < 10; i++) {
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_SET);
    usb_printf("%s GPIO=1\r\n", pin->name);
    HAL_Delay(500);
    HAL_GPIO_WritePin(pin->port, pin->pin, GPIO_PIN_RESET);
    usb_printf("%s GPIO=0\r\n", pin->name);
    HAL_Delay(500);
  }

  usb_printf("OK outtest %lu\r\n", (unsigned long)out_no);
}

void BoardTest_SetSpindlePwm(uint16_t permille) {
  if (permille > 1000U) {
    permille = 1000U;
  }

  spindle_pwm_requested_permille = permille;
  if (permille < spindle_pwm_minimum_permille) {
    permille = spindle_pwm_minimum_permille;
  }
  spindle_pwm_permille = permille;
  TIM1->CCR1 = permille;
}

void BoardTest_SetSpindlePwmMinimum(uint16_t permille) {
  if (permille > 1000U) {
    permille = 1000U;
  }
  spindle_pwm_minimum_permille = permille;
  BoardTest_SetSpindlePwm(spindle_pwm_requested_permille);
}

void BoardTest_SetGrblStatus(const float machine_position[4],
                             float spindle_speed, uint8_t limit_state,
                             uint8_t probe_state, const char *state_text) {
  for (uint8_t axis = 0U; axis < 4U; axis++) {
    grbl_machine_position[axis] = machine_position[axis];
  }
  grbl_spindle_speed = spindle_speed;
  grbl_limit_state = limit_state;
  grbl_probe_state = probe_state;
  strncpy(grbl_state_text, state_text, sizeof(grbl_state_text));
  grbl_state_text[sizeof(grbl_state_text) - 1U] = '\0';
  tft_grbl_update_field = 0U;
  grbl_status_pending = 1U;
}

static void parse_line(char *line) {
  char *argv[4] = {0};
  uint32_t argc = 0;

  for (char *p = strtok(line, " \t\r\n"); p != NULL && argc < 4; p = strtok(NULL, " \t\r\n")) {
    argv[argc++] = p;
  }

  if (argc == 0) {
    return;
  }

  for (char *p = argv[0]; *p; p++) {
    *p = (char)tolower((unsigned char)*p);
  }

  if (strcmp(argv[0], "?") == 0 || strcmp(argv[0], "help") == 0) {
    print_help();
  } else if (strcmp(argv[0], "s") == 0 || strcmp(argv[0], "status") == 0) {
    print_status();
  } else if (strcmp(argv[0], "idc") == 0) {
    print_idc_status();
  } else if (strcmp(argv[0], "en") == 0 && argc >= 2) {
    HAL_GPIO_WritePin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin, atoi(argv[1]) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    usb_printf("OK EN=%d\r\n", atoi(argv[1]) ? 1 : 0);
  } else if (strcmp(argv[0], "out") == 0 && argc >= 3) {
    set_output((uint32_t)atoi(argv[1]), (uint32_t)atoi(argv[2]));
  } else if (strcmp(argv[0], "outtest") == 0 && argc >= 2) {
    test_output((uint32_t)atoi(argv[1]));
  } else if (strcmp(argv[0], "dir") == 0 && argc >= 3) {
    set_dir(argv[1][0], (uint32_t)atoi(argv[2]));
  } else if (strcmp(argv[0], "step") == 0 && argc >= 3) {
    step_axis(argv[1][0], (uint32_t)strtoul(argv[2], NULL, 10));
  } else if (strcmp(argv[0], "spindle") == 0 && argc >= 2) {
    BoardTest_SetSpindlePwm(atoi(argv[1]) ? 1000U : 0U);
    usb_printf("OK PWM=%u/1000\r\n", spindle_pwm_permille);
  } else if (strcmp(argv[0], "pwm") == 0 && argc >= 2) {
    BoardTest_SetSpindlePwm((uint16_t)strtoul(argv[1], NULL, 10));
    usb_printf("OK PWM=%u/1000\r\n", spindle_pwm_permille);
  } else if (strcmp(argv[0], "usb") == 0 && argc >= 2) {
    if (atoi(argv[1])) {
      BoardTest_UsbConnect();
      usb_printf("OK USB pullup connected\r\n");
    } else {
      usb_printf("OK USB pullup disconnecting\r\n");
      BoardTest_UsbDisconnect();
    }
  } else if (strcmp(argv[0], "cycle") == 0) {
    cycle_outputs();
  } else {
    usb_printf("ERR unknown command. Type ?\r\n");
  }
}

void BoardTest_Init(void) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUFFER_ENABLE_GPIO_Port, BUFFER_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);
  BoardTest_IdcInit();
  if (TFT_ILI9488_MODE) {
    tft_init();
    tft_draw_dashboard();
    XPT2046_Init();
  } else if (LCD_PIN_DIAGNOSTIC_MODE) {
    lcd_pin_diagnostic_init();
  } else {
    lcd_init();
  }
  BoardTest_PwmInit();
}

void BoardTest_IdcInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LCD_CLK_Pin | LCD_DIN_Pin | LCD_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_CE_Pin | GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LCD_RST_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = LCD_CLK_Pin | LCD_DIN_Pin | LCD_CE_Pin | LCD_DC_Pin |
                        GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LCD_RST_Pin;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void BoardTest_PwmInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();

  GPIO_InitStruct.Pin = SPINDLE_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPINDLE_PWM_GPIO_Port, &GPIO_InitStruct);

  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->SMCR = 0;
  TIM1->DIER = 0;
  TIM1->CCER = 0;

  TIM1->PSC = (SystemCoreClock / 1000000U) - 1U;
  TIM1->ARR = 999U;
  TIM1->CCR1 = spindle_pwm_minimum_permille;

  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
  TIM1->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE;
  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;

  spindle_pwm_requested_permille = 0U;
  spindle_pwm_permille = spindle_pwm_minimum_permille;
}

void BoardTest_UsbPullupInit(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_INACTIVE_STATE);

  GPIO_InitStruct.Pin = USB_PULLUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PULLUP_GPIO_Port, &GPIO_InitStruct);
}

void BoardTest_UsbDisconnect(void) {
  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_INACTIVE_STATE);
}

void BoardTest_UsbConnect(void) {
  HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin, USB_PULLUP_ACTIVE_STATE);
}

void BoardTest_Task(void) {
  update_status_led();

  if (TFT_ILI9488_MODE) {
    tft_touch_task();
    if ((uint32_t)(HAL_GetTick() - tft_button_update_tick) >= 5U) {
      tft_button_update_tick = HAL_GetTick();
      tft_update_button_faces();
    }
    if (tft_usb_state != usb_port_open) {
      tft_draw_usb_status();
    }
    if (grbl_status_pending &&
        (uint32_t)(HAL_GetTick() - tft_grbl_update_tick) >= 5U) {
      tft_grbl_update_tick = HAL_GetTick();
      tft_draw_grbl_status_field();
    }
    if (tft_last_pwm != spindle_pwm_permille) {
      grbl_status_pending = 1U;
    }
  } else if (LCD_PIN_DIAGNOSTIC_MODE) {
    lcd_pin_diagnostic_task();
  } else {
    if (lcd_usb_state != usb_port_open) {
      lcd_show_status();
    }
    lcd_screen_diagnostic_task();
  }

  if (command_ready) {
    char line[sizeof(command_line)];

    __disable_irq();
    strncpy(line, command_line, sizeof(line));
    line[sizeof(line) - 1U] = '\0';
    command_ready = 0;
    __enable_irq();

    parse_line(line);
  }
}

void BoardTest_SetUsbPortOpen(uint8_t is_open) {
  usb_port_open = is_open ? 1U : 0U;
}

uint8_t BoardTest_IsUsbPortOpen(void) {
  return usb_port_open;
}

BoardTouchAction BoardTest_TakeTouchAction(void) {
  BoardTouchAction action = touch_action_pending;
  touch_action_pending = BOARD_TOUCH_ACTION_NONE;
  return action;
}

void BoardTest_OnUsbRx(uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    char c = (char)data[i];

    if (c == '\r' || c == '\n') {
      if (rx_len > 0 && !command_ready) {
        rx_line[rx_len] = '\0';
        strncpy(command_line, rx_line, sizeof(command_line));
        command_line[sizeof(command_line) - 1U] = '\0';
        command_ready = 1;
        rx_len = 0;
      }
    } else if (rx_len < sizeof(rx_line) - 1U) {
      rx_line[rx_len++] = c;
    } else {
      rx_len = 0;
    }
  }
}
