/*
  USB CDC serial adapter for Grbl on the BSMCE04U-PP STM32F103 board.
  The Grbl protocol remains configured for 115200 baud semantics.
*/

#include "grbl.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE + 1U)
#define USB_TX_CHUNK_SIZE 64U

static uint8_t serial_rx_buffer[RX_RING_BUFFER];
static volatile uint8_t serial_rx_buffer_head;
static volatile uint8_t serial_rx_buffer_tail;
static uint8_t usb_tx_buffer[USB_TX_CHUNK_SIZE];
static uint8_t usb_tx_length;

extern USBD_HandleTypeDef hUsbDeviceFS;

static void serial_process_rx_byte(uint8_t data)
{
  uint8_t next_head;

  switch (data) {
    case CMD_RESET: mc_reset(); return;
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); return;
    case CMD_CYCLE_START: system_set_exec_state_flag(EXEC_CYCLE_START); return;
    case CMD_FEED_HOLD: system_set_exec_state_flag(EXEC_FEED_HOLD); return;
    default: break;
  }

  if (data > 0x7FU) {
    switch (data) {
      case CMD_SAFETY_DOOR: system_set_exec_state_flag(EXEC_SAFETY_DOOR); break;
      case CMD_JOG_CANCEL:
        if (sys.state & STATE_JOG) {
          system_set_exec_state_flag(EXEC_MOTION_CANCEL);
        }
        break;
      case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
      case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
      case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
      case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
      case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
      case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
      case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
      case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
      case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
      case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
      case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
      case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
      case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
      case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
      case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
#ifdef ENABLE_M7
      case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
#endif
      default: break;
    }
    return;
  }

  next_head = (uint8_t)(serial_rx_buffer_head + 1U);
  if (next_head == RX_RING_BUFFER) {
    next_head = 0U;
  }
  if (next_head != serial_rx_buffer_tail) {
    serial_rx_buffer[serial_rx_buffer_head] = data;
    serial_rx_buffer_head = next_head;
  }
}

void serial_init()
{
  serial_rx_buffer_head = 0U;
  serial_rx_buffer_tail = 0U;
  usb_tx_length = 0U;
}

void grbl_serial_flush(void)
{
  USBD_CDC_HandleTypeDef *hcdc;

  if (usb_tx_length == 0U) {
    return;
  }
  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL)) {
    usb_tx_length = 0U;
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  while (hcdc->TxState != 0U) {
    if ((sys_rt_exec_state & EXEC_RESET) ||
        (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)) {
      usb_tx_length = 0U;
      return;
    }
  }

  if (CDC_Transmit_FS(usb_tx_buffer, usb_tx_length) == USBD_OK) {
    while (hcdc->TxState != 0U) {
      if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        usb_tx_length = 0U;
        return;
      }
    }
    usb_tx_length = 0U;
  }
}

void serial_write(uint8_t data)
{
  usb_tx_buffer[usb_tx_length++] = data;
  if (usb_tx_length == USB_TX_CHUNK_SIZE || data == '\n') {
    grbl_serial_flush();
  }
}

uint8_t serial_read()
{
  uint8_t data;
  uint8_t tail = serial_rx_buffer_tail;

  if (serial_rx_buffer_head == tail) {
    grbl_serial_flush();
    return SERIAL_NO_DATA;
  }

  data = serial_rx_buffer[tail++];
  if (tail == RX_RING_BUFFER) {
    tail = 0U;
  }
  serial_rx_buffer_tail = tail;
  return data;
}

void grbl_serial_receive(const uint8_t *data, uint32_t length)
{
  while (length-- != 0U) {
    serial_process_rx_byte(*data++);
  }
}

void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}

uint8_t serial_get_rx_buffer_available()
{
  uint8_t head = serial_rx_buffer_head;
  uint8_t tail = serial_rx_buffer_tail;
  if (head >= tail) {
    return (uint8_t)(RX_BUFFER_SIZE - (head - tail));
  }
  return (uint8_t)(tail - head - 1U);
}

uint8_t serial_get_rx_buffer_count()
{
  uint8_t head = serial_rx_buffer_head;
  uint8_t tail = serial_rx_buffer_tail;
  if (head >= tail) {
    return (uint8_t)(head - tail);
  }
  return (uint8_t)(RX_RING_BUFFER - (tail - head));
}

uint8_t serial_get_tx_buffer_count()
{
  return usb_tx_length;
}
