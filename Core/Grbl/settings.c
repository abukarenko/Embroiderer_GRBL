/*
  settings.c - eeprom configuration handling
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

settings_t settings;

static void write_global_settings(void);

typedef struct {
  float steps_per_mm[3];
  float max_rate[3];
  float acceleration[3];
  float max_travel[3];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
} settings_v10_t;

typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
} settings_v11_t;

typedef struct {
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time;
  uint8_t status_report_mask;
  float junction_deviation;
  float arc_tolerance;
  float rpm_max;
  float rpm_min;
  uint8_t flags;
  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
  uint8_t probe_resume_enable;
} settings_v12_t;

_Static_assert(sizeof(settings_v10_t) == 92U,
               "Unexpected legacy settings layout");
_Static_assert(sizeof(settings_v11_t) == 108U,
               "Unexpected v11 settings layout");
_Static_assert(sizeof(settings_v12_t) == 112U,
               "Unexpected v12 settings layout");

static uint8_t settings_migrate_v12(void)
{
  settings_v12_t old;

  if (!memcpy_from_eeprom_with_checksum((char *)&old, EEPROM_ADDR_GLOBAL,
                                        sizeof(old))) {
    return false;
  }

  memset(&settings, 0, sizeof(settings));
  memcpy(&settings, &old, sizeof(old));
  settings.spindle_pwm_min_percent = DEFAULT_SPINDLE_PWM_MIN_PERCENT;
  write_global_settings();
  return true;
}

static uint8_t settings_migrate_v11(void)
{
  settings_v11_t old;

  if (!memcpy_from_eeprom_with_checksum((char *)&old, EEPROM_ADDR_GLOBAL,
                                        sizeof(old))) {
    return false;
  }

  memset(&settings, 0, sizeof(settings));
  memcpy(&settings, &old, sizeof(old));
  settings.probe_resume_enable = DEFAULT_PROBE_RESUME_ENABLE;
  write_global_settings();
  return true;
}

static uint8_t settings_migrate_v10(void)
{
  settings_v10_t old;
  float old_coord[SETTING_INDEX_NCOORD + 1U][3];
  uint8_t coord_valid[SETTING_INDEX_NCOORD + 1U];
  uint8_t idx;

  if (!memcpy_from_eeprom_with_checksum((char *)&old, EEPROM_ADDR_GLOBAL,
                                        sizeof(old))) {
    return false;
  }

  for (idx = 0U; idx <= SETTING_INDEX_NCOORD; idx++) {
    uint32_t old_addr = EEPROM_ADDR_PARAMETERS + idx * (sizeof(float) * 3U + 1U);
    coord_valid[idx] = memcpy_from_eeprom_with_checksum(
        (char *)old_coord[idx], old_addr, sizeof(old_coord[idx]));
  }

  memset(&settings, 0, sizeof(settings));
  for (idx = 0U; idx < 3U; idx++) {
    settings.steps_per_mm[idx] = old.steps_per_mm[idx];
    settings.max_rate[idx] = old.max_rate[idx];
    settings.acceleration[idx] = old.acceleration[idx];
    settings.max_travel[idx] = old.max_travel[idx];
  }
  settings.steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_DEGREE;
  settings.max_rate[A_AXIS] = DEFAULT_A_MAX_RATE;
  settings.acceleration[A_AXIS] = DEFAULT_A_ACCELERATION;
  settings.max_travel[A_AXIS] = -DEFAULT_A_MAX_TRAVEL;

  settings.pulse_microseconds = old.pulse_microseconds;
  settings.step_invert_mask = old.step_invert_mask |
      (DEFAULT_STEPPING_INVERT_MASK & bit(A_AXIS));
  settings.dir_invert_mask = old.dir_invert_mask |
      (DEFAULT_DIRECTION_INVERT_MASK & bit(A_AXIS));
  settings.stepper_idle_lock_time = old.stepper_idle_lock_time;
  settings.status_report_mask = old.status_report_mask;
  settings.junction_deviation = old.junction_deviation;
  settings.arc_tolerance = old.arc_tolerance;
  settings.rpm_max = old.rpm_max;
  settings.rpm_min = old.rpm_min;
  settings.flags = old.flags;
  settings.homing_dir_mask = old.homing_dir_mask & LIMIT_MASK;
  settings.homing_feed_rate = old.homing_feed_rate;
  settings.homing_seek_rate = old.homing_seek_rate;
  settings.homing_debounce_delay = old.homing_debounce_delay;
  settings.homing_pulloff = old.homing_pulloff;
  write_global_settings();

  for (idx = 0U; idx <= SETTING_INDEX_NCOORD; idx++) {
    float coord[N_AXIS] = {0.0f};
    if (coord_valid[idx]) {
      coord[X_AXIS] = old_coord[idx][X_AXIS];
      coord[Y_AXIS] = old_coord[idx][Y_AXIS];
      coord[Z_AXIS] = old_coord[idx][Z_AXIS];
    }
    settings_write_coord_data(idx, coord);
  }
  return true;
}


// Method to store startup lines into EEPROM
void settings_store_startup_line(uint8_t n, char *line)
{
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing. 
  #endif
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  memcpy_to_eeprom_with_checksum(addr,(char*)line, LINE_BUFFER_SIZE);
}


// Method to store build info into EEPROM
// NOTE: This function can only be called in IDLE state.
void settings_store_build_info(char *line)
{
  // Build info can only be stored when state is IDLE.
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO,(char*)line, LINE_BUFFER_SIZE);
}


// Method to store coord data parameters into EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize();
  #endif
  uint32_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  memcpy_to_eeprom_with_checksum(addr,(char*)coord_data, sizeof(float)*N_AXIS);
}


// Method to store Grbl global settings struct and version number into EEPROM
// NOTE: This function can only be called in IDLE state.
static void write_global_settings(void)
{
  eeprom_put_char(0, SETTINGS_VERSION);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (char*)&settings, sizeof(settings_t));
}


// Method to restore EEPROM-saved Grbl global settings back to defaults.
void settings_restore(uint8_t restore_flag) {
  if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {
    settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
    settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
    settings.step_invert_mask = DEFAULT_STEPPING_INVERT_MASK;
    settings.dir_invert_mask = DEFAULT_DIRECTION_INVERT_MASK;
    settings.status_report_mask = DEFAULT_STATUS_REPORT_MASK;
    settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
    settings.arc_tolerance = DEFAULT_ARC_TOLERANCE;
    settings.probe_resume_enable = DEFAULT_PROBE_RESUME_ENABLE;
    settings.spindle_pwm_min_percent = DEFAULT_SPINDLE_PWM_MIN_PERCENT;

    settings.rpm_max = DEFAULT_SPINDLE_RPM_MAX;
    settings.rpm_min = DEFAULT_SPINDLE_RPM_MIN;

    settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
    settings.homing_feed_rate = DEFAULT_HOMING_FEED_RATE;
    settings.homing_seek_rate = DEFAULT_HOMING_SEEK_RATE;
    settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
    settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;

    settings.flags = 0;
    if (DEFAULT_REPORT_INCHES) { settings.flags |= BITFLAG_REPORT_INCHES; }
    if (DEFAULT_LASER_MODE) { settings.flags |= BITFLAG_LASER_MODE; }
    if (DEFAULT_INVERT_ST_ENABLE) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
    if (DEFAULT_HARD_LIMIT_ENABLE) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
    if (DEFAULT_HOMING_ENABLE) { settings.flags |= BITFLAG_HOMING_ENABLE; }
    if (DEFAULT_SOFT_LIMIT_ENABLE) { settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE; }
    if (DEFAULT_INVERT_LIMIT_PINS) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
    if (DEFAULT_INVERT_PROBE_PIN) { settings.flags |= BITFLAG_INVERT_PROBE_PIN; }

    settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
    settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
    settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
    settings.steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_DEGREE;
    settings.max_rate[X_AXIS] = DEFAULT_X_MAX_RATE;
    settings.max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE;
    settings.max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE;
    settings.max_rate[A_AXIS] = DEFAULT_A_MAX_RATE;
    settings.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
    settings.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
    settings.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
    settings.acceleration[A_AXIS] = DEFAULT_A_ACCELERATION;
    settings.max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL);
    settings.max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL);
    settings.max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL);
    settings.max_travel[A_AXIS] = (-DEFAULT_A_MAX_TRAVEL);

    write_global_settings();
  }

  if (restore_flag & SETTINGS_RESTORE_PARAMETERS) {
    uint8_t idx;
    float coord_data[N_AXIS];
    memset(&coord_data, 0, sizeof(coord_data));
    for (idx=0; idx <= SETTING_INDEX_NCOORD; idx++) { settings_write_coord_data(idx, coord_data); }
  }

  if (restore_flag & SETTINGS_RESTORE_STARTUP_LINES) {
    #if N_STARTUP_LINE > 0
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK, 0);
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+1, 0); // Checksum
    #endif
    #if N_STARTUP_LINE > 1
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+(LINE_BUFFER_SIZE+1), 0);
      eeprom_put_char(EEPROM_ADDR_STARTUP_BLOCK+(LINE_BUFFER_SIZE+2), 0); // Checksum
    #endif
  }

  if (restore_flag & SETTINGS_RESTORE_BUILD_INFO) {
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO , 0);
    eeprom_put_char(EEPROM_ADDR_BUILD_INFO+1 , 0); // Checksum
  }
}


// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  if (!(memcpy_from_eeprom_with_checksum((char*)line, addr, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    line[0] = 0; // Empty line
    settings_store_startup_line(n, line);
    return(false);
  }
  return(true);
}


// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_build_info(char *line)
{
  if (!(memcpy_from_eeprom_with_checksum((char*)line, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    line[0] = 0; // Empty line
    settings_store_build_info(line);
    return(false);
  }
  return(true);
}


// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  uint32_t addr = coord_select*(sizeof(float)*N_AXIS+1) + EEPROM_ADDR_PARAMETERS;
  if (!(memcpy_from_eeprom_with_checksum((char*)coord_data, addr, sizeof(float)*N_AXIS))) {
    // Reset with default zero vector
		memset(coord_data, 0, sizeof(float)*N_AXIS);
		settings_write_coord_data(coord_select,coord_data);
    return(false);
  }
  return(true);
}


// Reads Grbl global settings struct from EEPROM.
uint8_t read_global_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  if (version == SETTINGS_VERSION) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t)))) {
      return(false);
    }
  } else {
    return(false);
  }
  return(true);
}


// A helper method to set settings from command line
uint8_t settings_store_global_setting(uint8_t parameter, float value) {
  if (value < 0.0f) { return(STATUS_NEGATIVE_VALUE); }
  if (parameter >= AXIS_SETTINGS_START_VAL) {
    // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
    // NOTE: Ensure the setting index corresponds to the report.c settings printout.
    parameter -= AXIS_SETTINGS_START_VAL;
    uint8_t set_idx = 0;
    while (set_idx < AXIS_N_SETTINGS) {
      if (parameter < N_AXIS) {
        // Valid axis setting found.
        switch (set_idx) {
          case 0:
            #ifdef MAX_STEP_RATE_HZ
            if (value*settings.max_rate[parameter] >(MAX_STEP_RATE_HZ*60.0f)) { return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.steps_per_mm[parameter] = value;
            break;
          case 1:
            #ifdef MAX_STEP_RATE_HZ
            if (value*settings.steps_per_mm[parameter] > (MAX_STEP_RATE_HZ*60.0f)) { return(STATUS_MAX_STEP_RATE_EXCEEDED); }
            #endif
            settings.max_rate[parameter] = value;
            break;
          case 2: settings.acceleration[parameter] = value*60*60; break; // Convert to mm/min^2 for grbl internal use.
          case 3: settings.max_travel[parameter] = -value; break;  // Store as negative for grbl internal use.
        }
        break; // Exit while-loop after setting has been configured and proceed to the EEPROM write call.
      } else {
        set_idx++;
        // If axis index greater than N_AXIS or setting index greater than number of axis settings, error out.
        if ((parameter < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS)) { return(STATUS_INVALID_STATEMENT); }
        parameter -= AXIS_SETTINGS_INCREMENT;
      }
    }
  } else {
    // Store non-axis Grbl settings
    uint8_t int_value = truncf(value);
    switch(parameter) {
      case 0:
        if (int_value < 3) { return(STATUS_SETTING_STEP_PULSE_MIN); }
        settings.pulse_microseconds = int_value; break;
      case 1: settings.stepper_idle_lock_time = int_value; break;
      case 2:
        settings.step_invert_mask = int_value;
        st_generate_step_dir_invert_masks(); // Regenerate step and direction port invert masks.
        break;
      case 3:
        settings.dir_invert_mask = int_value;
        st_generate_step_dir_invert_masks(); // Regenerate step and direction port invert masks.
        break;
      case 4: // Reset to ensure change. Immediate re-init may cause problems.
        if (int_value) { settings.flags |= BITFLAG_INVERT_ST_ENABLE; }
        else { settings.flags &= ~BITFLAG_INVERT_ST_ENABLE; }
        break;
      case 5: // Reset to ensure change. Immediate re-init may cause problems.
        if (int_value) { settings.flags |= BITFLAG_INVERT_LIMIT_PINS; }
        else { settings.flags &= ~BITFLAG_INVERT_LIMIT_PINS; }
        break;
      case 6: // Reset to ensure change. Immediate re-init may cause problems.
        if (int_value) { settings.flags |= BITFLAG_INVERT_PROBE_PIN; }
        else { settings.flags &= ~BITFLAG_INVERT_PROBE_PIN; }
        probe_configure_invert_mask(false);
        break;
      case 7:
        if (value != 0.0f) { return(STATUS_INVALID_STATEMENT); }
        settings.probe_resume_enable = int_value;
        break;
      case 8:
        if (value != 50.0f) { return(STATUS_INVALID_STATEMENT); }
        settings.spindle_pwm_min_percent = int_value;
#ifdef GRBL_STM32_HAL
        grbl_platform_spindle_set_minimum_percent(int_value);
#endif
        break;
      case 10: settings.status_report_mask = int_value; break;
      case 11: settings.junction_deviation = value; break;
      case 12: settings.arc_tolerance = value; break;
      case 13:
        if (int_value) { settings.flags |= BITFLAG_REPORT_INCHES; }
        else { settings.flags &= ~BITFLAG_REPORT_INCHES; }
        system_flag_wco_change(); // Make sure WCO is immediately updated.
        break;
      case 20:
        if (int_value) {
          if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE)) { return(STATUS_SOFT_LIMIT_ERROR); }
          settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
        } else { settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; }
        break;
      case 21:
        if (int_value) { settings.flags |= BITFLAG_HARD_LIMIT_ENABLE; }
        else { settings.flags &= ~BITFLAG_HARD_LIMIT_ENABLE; }
        limits_init(); // Re-init to immediately change. NOTE: Nice to have but could be problematic later.
        break;
      case 22:
        if (int_value) { settings.flags |= BITFLAG_HOMING_ENABLE; }
        else {
          settings.flags &= ~BITFLAG_HOMING_ENABLE;
          settings.flags &= ~BITFLAG_SOFT_LIMIT_ENABLE; // Force disable soft-limits.
        }
        break;
      case 23: settings.homing_dir_mask = int_value; break;
      case 24: settings.homing_feed_rate = value; break;
      case 25: settings.homing_seek_rate = value; break;
      case 26: settings.homing_debounce_delay = int_value; break;
      case 27: settings.homing_pulloff = value; break;
      case 30: settings.rpm_max = value; spindle_init(); break; // Re-initialize spindle rpm calibration
      case 31: settings.rpm_min = value; spindle_init(); break; // Re-initialize spindle rpm calibration
      case 32:
        if (value != 0.0f) { return(STATUS_INVALID_STATEMENT); }
        #ifdef VARIABLE_SPINDLE
          if (int_value) { settings.flags |= BITFLAG_LASER_MODE; }
          else { settings.flags &= ~BITFLAG_LASER_MODE; }
        #else
				return(STATUS_SETTING_DISABLED_LASER);
        #endif
        break;
      default:
        return(STATUS_INVALID_STATEMENT);
    }
  }
  write_global_settings();
  return(STATUS_OK);
}


// Initialize the config subsystem
void settings_init() {
  if ((eeprom_get_char(0) == SETTINGS_VERSION_LEGACY_PROBE_RESUME) &&
      settings_migrate_v12()) {
    return;
  }
  if ((eeprom_get_char(0) == SETTINGS_VERSION_LEGACY_4_AXIS) &&
      settings_migrate_v11()) {
    return;
  }
  if ((eeprom_get_char(0) == SETTINGS_VERSION_LEGACY_3_AXIS) &&
      settings_migrate_v10()) {
    return;
  }
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_restore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data.
    report_grbl_settings();
  }
}

