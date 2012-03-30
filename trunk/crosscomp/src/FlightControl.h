/**
 * MultiWii NG 0.1 - 2012
 * Flight Control FSM. (input) -> (flight) -> (...)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

inline void blink_led(uint8_t pattern) {
  flight.led_pattern_req = pattern;
}

inline void beep(uint8_t pattern) {
  flight.beep_pattern_req = pattern;
}

 inline void process_idle_state() {
  if (imu.acc_off_cal || imu.gyro_off_cal ) {
    blink_led(LED_PATTERN_CALIBRATION_START);
    flight.sys_state = SYS_STATE_CALIBRATING;
  } else
  if (input.stick_state == STICK_STATE_ARM) {
    flight.sys_state = SYS_STATE_ARM_REQ;
    flight.delay_cnt = 5;
    blink_led(LED_PATTERN_SHORT_BLINK);
    beep(BEEP_PATTERN_SHORT_BLINK);
  } else
  if (input.stick_state == STICK_STATE_ENTER_TRIM) {
    flight.delay_cnt = 5;
    flight.sys_state = SYS_STATE_LEVEL_TRIM;
    beep(BEEP_PATTERN_SHORT_BLINK);
  }
 }

 inline void process_calibrating_state() {
   blink_led(LED_PATTERN_FAST_BLINK);
   if ((!imu.acc_off_cal) && (!imu.gyro_off_cal)) {
     ahrs_reset();
     flight.sys_state = SYS_STATE_IDLE;
     blink_led(LED_PATTERN_CALIBRATION_END);
     beep(BEEP_PATTERN_CALIBRATION_END);
   }
 }

int16_t get_trim_val(int8_t val, int16_t trim) {
  uint8_t changed = 0;
  if (val >  100 / 4)  {trim += 1; changed = 2;}
  if (val >  300 / 4)  {trim += 4; changed = 1;}
  if (val < -100 / 4)  {trim -= 1; changed = 2;}
  if (val < -300 / 4)  {trim -= 4; changed = 1;}
  if (changed) {
    beep(BEEP_PATTERN_SHORT_BLINK);
    flight.delay_cnt = changed;
  }
  return trim;
}

inline void process_level_trim_state() {
  blink_led(LED_PATTERN_SLOW_BLINK);
  if (input.stick_state == STICK_STATE_EXIT_TRIM) {
    flight.sys_state = SYS_STATE_IDLE;
    write_storage();
    beep(BEEP_PATTERN_SHORT_BLINK);
  } else {
    if (!flight.delay_cnt) {
      ahrs.setup.level_trim.roll  = get_trim_val(input.ctrl.roll >> 2, ahrs.setup.level_trim.roll);
      ahrs.setup.level_trim.pitch = get_trim_val(input.ctrl.pitch >> 2, ahrs.setup.level_trim.pitch);
    } else flight.delay_cnt--;
  }
 }

 inline void process_arm_req_state() {
   if (input.stick_state == STICK_STATE_ARM) {
      if (!(flight.delay_cnt--)) {
        flight.sys_state = SYS_STATE_ARMED;
        out.motors_armed = 1;
        pid.locked = 1;
        reset_pid_state();
      }
   } else flight.sys_state = SYS_STATE_IDLE;
 }

 inline void process_armed_state() {
   if (input.stick_state == STICK_STATE_DISARM) {
     flight.sys_state = SYS_STATE_DISARM_REQ;
     flight.delay_cnt = 10;
     beep(BEEP_PATTERN_SHORT_BLINK);
   } else if (!(input.stick_state & _BV(STICK_STATE_TH_LOW))) {
     flight.sys_state = SYS_STATE_FLIGHT;
     pid.locked = 0;
   } else blink_led(LED_PATTERN_ON);
 }

 inline void process_disarm_req_state() {
   if (input.stick_state == STICK_STATE_DISARM) {
     blink_led(LED_PATTERN_ON);
     if (!(flight.delay_cnt--)) {
       flight.sys_state = SYS_STATE_IDLE;
       out.motors_armed = 0;
       blink_led(LED_PATTERN_OFF);
     }
   } else flight.sys_state = SYS_STATE_ARMED;
 }

 inline void process_flight_state() {
   if ((input.stick_state & _BV(STICK_STATE_TH_LOW))) {
     pid.locked = 1;
     reset_pid_state();
     flight.sys_state = SYS_STATE_ARMED;
   } else {
     if (batt_voltage < flight.setup.vbat.voltage_warn2) {
       blink_led(LED_PATTERN_SHORT_BANK);
       beep(BEEP_PATTERN_VBAT_W2);
     } else
     if (batt_voltage < flight.setup.vbat.voltage_warn1) {
       blink_led(LED_PATTERN_SHORT_BANK);
       beep(BEEP_PATTERN_VBAT_W1);
     } else blink_led(LED_PATTERN_ON);
   }
 }

 void process_system_states() {
  switch (flight.sys_state) {
    case SYS_STATE_IDLE: process_idle_state(); break;
    case SYS_STATE_CALIBRATING: process_calibrating_state(); break;
    case SYS_STATE_ARM_REQ: process_arm_req_state(); break;
    case SYS_STATE_ARMED: process_armed_state(); break;
    case SYS_STATE_DISARM_REQ: process_disarm_req_state(); break;
    case SYS_STATE_FLIGHT: process_flight_state(); break;
    case SYS_STATE_LEVEL_TRIM: process_level_trim_state(); break;
  }
 }

void process_led_state() {
  uint8_t led_pattern = flight.led_pattern;
  if (!led_pattern) {
    led_pattern = led_pattern_cfg[flight.led_pattern_req];
    flight.led_pattern_req = 0;
  }
  if ((led_pattern & 0x01))
    StatusLEDOn();
  else
    StatusLEDOff();
  led_pattern = (led_pattern >> 1);
  flight.led_pattern = led_pattern;
}

void process_beep_state() {
  uint8_t beep_pattern = flight.beep_pattern;
  if (!beep_pattern) {
    beep_pattern = beep_pattern_cfg[flight.beep_pattern_req];
    flight.beep_pattern_req = 0;
  }
  if ((beep_pattern & 0x01))
    BeepOn();
  else
    BeepOff();
  beep_pattern = (beep_pattern >> 1);
  flight.beep_pattern = beep_pattern;
}

inline void FlightControl_Init() {
  flight.setup.vbat.voltage_scaler = 127;
  flight.setup.vbat.voltage_warn1  = round((10.5f) * 1000);
  flight.setup.vbat.voltage_warn2  = round((9.9f)  * 1000);
}

inline void FlightControl_loop_5hz() {
  process_system_states();
  process_led_state();
  process_beep_state();
}
