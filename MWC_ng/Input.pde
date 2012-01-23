/**
 * MultiWii NG 0.1 - 2012
 * Process Input. (RX) -> (input)
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

#define MINCHECK 1100
#define MAXCHECK 1900


void build_expo_table() {
  for(uint8_t i = 0; i < 7; i++) 
    input.control_expo_lookup[i] = (2500 + input.setup.ctrl_exp * (i*i - 25)) * i * (int32_t)input.setup.ctrl_rate / 1250;
}  

void process_input_control() {
  // Roll and Pitch, range -500..500 
  for(uint8_t i = CTRL_CHANNEL_ROLL; i <= CTRL_CHANNEL_PITCH; i++) {
    uint16_t tmp = abs(int16_t(rx_data.raw[i] - MIDRC));
    uint16_t idx = tmp/100;
    tmp = input.control_expo_lookup[idx] + (tmp - idx*100) * (input.control_expo_lookup[idx+1] - input.control_expo_lookup[idx]) / 100;
    #if (DEADBAND_RP > 0)
      if (tmp < DEADBAND_RP) tmp = 0;
    #endif 
    if (rx_data.raw[i] > MIDRC)
      input.ctrl.raw[i] = tmp;
    else  
      input.ctrl.raw[i] = -tmp;
  }
  // Yaw, range -500..500 
  int16_t tmp = rx_data.yaw - MIDRC;
  #if (DEADBAND_YAW > 0)
    if (abs(tmp) < DEADBAND_YAW) tmp = 0;
  #endif 
  input.ctrl.yaw      = tmp;
  // Throttle, range 0..1000
  tmp = rx_data.throttle;
  if (tmp > RC_MINTHROTTLE) input.ctrl.throttle = tmp - RC_MINTHROTTLE;
  else input.ctrl.throttle = 0;
}  

void process_stick_states() {
  uint8_t tmp = 0;
  if (rx_data.throttle < MINCHECK) tmp |= _BV(STICK_STATE_TH_LOW);
  if (rx_data.throttle > MAXCHECK) tmp |= _BV(STICK_STATE_TH_HIGH);
  if (rx_data.roll < MINCHECK) tmp |= _BV(STICK_STATE_ROLL_LOW);
  if (rx_data.roll > MAXCHECK) tmp |= _BV(STICK_STATE_ROLL_HIGH);
  if (rx_data.pitch < MINCHECK) tmp |= _BV(STICK_STATE_PITCH_LOW);
  if (rx_data.pitch > MAXCHECK) tmp |= _BV(STICK_STATE_PITCH_HIGH);
  if (rx_data.yaw < MINCHECK) tmp |= _BV(STICK_STATE_YAW_LOW);
  if (rx_data.yaw > MAXCHECK) tmp |= _BV(STICK_STATE_YAW_HIGH);
  input.stick_state = tmp; 
}  

inline void Input_Init() {
  input.setup.ctrl_rate = 45;
  input.setup.ctrl_exp = 65;
  build_expo_table();
}  

inline void Input_loop_50hz() {
  process_input_control();
}  

inline void Input_loop_5hz() {
  process_stick_states();
}  

