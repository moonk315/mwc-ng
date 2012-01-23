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

 inline void process_idle_state() {
   if (input.stick_state == STICK_STATE_ARM) {
     flight.sys_state = SYS_STATE_ARM_REQ;
     flight.delay_cnt = 10;
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
   } else if (!(input.stick_state & _BV(STICK_STATE_TH_LOW))) {
     flight.sys_state = SYS_STATE_FLIGHT;
     pid.locked = 0;
   }    
 }   

 inline void process_disarm_req_state() {
   if (input.stick_state == STICK_STATE_DISARM) {
     if (!(flight.delay_cnt--)) {
       flight.sys_state = SYS_STATE_IDLE;
       out.motors_armed = 0;
     } 
   } else flight.sys_state = SYS_STATE_ARMED;
 }   
 
 inline void process_flight_state() {
   if ((input.stick_state & _BV(STICK_STATE_TH_LOW))) {
     pid.locked = 1;
     reset_pid_state();
     flight.sys_state = SYS_STATE_ARMED;
   }  
 }   
 
 void process_system_states() {
  switch (flight.sys_state) {
    case SYS_STATE_IDLE: process_idle_state(); break;
    case SYS_STATE_ARM_REQ: process_arm_req_state(); break;  
    case SYS_STATE_ARMED: process_armed_state(); break;  
    case SYS_STATE_DISARM_REQ: process_disarm_req_state(); break;
    case SYS_STATE_FLIGHT: process_flight_state(); break;
  }  
 }  
 
inline void FlightControl_Init() {
}  

inline void FlightControl_loop_5hz() {
  process_system_states();
}  
