/**
 * MultiWii NG 0.1 - 2012
 * Main sketch file.
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
#include <pt.h>
#include <pt-sem.h>
#include "MWC_global.h"
#include "config.h"
#include "MWC_Debug.h"


void debug_print_system_state() {
  //dprintf("\033[1;1H");
  //dprintf("PPM  (th, r, p, y, aux1): %8d, %8d, %8d, %8d  \n",  get_raw_ppm_data_no_block(RX_CHANNEL_THROTTLE), get_raw_ppm_data_no_block(RX_CHANNEL_ROLL), get_raw_ppm_data_no_block(RX_CHANNEL_PITCH), get_raw_ppm_data_no_block(RX_CHANNEL_YAW), get_raw_ppm_data_no_block(RX_CHANNEL_AUX1));
  //dprintf("RX:  (th, r, p, y, aux1, aux2, aux3, aux4): %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d  \n",  rx_data.throttle, rx_data.roll, rx_data.pitch, rx_data.yaw, rx_data.aux1, rx_data.aux2, rx_data.aux3, rx_data.aux4);
  /*
  dprintf("Gyro (r, p, y): %8d, %8d, %8d  \n",  imu.gyro.eul.roll, imu.gyro.eul.pitch, imu.gyro.eul.yaw);
  dprintf("Acc  (X, Y, Z): %8d, %8d, %8d  \n",  imu.acc.fr.x, imu.acc.fr.y, imu.acc.fr.z);
  dprintf("ACC f(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.acc_grav.x), int16_t(ahrs.acc_grav.y), int16_t(ahrs.acc_grav.z));
  dprintf("AHRSV(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.est_grav.x), int16_t(ahrs.est_grav.y), int16_t(ahrs.est_grav.z));
  dprintf("AHRS (r, p, y): %8d, %8d, %8d  \n",  ahrs.eul_ref.roll, ahrs.eul_ref.pitch, ahrs.eul_ref.yaw);
  dprintf("Input: (th, r, p, y, st): %8d, %8d, %8d, %8d  %x \n",  input.ctrl.throttle, input.ctrl.roll, input.ctrl.pitch, input.ctrl.yaw, input.stick_state);
  dprintf("Fl. Ctrl: (st): %x \n",  flight.sys_state);
  dprintf("PID:   (th, r, p, y): %8d, %8d, %8d, %8d  \n",  pid.ctrl.throttle, pid.ctrl.roll, pid.ctrl.pitch, pid.ctrl.yaw);
  dprintf("Out.m[]:(0, 1, 2, 3): %8d, %8d, %8d, %8d  \n",  out.motor[0], out.motor[1], out.motor[2], out.motor[3]);
  */
}  

void send_message(uint8_t msg) {
  
}  

void setup() { 
  cli();  
  Board_Init();
  Debug_Init();
  RX_Init();
  Sensors_Init();
  AHRS_Init();
  Input_Init();
  PID_Init();
  Output_Init();
  FlightControl_Init();
  sei();
  imu.acc_off_cal = 1024;
  imu.gyro_off_cal = 512;
}

static struct timer_small timer_400hz = {0, 2500*2*2};  
static struct timer_small timer_200hz = {0, 5000*2};  
static struct timer_small timer_50hz  = {0, 20000*2};  
static struct timer_big   timer_5hz   = {0, 200000*2};  

static struct pt thread_400hz_pt;
static struct pt thread_200hz_pt;
static struct pt thread_50hz_pt;
static struct pt thread_5hz_pt;

static struct pt thread_gyro_read_pt;
static struct pt thread_acc_read_pt;
static struct pt_sem i2c_bus_mutex;

static PT_THREAD(thread_400hz(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_400hz, dt));
  StatusLEDToggle();
  PT_SEM_WAIT(pt, &i2c_bus_mutex);
  PT_SPAWN(pt, &thread_gyro_read_pt, ThreadGyro_GetADC_pt(&thread_gyro_read_pt));
  Gyro_getADC();
  PT_SEM_SIGNAL(pt, &i2c_bus_mutex);  
  imu.gyro = imu.gyro_raw;
  AHRS_loop_400hz();
  PID_loop_400hz();
  Output_loop_400hz();
  PT_END(pt);
}  

static PT_THREAD(thread_200hz(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_200hz, dt));
  RX_loop_200hz();
  PT_END(pt);
}  

static PT_THREAD(thread_50hz(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_50hz, dt));
  PT_SEM_WAIT(pt, &i2c_bus_mutex);
  PT_SPAWN(pt, &thread_acc_read_pt, ThreadACC_GetADC_pt(&thread_acc_read_pt));
  ACC_getADC();
  PT_SEM_SIGNAL(pt, &i2c_bus_mutex);  
  AHRS_loop_50hz();
  RX_loop_50hz();
  Input_loop_50hz();
  PT_END(pt);
}  

static PT_THREAD(thread_5hz(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_5hz, dt));
  Input_loop_5hz();
  FlightControl_loop_5hz();
  debug_print_system_state();
  PT_END(pt);
}  

void loop() __attribute__ ((noreturn));
void loop() {
  PT_INIT(&thread_400hz_pt);
  PT_INIT(&thread_200hz_pt);
  PT_INIT(&thread_50hz_pt);
  PT_INIT(&thread_5hz_pt);
  for (;;) {
    Board_Idle();
    currentTime = __systick();
    PT_SCHEDULE(thread_400hz(&thread_400hz_pt, currentTime));
    //PT_SCHEDULE(thread_200hz(&thread_200hz_pt, currentTime));
    PT_SCHEDULE(thread_50hz(&thread_50hz_pt, currentTime));
    PT_SCHEDULE(thread_5hz(&thread_5hz_pt, currentTime)); 
  }
}

