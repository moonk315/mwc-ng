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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <pt.h>
#include <pt-sem.h>
#include <MWC_MAVLink.h>
#include "MWC_global.h"
#include "config.h"
#include "Core.h"
#include "MWC_Math.h"
#include "MWC_Debug.h"
#include "RX.h"
#include "Sensors.h"
#include "AHRS.h"
#include "Input.h"
#include "PID.h"
#include "Output.h"
#include "FlightControl.h"
#include "Params.h"
#include "MavLink.h"
#include "Storage.h"

void debug_print_system_state() {
  dprintf("\033[1;1H");
  //dprintf("PPM  (th, r, p, y): %8d, %8d, %8d, %8d \n",  get_raw_ppm_data_no_block(RX_CHANNEL_THROTTLE), get_raw_ppm_data_no_block(RX_CHANNEL_ROLL), get_raw_ppm_data_no_block(RX_CHANNEL_PITCH), get_raw_ppm_data_no_block(RX_CHANNEL_YAW));
  //dprintf("RX:  (th, r, p, y, aux1, aux2, aux3, aux4): %8d, %8d, %8d, %8d, %8d, %8d, %8d, %8d  \n",  rx_data.throttle, rx_data.roll, rx_data.pitch, rx_data.yaw, rx_data.aux1, rx_data.aux2, rx_data.aux3, rx_data.aux4);
  //dprintf("Gyro (r, p, y): %8d, %8d, %8d  \n",  imu.gyro.eul.roll, imu.gyro.eul.pitch, imu.gyro.eul.yaw);
  //dprintf("Acc  (X, Y, Z): %8d, %8d, %8d  \n",  imu.acc.fr.x, imu.acc.fr.y, imu.acc.fr.z);
  //dprintf("ACC f(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.acc_grav.x), int16_t(ahrs.acc_grav.y), int16_t(ahrs.acc_grav.z));
  //dprintf("AHRSV(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.est_grav.x), int16_t(ahrs.est_grav.y), int16_t(ahrs.est_grav.z));
  //dprintf("AHRS (r, p, y): %8d, %8d, %8d  \n",  ahrs.eul_ref.roll, ahrs.eul_ref.pitch, ahrs.eul_ref.yaw);
  //dprintf("CPU: %8d%%  \n",  cpu_util_pct * 100 / 255);
  //dprintf("Input: (th, r, p, y, st): %8d, %8d, %8d, %8d  %x \n",  input.ctrl.throttle, input.ctrl.roll, input.ctrl.pitch, input.ctrl.yaw, input.stick_state);
  //dprintf("Fl. Ctrl: (st): %x \n",  flight.sys_state);
  //dprintf("PID:   (th, r, p, y): %8d, %8d, %8d, %8d  \n",  pid.ctrl.throttle, pid.ctrl.roll, pid.ctrl.pitch, pid.ctrl.yaw);
  //dprintf("Out.m[]:(0, 1, 2, 3): %8d, %8d, %8d, %8d  \n",  out.motor[0], out.motor[1], out.motor[2], out.motor[3]);
  dprintf("Mag  (X, Y, Z): %8d, %8d, %8d  \n",  imu.mag.fr.x, imu.mag.fr.y, imu.mag.fr.z);
  dprintf("MAG f(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.mag_mag.x), int16_t(ahrs.mag_mag.y), int16_t(ahrs.mag_mag.z));
  dprintf("AHRSV(x, y, z): %8d, %8d, %8d  \n",  int16_t(ahrs.est_mag.x), int16_t(ahrs.est_mag.y), int16_t(ahrs.est_mag.z));
  dprintf("AHRS (r, p, y): %8d, %8d, %8d  \n",  ahrs.eul_ref.roll, ahrs.eul_ref.pitch, ahrs.eul_ref.yaw);
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
  Params_Init();
  MavLink_Init();
  Storage_Init();
  sei();
  imu_calibrate_gyro();
  imu_calibrate_mag_gain();
}

static struct timer_small timer_inner_ctrl = {0, INNER_CTRL_LOOP_TIME * 2};
static struct timer_small timer_outer_ctrl = {0, OUTER_CTRL_LOOP_TIME * 2};
static struct timer_small timer_acc_ctrl   = {0, ACC_CTRL_LOOP_TIME   * 2};
static struct timer_big   timer_service    = {0, SERVICE_LOOP_TIME    * 2};

static struct pt thread_inner_ctrl_pt;
static struct pt thread_outer_ctrl_pt;
static struct pt thread_acc_ctrl_pt;
static struct pt thread_service_pt;

static struct pt thread_gyro_read_pt;
static struct pt thread_acc_read_pt;
//static struct pt_sem i2c_bus_mutex;

#define IDLE_LOOP_PERIOD 20 // measured
#define IDLE_LOOP_CNT (SERVICE_LOOP_TIME / IDLE_LOOP_PERIOD)
static uint16_t main_loop_cnt;

static PT_THREAD(thread_inner_ctrl(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_inner_ctrl, dt));
  current_time_us += INNER_CTRL_LOOP_TIME;
  current_time_ms += (INNER_CTRL_LOOP_TIME / 1000);
  DebugLEDToggle();
  if (GYRO  != _NONE_) {
    PT_SEM_WAIT(pt, &i2c_bus_mutex);
    PT_SPAWN(pt, &thread_gyro_read_pt, ThreadGyro_GetADC_pt(&thread_gyro_read_pt));
    Gyro_getADC();
    PT_SEM_SIGNAL(pt, &i2c_bus_mutex);
    for (uint8_t i = 0; i < 3; i++)
      imu.gyro.raw[i] = ((imu.gyro_raw.raw[i] >> 1) + (imu.gyro_prev.raw[i] >> 1)) >> 1;
    imu.gyro_prev = imu.gyro_raw;
  }
  PID_loop_inner();
  Output_loop_400hz();
  for (uint8_t i = 0; i < 3; i++)
    imu.gyro_ahrs.raw[i] += (((int32_t)imu.gyro_raw.raw[i] << 8)  - imu.gyro_ahrs.raw[i]) >> 5;
  PT_END(pt);
}

static PT_THREAD(thread_outer_ctrl(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_outer_ctrl, dt));
  //DebugLEDToggle();
  if (GYRO  != _NONE_) {
    AHRS_loop_outer();
  }
  PID_loop_outer();
  PT_END(pt);
}

static PT_THREAD(thread_acc_ctrl(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_acc_ctrl, dt));
  if (ACC  != _NONE_) {
    PT_SEM_WAIT(pt, &i2c_bus_mutex);
    PT_SPAWN(pt, &thread_acc_read_pt, ThreadACC_GetADC_pt(&thread_acc_read_pt));
    ACC_getADC();
    PT_SEM_SIGNAL(pt, &i2c_bus_mutex);
  }
  if (MAG  != _NONE_) {
    PT_SEM_WAIT(pt, &i2c_bus_mutex);
    if (imu.mag_gain_cal == 10)
      Mag_calibrate_gain_start();
    if (imu.mag_gain_cal <= 1) {
      PT_SPAWN(pt, &thread_acc_read_pt, ThreadMag_GetADC_pt(&thread_acc_read_pt));
      Mag_getADC();
      if (imu.mag_gain_cal == 1) {
        Mag_calibrate_gain_end();
        imu.mag_gain_cal = 0;
      }
    } else imu.mag_gain_cal--;
    PT_SEM_SIGNAL(pt, &i2c_bus_mutex);
  }
  if (ACC  != _NONE_) {
    AHRS_loop_acc();
  }
  RX_loop_50hz();
  Input_loop_50hz();
  PT_END(pt);
}

static PT_THREAD(thread_service(struct pt *pt, uint16_t dt)) {
  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, timer_expired(&timer_service, dt));
  Input_loop_5hz();
  FlightControl_loop_5hz();
  // Calc CPU utilization
  uint16_t delta = main_loop_cnt;
  uint8_t idle_pct = ((uint32_t)delta << 8) / IDLE_LOOP_CNT;
  cpu_util_pct = 255 - idle_pct;
  main_loop_cnt = 0;
  // Battery Monitor
  if (IsBatteryVoltageMeasurementFinished()) {
    batt_voltage =  ((int32_t) GetBatteryVoltage() * flight.setup.vbat.voltage_scaler) >> 7;
    StartBatteryVoltageMeasurement();
  }
  debug_print_system_state();
  PT_END(pt);
}

void loop() __attribute__ ((noreturn));
void loop() {
  uint16_t current_tick;
  PT_INIT(&thread_inner_ctrl_pt);
  PT_INIT(&thread_outer_ctrl_pt);
  PT_INIT(&thread_acc_ctrl_pt);
  PT_INIT(&thread_service_pt);
  PT_SEM_INIT(&i2c_bus_mutex, 1);
  for (;;) {
    main_loop_cnt++;
    Board_Idle();
    current_tick = __systick();
    PT_SCHEDULE(thread_inner_ctrl(&thread_inner_ctrl_pt, current_tick));
    PT_SCHEDULE(thread_outer_ctrl(&thread_outer_ctrl_pt, current_tick));
    PT_SCHEDULE(thread_acc_ctrl(&thread_acc_ctrl_pt, current_tick));
    PT_SCHEDULE(thread_service(&thread_service_pt, current_tick));
    MavLink_Loop(current_tick);
  }
}

