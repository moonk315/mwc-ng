/**
 * MultiWii NG 0.1 - 2012
 * Process PIDs. (input) -> (pid)
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

int32_t calc_pid(int16_t pr_err, int16_t sp_err, pid_terms_t *t, pid_rt_t *rt) {
  // P term
  int32_t res = sp_err * t->P / 10;
  // I term
  rt->i_term += sp_err;
  rt->i_term = constrain(rt->i_term, t->windup_min, t->windup_max);
  res += rt->i_term * t->I / 1000;
  // D term
  int16_t tmp = pr_err - rt->last_pr_error;
  rt->last_pr_error = pr_err;
  rt->d_term_fir[rt->d_term_fir_ptr] = tmp;
  if (rt->d_term_fir_ptr++ >= 3) rt->d_term_fir_ptr = 0;
  tmp = 0;
  for (uint8_t i = 0; i < 3; i++) tmp += rt->d_term_fir[i];
  res -= tmp * t->D / 3;
  return res;
};  

int32_t calc_pd(int16_t pr_err, int16_t sp_err, pid_terms_t *t, pid_rt_t *rt) {
  // P term
  int32_t res = sp_err * t->P / 10;
  // D term
  int16_t tmp = pr_err - rt->last_pr_error;
  rt->last_pr_error = pr_err;
  rt->d_term_fir[rt->d_term_fir_ptr] = tmp;
  if (rt->d_term_fir_ptr++ >= 3) rt->d_term_fir_ptr = 0;
  tmp = 0;
  for (uint8_t i = 0; i < 3; i++) tmp += rt->d_term_fir[i];
  res -= tmp * t->D / 3;
  return res;
};  

int32_t calc_pi(int16_t pr_err, int16_t sp_err, pid_terms_t *t, pid_rt_t *rt) {
  // P term
  int32_t res = sp_err * t->P / 10;
  // I term
  rt->i_term += sp_err;
  rt->i_term = constrain(rt->i_term, t->windup_min, t->windup_max);
  res += rt->i_term * t->I / 1000;
  return res;
};

void reset_pid_state() {
  memset(&pid.rt, 0, sizeof(pid.rt));
  memset(&pid.ctrl, 0, sizeof(pid.ctrl));
};

inline void PID_loop_100hz() {
  if (pid.locked) return;
  pid.ctrl.throttle = input.ctrl.throttle;
  pid.ctrl.yaw = input.ctrl.yaw;
  pid.ctrl.roll  = calc_pid(-imu.gyro.eul.roll, (input.ctrl.roll << 3)  - imu.gyro.eul.roll,  &pid.active_profile->roll,  &pid.rt.roll) >> 3; 
  pid.ctrl.pitch = calc_pid(-imu.gyro.eul.roll, (input.ctrl.pitch << 3) - imu.gyro.eul.pitch, &pid.active_profile->pitch, &pid.rt.pitch) >> 3; 
} 

inline void PID_Init() {
  pid.setup.profile[0].roll.P = 40;
  pid.setup.profile[0].roll.I = 30;
  pid.setup.profile[0].roll.windup_min = -16000;
  pid.setup.profile[0].roll.windup_max = +16000;
  pid.setup.profile[0].roll.D = 17;
  pid.setup.profile[0].pitch.P = 40;
  pid.setup.profile[0].pitch.I = 30;
  pid.setup.profile[0].pitch.windup_min = -16000;
  pid.setup.profile[0].pitch.windup_max = +16000;
  pid.setup.profile[0].pitch.D = 17;
  pid.setup.profile[0].yaw.P = 80;
  pid.setup.profile[0].yaw.I = 30;
  pid.active_profile = &pid.setup.profile[0];
}  


