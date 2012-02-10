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

int32_t calc_pid(int16_t pr_err, int32_t sp_err, pid_terms_t *t, pid_rt_t *rt) {
  // P term
  int32_t res = sp_err * t->P / 10;
  // I term
  rt->i_term += sp_err;
  if (rt->i_term < -t->i_windup) rt->i_term = -t->i_windup;
  else
  if (rt->i_term > t->i_windup) rt->i_term = t->i_windup;
  res += rt->i_term * t->I  / 1000;
  // D term
  int16_t tmp = rt->last_pr_error - pr_err;
  rt->last_pr_error = pr_err;
  res -= tmp * t->D;
  return res;
};  


int32_t update_pid16(int16_t sp, int16_t pv, pid_terms_t *t, pid_rt_t *rt) {
  int32_t sp_err = sp - pv;
  // P term
  int32_t res = (sp_err * t->P) >> 4;
  // I term
  rt->i_term += sp_err;
  if (rt->i_term < -t->i_windup) rt->i_term = -t->i_windup;
  else
  if (rt->i_term > t->i_windup) rt->i_term = t->i_windup;
  res += (rt->i_term * t->I)  >> 10;
  // D term
  int16_t tmp = pv - rt->last_pr_error;
  rt->last_pr_error = pv;
  res -= tmp * t->D;
  // Feed forward
  res += ((int32_t)sp * t->FF) >> 7;
  return res;
}  


void reset_pid_state() {
  memset(&pid.rt, 0, sizeof(pid.rt));
  memset(&pid.ctrl, 0, sizeof(pid.ctrl));
};

inline void PID_loop_inner() {
  if (pid.locked) return;
  //pid.ctrl.throttle = input.ctrl.throttle;
  //pid.ctrl.yaw = input.ctrl.yaw;
  //pid.ctrl.roll  =(input.ctrl.roll >> 0)  + calc_pid(-imu.gyro.eul.roll,   (input.ctrl.roll << 2)  - imu.gyro.eul.roll - ahrs.eul_ref.roll,  &pid.active_profile->roll,  &pid.rt.roll) >> 5; 
  //pid.ctrl.pitch =(input.ctrl.pitch >> 0) + calc_pid(-imu.gyro.eul.pitch, (input.ctrl.pitch << 2) - imu.gyro.eul.pitch - ahrs.eul_ref.pitch,  &pid.active_profile->pitch, &pid.rt.pitch) >> 5; 
  //pid.ctrl.yaw   =(input.ctrl.yaw >> 0)   + calc_pid(-imu.gyro.eul.yaw,     (input.ctrl.yaw << 2)   - imu.gyro.eul.yaw,   &pid.active_profile->yaw,   &pid.rt.yaw) >> 5; 
//  pid.ctrl.roll   = update_pid16(input.ctrl.roll << 2,  imu.gyro.eul.roll,  &pid.active_profile->inner.roll,  &pid.rt.inner.roll)  >> 5;
//  pid.ctrl.pitch  = update_pid16(input.ctrl.pitch << 2, imu.gyro.eul.pitch, &pid.active_profile->inner.roll,  &pid.rt.inner.pitch) >> 5;
//  pid.ctrl.yaw    = update_pid16(input.ctrl.yaw << 2,   imu.gyro.eul.yaw,   &pid.active_profile->inner.yaw,   &pid.rt.inner.yaw)   >> 5;
  //
  pid.ctrl.roll   = update_pid16(pid.rt.outer_pid.roll << 2,  imu.gyro.eul.roll,  &pid.active_profile->inner.roll,  &pid.rt.inner.roll)  >> 5;
  pid.ctrl.pitch  = update_pid16(pid.rt.outer_pid.pitch << 2, imu.gyro.eul.pitch, &pid.active_profile->inner.roll,  &pid.rt.inner.pitch) >> 5;
  pid.ctrl.yaw    = update_pid16(pid.rt.outer_pid.yaw << 2,   imu.gyro.eul.yaw,   &pid.active_profile->inner.yaw,   &pid.rt.inner.yaw)   >> 5;  
  pid.ctrl.throttle = update_pid16(pid.rt.outer_pid.throttle,   0,   &pid.active_profile->inner.throttle,   &pid.rt.inner.throttle);

} 

inline void PID_loop_outer() {
  if (pid.locked) return;
  pid.rt.outer_pid.roll = update_pid16(input.ctrl.roll,   ahrs.eul_ref.roll,  &pid.active_profile->outer.roll,    &pid.rt.outer.roll) ;
  pid.rt.outer_pid.pitch = update_pid16(input.ctrl.pitch, ahrs.eul_ref.pitch, &pid.active_profile->outer.pitch,  &pid.rt.outer.pitch);
  pid.rt.outer_pid.yaw   = update_pid16(input.ctrl.yaw,   ahrs.eul_ref.yaw,   &pid.active_profile->outer.yaw,  &pid.rt.outer.yaw);
  pid.rt.outer_pid.throttle   = update_pid16(input.ctrl.throttle, 0, &pid.active_profile->outer.throttle,  &pid.rt.outer.throttle);
}  

#define P(x)  ((uint8_t) x * 16)
#define I(x)  ((uint8_t) x * 1024)
#define D(x)  ((uint8_t) x) 
#define FF(x) ((uint8_t) x * 128)

inline void PID_Init() {
  pid.setup.profile[0].inner.roll.P = P(12.0);
  pid.setup.profile[0].inner.roll.I = I(0.080);
  pid.setup.profile[0].inner.roll.i_windup = 64000;// -16000*2;
  pid.setup.profile[0].inner.roll.D = D(17);
  pid.setup.profile[0].inner.roll.FF = FF(1.0);
  pid.setup.profile[0].outer.roll.FF = FF(1.0);
  pid.setup.profile[0].inner.pitch.P = P(12.0);
  pid.setup.profile[0].inner.pitch.I = I(0.080);
  pid.setup.profile[0].inner.pitch.i_windup = 64000;//-16000*2;
  pid.setup.profile[0].inner.pitch.D = D(17);
  pid.setup.profile[0].inner.pitch.FF = FF(1.0);
  pid.setup.profile[0].outer.pitch.FF = FF(1.0);
  pid.setup.profile[0].inner.yaw.P = P(8.0);
  pid.setup.profile[0].inner.yaw.I = I(0.080);
  pid.setup.profile[0].inner.yaw.i_windup = 64000;//-16000*2;
  pid.setup.profile[0].inner.yaw.FF = FF(1.0);
  pid.setup.profile[0].outer.yaw.FF = FF(1.0);
  // throttle passthrough
  pid.setup.profile[0].inner.throttle.FF = FF(1.0);
  pid.setup.profile[0].outer.throttle.FF = FF(1.0);
  pid.active_profile = &pid.setup.profile[0];
}  


