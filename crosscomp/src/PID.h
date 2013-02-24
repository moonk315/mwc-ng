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

int32_t update_pid16(int16_t sp, int16_t pv, pid_terms_t *t, pid_rt_t *rt) {
  int32_t sp32 = (int32_t)sp;
  int32_t sp_err32 = sp32 - pv;
  // I term
  int32_t i_term = rt->i_term;
  i_term += sp_err32;
  if (i_term < -(t->i_windup)) i_term = -(t->i_windup);
  else
  if (i_term > t->i_windup) i_term = t->i_windup;
  int32_t res = (i_term * t->I)  >> 10;
  rt->i_term = i_term;
  // P term
  res += (sp_err32 * t->P) >> 4;
  // D term
  int32_t tmp32 = pv - rt->last_pr_error;
  rt->last_pr_error = pv;
  res -= tmp32 * t->D;
  // Feed forward
  res += (sp32 * t->FF) >> 7;
  return res;
}

void reset_pid_state() {
  memset(&pid.rt, 0, sizeof(pid.rt));
  memset(&pid.ctrl, 0, sizeof(pid.ctrl));
};

inline void PID_loop_inner() {
  if (pid.locked) return;
  //
  pid.ctrl.roll   = update_pid16(pid.rt.outer_pid.roll ,  imu.gyro.eul.roll,  &pid.active_profile->inner.roll,  &pid.rt.inner.roll)  >> 5;
  pid.ctrl.pitch  = update_pid16(pid.rt.outer_pid.pitch,  imu.gyro.eul.pitch, &pid.active_profile->inner.pitch,  &pid.rt.inner.pitch) >> 5;
  pid.ctrl.yaw    = update_pid16(pid.rt.outer_pid.yaw,    imu.gyro.eul.yaw,   &pid.active_profile->inner.yaw,   &pid.rt.inner.yaw)   >> 5;
  pid.ctrl.throttle = update_pid16(pid.rt.outer_pid.throttle,   0,   &pid.active_profile->inner.throttle,   &pid.rt.inner.throttle);

}

inline void PID_loop_outer() {
  if (!pid.locked) {
    pid.rt.outer_pid.roll =  update_pid16((input.ctrl.roll + pid.ictrl_last.roll) << 1,   ahrs.ctrl_ref.roll,  &pid.active_profile->outer.roll,   &pid.rt.outer.roll) ;
    pid.rt.outer_pid.pitch = update_pid16((input.ctrl.pitch + pid.ictrl_last.pitch) << 1, ahrs.ctrl_ref.pitch, &pid.active_profile->outer.pitch,  &pid.rt.outer.pitch);
    pid.rt.outer_pid.yaw   = update_pid16((input.ctrl.yaw + pid.ictrl_last.yaw) << 1,     ahrs.ctrl_ref.yaw,   &pid.active_profile->outer.yaw,    &pid.rt.outer.yaw);
    pid.rt.outer_pid.throttle   = update_pid16(input.ctrl.throttle, 0, &pid.active_profile->outer.throttle,  &pid.rt.outer.throttle);
  }
  pid.ictrl_last = input.ctrl;
}

#define P(x)  ((uint8_t)round(x * 16.0f))
#define I(x)  ((uint8_t)round(x * 1024.0f))
#define D(x)  ((uint8_t) x)
#define FF(x) ((uint8_t) round(x * 128.0f))

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


