/**
 * MultiWii NG 0.1 - 2012
 * AHRS. (imu) -> (ahrs)
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

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 13

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 8

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 220.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 400.0f

#define AHRS_FAST_SPIN 50

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define INV_LPF_CONST         (1.0f/ACC_LPF_FACTOR)

#define ssin(val) (val)
#define scos(val) (1.0f)

#define GYRO_SCALE ((PI)/(Gyro_getLSB() * 180.0f * 1000000.0f) * OUTER_CTRL_LOOP_TIME / 256.0 * 1.04)

static uint8_t ahrs_fast_spin() {
  if (abs(imu.gyro.eul.roll)  >  (AHRS_FAST_SPIN * Gyro_getLSB() / 2)) return 1;
  if (abs(imu.gyro.eul.pitch) >  (AHRS_FAST_SPIN * Gyro_getLSB() / 2)) return 1;
  if (abs(imu.gyro.eul.yaw)   >  (AHRS_FAST_SPIN * Gyro_getLSB() / 2)) return 1;
  return 0;
}

static void ahrs_rotate_vectors(){
  float deltaGyroAngle, tmp;
  #define eg  ahrs.est_grav
  #define em  ahrs.est_mag
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  = -(imu.gyro_ahrs.eul.yaw - ahrs.acc_err.z) * GYRO_SCALE;
  tmp  =  eg.x;
  eg.x =  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * eg.y;
  eg.y =  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * eg.y;
  if (MAG != _NONE_) {
    tmp = em.x;
    em.x =  scos(deltaGyroAngle) * tmp - ssin(deltaGyroAngle) * em.y;
    em.y =  ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * em.y;
  }
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  = (imu.gyro_ahrs.eul.pitch - ahrs.acc_err.y) * GYRO_SCALE;
  tmp = eg.x;
  eg.x =  ssin(deltaGyroAngle) * eg.z + scos(deltaGyroAngle) * tmp;
  eg.z =  scos(deltaGyroAngle) * eg.z - ssin(deltaGyroAngle) * tmp;
  if (MAG != _NONE_) {
    tmp = em.x;
    em.x =  ssin(deltaGyroAngle) * em.z + scos(deltaGyroAngle) * tmp;
    em.z =  scos(deltaGyroAngle) * em.z - ssin(deltaGyroAngle) * tmp;
  }
  // Rotate Estimated vector(s), ROLL
  deltaGyroAngle  = (imu.gyro_ahrs.eul.roll - ahrs.acc_err.x) * GYRO_SCALE;
  tmp = eg.y;
  eg.y =  scos(deltaGyroAngle) * tmp + ssin(deltaGyroAngle) * eg.z;
  eg.z = -ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * eg.z;
  if (MAG != _NONE_) {
    tmp = em.y;
    em.y =  scos(deltaGyroAngle) * tmp + ssin(deltaGyroAngle) * em.z;
    em.z = -ssin(deltaGyroAngle) * tmp + scos(deltaGyroAngle) * em.z;
  }
}

static void ahrs_calc_ctrl_ref(){
  // Calculate level reference and apply trims
  ahrs.ctrl_ref.roll  = (int16_t)(eg.y * 2828.43f) - ahrs.setup.level_trim.roll;
  ahrs.ctrl_ref.pitch = (int16_t)(eg.x * 2828.43f) - ahrs.setup.level_trim.pitch;
}

static void ahrs_calc_euler(){
  float _sin,_cos, Mpz, Mfy, Mfx, Gpz, invmag;
  Gpz =  eg.z;
  ahrs.eul_ref.roll = _atan2(eg.y, Gpz);
  // To earth frame (roll)
  invmag = InvSqrt(Gpz * Gpz + ahrs.est_grav.y * ahrs.est_grav.y);
  _sin = ahrs.est_grav.y * invmag;
  _cos = Gpz * invmag;
  Gpz = ahrs.est_grav.y * _sin + Gpz * _cos;
  ahrs.eul_ref.pitch = _atan2(ahrs.est_grav.x, Gpz);
  if (MAG != _NONE_) {
    // To earth frame (roll)
    Mpz =  ahrs.est_mag.z;
    Mfy = ahrs.est_mag.y * _cos - Mpz * _sin;
    Mpz = ahrs.est_mag.y * _sin + Mpz * _cos;
    // To earth frame (pitch)
    invmag = InvSqrt(Gpz * Gpz + ahrs.est_grav.x * ahrs.est_grav.x);
    _sin = -ahrs.est_grav.x * invmag;
    _cos = Gpz * invmag;
    //if (_cos < 0.0) _cos = -_cos;
    Mfx =  ahrs.est_mag.x * _cos + Mpz * _sin;
    ahrs.eul_ref.yaw = -_atan2(-Mfy, Mfx);
  }
}

inline float ahrs_get_roll_speed() {
  //return (imu.gyro_ahrs.eul.roll - ahrs.acc_err.x) * GYRO_SPEED_SCALE;
  return ahrs.acc_err.x;
  //return eg.x;
}

inline float ahrs_get_pitch_speed() {
  //return (imu.gyro_ahrs.eul.pitch - ahrs.acc_err.y) * GYRO_SPEED_SCALE;
  return ahrs.acc_err.y;
  //return eg.y;
}

inline float ahrs_get_yaw_speed() {
  //return (imu.gyro_ahrs.eul.yaw - ahrs.acc_err.z) * GYRO_SPEED_SCALE;
  return ahrs.acc_err.z;
  //return eg.z;
}

static void apply_acc_cf(){
  float ax = ahrs.acc_grav.x;
  float ay = ahrs.acc_grav.y;
  float az = ahrs.acc_grav.z;
  float norm = v3_norm(ax, ay, az);
  ax *= norm;
  ay *= norm;
  az *= norm;
  // CF
  ahrs.est_grav.x = (ahrs.est_grav.x * GYR_CMPF_FACTOR + ax) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.y = (ahrs.est_grav.y * GYR_CMPF_FACTOR + ay) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.z = (ahrs.est_grav.z * GYR_CMPF_FACTOR + az) * INV_GYR_CMPF_FACTOR;
  if (!ahrs_fast_spin()) {
    // Error
    ahrs.acc_err.x += (az * ahrs.est_grav.y - ay * ahrs.est_grav.z) * 30.0;
    ahrs.acc_err.y += (az * ahrs.est_grav.x - ax * ahrs.est_grav.z) * 30.0;
    if (MAG != _NONE_)
      ahrs.acc_err.z -= (ax * ahrs.est_grav.y - ay * ahrs.est_grav.x) * 10.0;
  }
}

static void apply_mag_cf(){
  float ax = ahrs.mag_mag.x;// - 47.36787796f;
  float ay = ahrs.mag_mag.y;// + 7.765872955f;
  float az = ahrs.mag_mag.z;
  float norm = v3_norm(ax, ay, az);
  ax *= norm;
  ay *= norm;
  az *= norm;
  // CF
  ahrs.est_mag.x = (ahrs.est_mag.x * GYR_CMPFM_FACTOR + ax) * INV_GYR_CMPFM_FACTOR;
  ahrs.est_mag.y = (ahrs.est_mag.y * GYR_CMPFM_FACTOR + ay) * INV_GYR_CMPFM_FACTOR;
  ahrs.est_mag.z = (ahrs.est_mag.z * GYR_CMPFM_FACTOR + az) * INV_GYR_CMPFM_FACTOR;
  if (!ahrs_fast_spin()) {
    // Error
    ahrs.acc_err.z -= (ax * ahrs.est_mag.y - ay * ahrs.est_mag.x) * 5.0;
  }
}

static void pre_filter_acc(){
  ahrs.acc_grav.x = ahrs.acc_grav.x * (1.0f - INV_LPF_CONST) + imu.acc.fr.x * INV_LPF_CONST;
  ahrs.acc_grav.y = ahrs.acc_grav.y * (1.0f - INV_LPF_CONST) + imu.acc.fr.y * INV_LPF_CONST;
  ahrs.acc_grav.z = ahrs.acc_grav.z * (1.0f - INV_LPF_CONST) + imu.acc.fr.z * INV_LPF_CONST;
}

static void pre_filter_mag(){
  ahrs.mag_mag.x = ahrs.mag_mag.x * (1.0f - INV_LPF_CONST) + (imu.mag.fr.x * ahrs.setup.mag_gain.x) * INV_LPF_CONST;
  ahrs.mag_mag.y = ahrs.mag_mag.y * (1.0f - INV_LPF_CONST) + (imu.mag.fr.y * ahrs.setup.mag_gain.y) * INV_LPF_CONST;
  ahrs.mag_mag.z = ahrs.mag_mag.z * (1.0f - INV_LPF_CONST) + (imu.mag.fr.z * ahrs.setup.mag_gain.z) * INV_LPF_CONST;
}

__attribute__ ((noinline))
void ahrs_reset() {
  ahrs.acc_grav.x = imu.acc.fr.x;
  ahrs.acc_grav.y = imu.acc.fr.y;
  ahrs.acc_grav.z = imu.acc.fr.z;
  float norm = InvSqrt(ahrs.acc_grav.x * ahrs.acc_grav.x + ahrs.acc_grav.y * ahrs.acc_grav.y + ahrs.acc_grav.z * ahrs.acc_grav.z);
  ahrs.est_grav.x = ahrs.acc_grav.x * norm;
  ahrs.est_grav.y = ahrs.acc_grav.y * norm;
  ahrs.est_grav.z = ahrs.acc_grav.z * norm;
  ahrs.acc_err.x = 0;
  ahrs.acc_err.y = 0;
  ahrs.acc_err.z = 0;
  if (MAG != _NONE_) {
    ahrs.mag_mag.x = imu.mag.fr.x;
    ahrs.mag_mag.y = imu.mag.fr.y;
    ahrs.mag_mag.z = imu.mag.fr.z;
    norm = InvSqrt(ahrs.mag_mag.x * ahrs.mag_mag.x + ahrs.mag_mag.y * ahrs.mag_mag.y + ahrs.mag_mag.z * ahrs.mag_mag.z);
    ahrs.est_mag.x = ahrs.mag_mag.x * norm;
    ahrs.est_mag.y = ahrs.mag_mag.y * norm;
    ahrs.est_mag.z = ahrs.mag_mag.z * norm;
  }
}

inline void AHRS_Init() {
  ahrs_reset();
  ahrs.setup.mag_gain.x = 1.0f;
  ahrs.setup.mag_gain.y = 1.0f;
  ahrs.setup.mag_gain.z = 1.0f;
}

inline void AHRS_loop_acc() {
  pre_filter_acc();
  apply_acc_cf();
  if (MAG != _NONE_) {
    pre_filter_mag();
    apply_mag_cf();
  }
  ahrs_calc_euler();
}

inline void  AHRS_loop_outer() {
  ahrs_rotate_vectors();
  ahrs_calc_ctrl_ref();
}

inline void  AHRS_loop_5hz() {
}

