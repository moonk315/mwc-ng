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
#define ACC_LPF_FACTOR 16

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 8

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 210.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 300.0f

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define INV_LPF_CONST         (1.0f/ACC_LPF_FACTOR)

#define ssin(val) (val)
#define scos(val) 1.0f

#define GYRO_LSB   14.375f
//#define GYRO_LSB   13.7f
#define GYRO_SCALE ((PI)/(GYRO_LSB * 180.0f * 1000000.0f) * OUTER_CTRL_LOOP_TIME / 256.0)

inline void rotate_vectors(){
  float deltaGyroAngle;
  #define eg  ahrs.est_grav
  #define em  ahrs.est_mag
  // Rotate Estimated vector(s), ROLL
  deltaGyroAngle  = imu.gyro_ahrs.eul.roll * GYRO_SCALE;
  eg.y =  scos(deltaGyroAngle) * eg.y + ssin(deltaGyroAngle) * eg.z;
  eg.z = -ssin(deltaGyroAngle) * eg.y + scos(deltaGyroAngle) * eg.z;
  if (MAG != _NONE_) {
    em.y =  scos(deltaGyroAngle) * em.y + ssin(deltaGyroAngle) * em.z;
    em.z = -ssin(deltaGyroAngle) * em.y + scos(deltaGyroAngle) * em.z;
  }
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  = imu.gyro_ahrs.eul.pitch * GYRO_SCALE;
  eg.z =  scos(deltaGyroAngle) * eg.z - ssin(deltaGyroAngle) * eg.x;
  eg.x =  ssin(deltaGyroAngle) * eg.z + scos(deltaGyroAngle) * eg.x;
  if (MAG != _NONE_) {
    em.z =  scos(deltaGyroAngle) * em.z - ssin(deltaGyroAngle) * em.x;
    em.x =  ssin(deltaGyroAngle) * em.z + scos(deltaGyroAngle) * em.x;
  }
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  = -imu.gyro_ahrs.eul.yaw * GYRO_SCALE;
  eg.x =  scos(deltaGyroAngle) * eg.x - ssin(deltaGyroAngle) * eg.y;
  eg.y =  ssin(deltaGyroAngle) * eg.x + scos(deltaGyroAngle) * eg.y;
  if (MAG != _NONE_) {
    em.x =  scos(deltaGyroAngle) * em.x - ssin(deltaGyroAngle) * em.y;
    em.y =  ssin(deltaGyroAngle) * em.x + scos(deltaGyroAngle) * em.y;
  }
  // Calculate Euler Angles
  // TODO: Move it into mavling as we are not using Angles for flight
  ahrs.eul_ref.roll  = _atan2(eg.y, eg.z);
  ahrs.eul_ref.pitch = _atan2(eg.x, eg.z);
  if (MAG != _NONE_) {
    ahrs.eul_ref.yaw = _atan2(eg.z * em.x - eg.x * em.z, eg.y * em.z - eg.z * em.y);
  }
  // Calculate level reference and apply trims
  float inv_mag = InvSqrt(eg.x * eg.x + eg.y * eg.y + eg.z * eg.z) * 2000.0f;
  ahrs.ctrl_ref.roll  = (int16_t)(eg.y * inv_mag) - ahrs.setup.level_trim.roll;
  ahrs.ctrl_ref.pitch = (int16_t)(eg.x * inv_mag) - ahrs.setup.level_trim.pitch;
}

inline void apply_acc_cf(){
  ahrs.est_grav.x = (ahrs.est_grav.x * GYR_CMPF_FACTOR + ahrs.acc_grav.x) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.y = (ahrs.est_grav.y * GYR_CMPF_FACTOR + ahrs.acc_grav.y) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.z = (ahrs.est_grav.z * GYR_CMPF_FACTOR + ahrs.acc_grav.z) * INV_GYR_CMPF_FACTOR;
}

inline void apply_mag_cf(){
  ahrs.est_mag.x = (ahrs.est_mag.x * GYR_CMPF_FACTOR + ahrs.mag_mag.x) * INV_GYR_CMPF_FACTOR;
  ahrs.est_mag.y = (ahrs.est_mag.y * GYR_CMPF_FACTOR + ahrs.mag_mag.y) * INV_GYR_CMPF_FACTOR;
  ahrs.est_mag.z = (ahrs.est_mag.z * GYR_CMPF_FACTOR + ahrs.mag_mag.z) * INV_GYR_CMPF_FACTOR;
}

inline void pre_filter_acc(){
  ahrs.acc_grav.x = ahrs.acc_grav.x * (1.0f - INV_LPF_CONST) + imu.acc.fr.x * INV_LPF_CONST;
  ahrs.acc_grav.y = ahrs.acc_grav.y * (1.0f - INV_LPF_CONST) + imu.acc.fr.y * INV_LPF_CONST;
  ahrs.acc_grav.z = ahrs.acc_grav.z * (1.0f - INV_LPF_CONST) + imu.acc.fr.z * INV_LPF_CONST;
}

inline void pre_filter_mag(){
  ahrs.mag_mag.x = ahrs.mag_mag.x * (1.0f - INV_LPF_CONST) + (imu.mag.fr.x * ahrs.setup.mag_gain.x) * INV_LPF_CONST;
  ahrs.mag_mag.y = ahrs.mag_mag.y * (1.0f - INV_LPF_CONST) + (imu.mag.fr.y * ahrs.setup.mag_gain.y) * INV_LPF_CONST;
  ahrs.mag_mag.z = ahrs.mag_mag.z * (1.0f - INV_LPF_CONST) + (imu.mag.fr.z * ahrs.setup.mag_gain.x) * INV_LPF_CONST;
}

void ahrs_reset() __attribute__ ((noinline));
void ahrs_reset() {
  ahrs.est_grav.x = 0;
  ahrs.est_grav.y = 0;
  ahrs.est_grav.z = imu.acc_1g;
  ahrs.acc_grav.x = 0;
  ahrs.acc_grav.y = 0;
  ahrs.acc_grav.z = imu.acc_1g;
}

inline void AHRS_Init() {
  ahrs_reset();
  ahrs.setup.mag_gain.x = 1.0f;
  ahrs.setup.mag_gain.y = 1.0f;
  ahrs.setup.mag_gain.x = 1.0f;
}

inline void AHRS_loop_acc() {
  pre_filter_acc();
  apply_acc_cf();
  if (MAG != _NONE_) {
    pre_filter_mag();
    apply_mag_cf();
  }
}

inline void  AHRS_loop_outer() {
  rotate_vectors();
}

