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
#define ACC_LPF_FACTOR 8

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 8

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 75.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 300.0f

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define INV_LPF_CONST         (1.0f/ACC_LPF_FACTOR)

#define ssin(val) (val)
#define scos(val) 1.0f

#define GYRO_SCALE ((2000.0f * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.155f * 10000.0f)  

void rotate_vectors(){
  float deltaGyroAngle;
  fp_vector_t eg = ahrs.est_grav;
  // Rotate Estimated vector(s), ROLL
  deltaGyroAngle  = imu.gyro.eul.roll * GYRO_SCALE;
  eg.y =  scos(deltaGyroAngle) * eg.y + ssin(deltaGyroAngle) * eg.z;
  eg.z = -ssin(deltaGyroAngle) * eg.y + scos(deltaGyroAngle) * eg.z;
  #if MAG
    EstM.V.Y =  scos(deltaGyroAngle) * EstM.V.Y + ssin(deltaGyroAngle) * EstM.V.Z;
    EstM.V.Z = -ssin(deltaGyroAngle) * EstM.V.Y + scos(deltaGyroAngle) * EstM.V.Z;
  #endif 
  // Rotate Estimated vector(s), PITCH
  deltaGyroAngle  = imu.gyro.eul.pitch * GYRO_SCALE;
  eg.z =  scos(deltaGyroAngle) * eg.z - ssin(deltaGyroAngle) * eg.x;
  eg.x =  ssin(deltaGyroAngle) * eg.z + scos(deltaGyroAngle) * eg.x;
  #if MAG
    EstM.V.Z =  scos(deltaGyroAngle) * EstM.V.Z - ssin(deltaGyroAngle) * EstM.V.X;
    EstM.V.X =  ssin(deltaGyroAngle) * EstM.V.Z + scos(deltaGyroAngle) * EstM.V.X;
  #endif 
  // Rotate Estimated vector(s), YAW
  deltaGyroAngle  = imu.gyro.eul.yaw * GYRO_SCALE;
  eg.x =  scos(deltaGyroAngle) * eg.x - ssin(deltaGyroAngle) * eg.y;
  eg.y =  ssin(deltaGyroAngle) * eg.x + scos(deltaGyroAngle) * eg.y;
  #if MAG
    EstM.V.X =  scos(deltaGyroAngle) * EstM.V.X - ssin(deltaGyroAngle) * EstM.V.Y;
    EstM.V.Y =  ssin(deltaGyroAngle) * EstM.V.X + scos(deltaGyroAngle) * EstM.V.Y;
  #endif 
  ahrs.eul_ref.roll  = _atan2(eg.y, eg.z);
  ahrs.eul_ref.pitch = _atan2(eg.x, eg.z);
  ahrs.est_grav = eg;
}

void apply_acc_cf(){
  ahrs.est_grav.x = (ahrs.est_grav.x * GYR_CMPF_FACTOR + ahrs.acc_grav.x) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.y = (ahrs.est_grav.y * GYR_CMPF_FACTOR + ahrs.acc_grav.y) * INV_GYR_CMPF_FACTOR;
  ahrs.est_grav.z = (ahrs.est_grav.z * GYR_CMPF_FACTOR + ahrs.acc_grav.z) * INV_GYR_CMPF_FACTOR;
}  

void pre_filter_acc(){
  ahrs.acc_grav.x = ahrs.acc_grav.x * (1.0f - INV_LPF_CONST) + imu.acc.fr.x * INV_LPF_CONST;
  ahrs.acc_grav.y = ahrs.acc_grav.y * (1.0f - INV_LPF_CONST) + imu.acc.fr.y * INV_LPF_CONST;
  ahrs.acc_grav.z = ahrs.acc_grav.z * (1.0f - INV_LPF_CONST) + imu.acc.fr.z * INV_LPF_CONST;
}  


inline void AHRS_Init() {
  ahrs.est_grav.x = 0;
  ahrs.est_grav.y = 0;
  ahrs.est_grav.z = imu.acc_1g;
  ahrs.acc_grav.x = 0;
  ahrs.acc_grav.y = 0;
  ahrs.acc_grav.z = imu.acc_1g;
}  

inline void AHRS_loop_50hz() {
  pre_filter_acc();
  apply_acc_cf();
}  

inline void  AHRS_loop_400hz() {
  rotate_vectors();
}  

