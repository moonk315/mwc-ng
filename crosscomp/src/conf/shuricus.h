#ifndef SHURICUS_H_INCLUDED
#define SHURICUS_H_INCLUDED

// Frame
#define FRAME _QUADX_
#define YAW_DIRECTION 1
// ESC configuration
#define ESC   _PWM_
#define PWM_ESC_IDLE_THROTTLE 1130
#define PWM_ESC_MIN_THROTTLE  1000
#define PWM_ESC_MAX_THROTTLE  1860
// RX
#define RX    _PPM_
// Sensors
#define ACC  _ADXL345_
#define GYRO _MPU3050_
#define MAG  _NONE_
#define BARO _NONE_
#define ITG3200_LPF_98HZ
#define ADXL345_ADDRESS 0xA6

#define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  X; imu.acc.fr.y  = Y; imu.acc.fr.z  = -Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = X; imu.gyro_raw.eul.pitch =  Y; imu.gyro_raw.eul.yaw = Z;}

#endif // SHURICUS_H_INCLUDED
