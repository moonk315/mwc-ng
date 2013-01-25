#ifndef STM32FDISC_CONF_H_INCLUDED
#define STM32FDISC_CONF_H_INCLUDED

// Frame
#define FRAME _QUADX_
#define YAW_DIRECTION 1
// ESC configuration
#define ESC   _PWM_
#define PWM_ESC_IDLE_THROTTLE 1080
#define PWM_ESC_MIN_THROTTLE  1000
#define PWM_ESC_MAX_THROTTLE  2000
// RX
#define RX    _PPM_SERIAL_
// Sensors
#define ACC  _LSM303DLHC_
#define GYRO _L3GD20_SPI_
#define MAG  _LSM303DLHC_
#define BARO _NONE_

#define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  = -X; imu.acc.fr.y  = Y; imu.acc.fr.z  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = -Y; imu.gyro_raw.eul.pitch = X; imu.gyro_raw.eul.yaw = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.mag.fr.x  = X; imu.mag.fr.y  = -Y; imu.mag.fr.z  = -Z;}


#endif // STM32FDISC_CONF_H_INCLUDED
