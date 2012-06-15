#ifndef config_h
#define config_h

#define FRAME _QUADX_
#define RX    _PPM_SERIAL_
//#define RX    _PPM_
#define ESC   _PWM_

//#define SERIAL_COM_SPEED 256000
//#define SERIAL_COM_SPEED 115200
#define SERIAL_COM_SPEED 57600

// PPM ESC configuration
#define PWM_ESC_IDLE_THROTTLE 1080
#define PWM_ESC_MIN_THROTTLE  1000
#define PWM_ESC_MAX_THROTTLE  2000
#define PWM_ESC_EXT_RANGE

#define YAW_DIRECTION 1

#define I2C_SPEED 400000L
//#define BOARD _PROMINI_
//#define BOARD _AFROFLIGHT32_
//#define MWC_DEBUG

#define RC_MINTHROTTLE 1000
#define RC_MAXTHROTTLE 2000
#define DEADBAND_RP  2
#define DEADBAND_YAW 5

#define ACC  _ADXL345_
#define GYRO _ITG3200_
#define MAG  _NONE_
#define BARO _NONE_

#define ITG3200_LPF_98HZ
#define ADXL345_ADDRESS 0xA6


#define MIDRC 1500

#if (BOARD == _PROMINI_)
//#define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  X; imu.acc.fr.y  = -Y; imu.acc.fr.z  = Z;}
  #define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  -X; imu.acc.fr.y  = Y; imu.acc.fr.z  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = X; imu.gyro_raw.eul.pitch =  Y; imu.gyro_raw.eul.yaw = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = -Z;}
#endif

#if (BOARD == _AFROFLIGHT32_)
//  Normal
  #define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  Y; imu.acc.fr.y  = X; imu.acc.fr.z  = Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = X; imu.gyro_raw.eul.pitch =  Y; imu.gyro_raw.eul.yaw = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = -Z;}
//  Inverted
//#define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  Y; imu.acc.fr.y  = -X; imu.acc.fr.z  = -Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = X; imu.gyro_raw.eul.pitch =  -Y; imu.gyro_raw.eul.yaw = Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = -Z;}
#endif

#endif

