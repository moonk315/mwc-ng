#ifndef config_h
#define config_h

#define FRAME QUADX
#define RX    PPM_SERIAL
#define ESC   PWM

#define SERIAL_COM_SPEED 115200

// PPM ESC configuration
#define PWM_ESC_IDLE_THROTTLE 1150 
#define PWM_ESC_MIN_THROTTLE  1000
#define PWM_ESC_MAX_THROTTLE  2000
#define PWM_ESC_EXT_RANGE

#define YAW_DIRECTION 1 

#define I2C_SPEED 400000L
#define BOARD PROMINI
//#define MWC_DEBUG

#define RC_MINTHROTTLE 1000 
#define RC_MAXTHROTTLE 2000
#define DEADBAND_RP  2
#define DEADBAND_YAW 5 

#define ACC 1
#define ITG3200
#define ITG3200_LPF_256HZ
//#define L3G4200D

/* I2C accelerometer */
//#define ADXL345
//#define BMA020
#define BMA180

#define MIDRC 1500

#define ACC_ORIENTATION(X, Y, Z)  {imu.acc.fr.x  =  -X; imu.acc.fr.y  = -Y; imu.acc.fr.z  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.eul.roll = X; imu.gyro_raw.eul.pitch =  -Y; imu.gyro_raw.eul.yaw = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = -Z;}


#endif

