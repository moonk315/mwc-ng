/**
 * MultiWii NG 0.1 - 2012
 * Sensors support. ->(imu)
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

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
  #define ACC_ORIENTATION(X, Y, Z)  {imu.acc.world.x  = X; imu.acc.world.y  = Y; imu.acc.world.z  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyro_raw.world.x  = X; imu.gyro_raw.world.y  = Y; imu.gyro_raw.world.z  = Z;}
#endif
#if !defined(MAG_ORIENTATION)
  #define MAG_ORIENTATION(X, Y, Z)  {imu.mag.world.x  = X; imu.mag.world.y  = Y; imu.mag.world.z  = Z;}
#endif

static union {
  uint8_t raw[6];
  struct {
    int16_t x, y, z;
  } bma_180;
  struct {
    int16_t x, y, z;
  } adxl345;
  struct {
    int16_t x, y, z;
  } itg_3200;
  struct {
    int16_t x, y, z;
  } mpu6050;
  struct {
    int16_t x, y, z;
  } hmc5843;
  struct {
    int16_t x, z, y;
  } hmc5883;
  struct {
    int16_t x, y, z;
  } lsm303dlhc;
  struct {
    int16_t x, z, y;
  } l3gd20;
} sensor_buff;

static inline int16_t bswap_16(int16_t x) {
  union {
    int16_t x;
    struct {
      uint8_t a, b;
    };
  } in, out;
  in.x = x;
  out.a = in.b;
  out.b = in.a;
  return out.x;
}

inline void imu_calibrate_gyro() {
  if (GYRO != _NONE_)
    imu.gyro_off_cal = 512 * 2;
}

inline void imu_calibrate_acc() {
  if (ACC != _NONE_)
    imu.acc_off_cal = 512;
}

inline void imu_calibrate_mag_gain() {
  if (MAG != _NONE_)
    imu.mag_gain_cal = 10;
}

// ****************
// GYRO common part
// ****************
void gyro_calc_offset() {
  static int32_t g[3];
  uint16_t gyro_off_cal_cache = imu.gyro_off_cal;
  for (uint8_t i = 0; i < 3; i++) {
    if (gyro_off_cal_cache  == 512 * 2) g[i] = 0;
    if (gyro_off_cal_cache  == 512)     g[i] = 0;
    g[i] += imu.gyro_raw.raw[i];
    if (gyro_off_cal_cache == 513)
      imu.gyro_offset.raw[i] = g[i] >> 9;
    if (gyro_off_cal_cache == 1) {
      if (abs(imu.gyro_offset.raw[i] - (g[i] >> 9)) > 8) {
        gyro_off_cal_cache = 512 * 2 + 1;
      }
    }
  }
  gyro_off_cal_cache--;
  imu.gyro_off_cal = gyro_off_cal_cache;
}

inline void gyro_correct_offset() {
  for (uint8_t i = 0; i < 3; i++)
    imu.gyro_raw.raw[i]  -= imu.gyro_offset.raw[i];
}

inline void gyro_common() {
  if (imu.gyro_off_cal) gyro_calc_offset();
  gyro_correct_offset();
}

// ****************
// ACC common part
// ****************
void acc_calc_offset() {
  static int32_t a[3];
  uint16_t acc_off_cal_cache = imu.acc_off_cal;
  for (uint8_t i = 0; i < 3; i++) {
    if (acc_off_cal_cache == 512) a[i] = 0;
    a[i] += imu.acc.raw[i];
    if (acc_off_cal_cache == 1)
      imu.acc_offset.raw[i] = a[i] / 512;
  }
  if (acc_off_cal_cache == 1) {
    imu.acc_offset.fr.z -= imu.acc_1g;
  }
  acc_off_cal_cache--;
  imu.acc_off_cal = acc_off_cal_cache;
}

inline void acc_correct_offset() {
  for (uint8_t i = 0; i < 3; i++)
    imu.acc.raw[i]  -= imu.acc_offset.raw[i];
}

inline void acc_common() {
  if (imu.acc_off_cal) acc_calc_offset();
  acc_correct_offset();
}

// ************************************************************************************************************
// I2C Accelerometer ADXL345
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if (ACC == _ADXL345_)

#if !defined(ADXL345_ADDRESS)
  #define ADXL345_ADDRESS 0x3a
#endif

void ACC_init () {
  __delay_ms(10);
  i2c_write_byte(ADXL345_ADDRESS, 0x2d, 1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_write_byte(ADXL345_ADDRESS, 0x31, 0x0b); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_write_byte(ADXL345_ADDRESS, 0x2c, 8+2+1); // register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)
  //i2c_write_byte(ADXL345_ADDRESS,0x2C,8+4+2+1); // register: BW_RATE     -- value: 1600Hz sampling (see table 5 of the spec)
  //i2c_write_byte(ADXL345_ADDRESS,0x2C,8+4+2); // register: BW_RATE     -- value: 800Hz sampling (see table 5 of the spec)
  imu.acc_1g = 256;
}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, ADXL345_ADDRESS, 0x32, sensor_buff.raw, 6);
}

void ACC_getADC () {
  if (!i2c_trn_error()) {
    ACC_ORIENTATION(sensor_buff.adxl345.x, sensor_buff.adxl345.y, sensor_buff.adxl345.z);
    acc_common();
  }
}
#endif

// ************************************************************************************************************
// contribution initially from opie11 (rc-groups)
// adaptation from C2po (may 2011)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC)
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:   |                                           bw<3:0> |                        tcs<3:0> |
//                   |                                             150Hz |                 !!Calibration!! |
// ************************************************************************************************************
#if (ACC == _BMA180_)

#if !defined(BMA180_ADDRESS)
  #define BMA180_ADDRESS 0x80
#endif

void ACC_init () {
  i2c_write_byte(BMA180_ADDRESS, 0x0D, 1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  __delay_ms(5);
  uint8_t control = i2c_read_byte(BMA180_ADDRESS, 0x20);
  control = control & 0x0F; // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 10Hz (bits value = 0000xxxx)
  control = control | 0x00;
  i2c_write_byte(BMA180_ADDRESS, 0x20, control);
  __delay_ms(5);
  control = i2c_read_byte(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;
  control = control | 0x00;
  i2c_write_byte(BMA180_ADDRESS, 0x30, control);
  __delay_ms(5);
  // Set range
  // Note: 2g is the default range on startup (If the EEPROM content was not changed)
  control = i2c_read_byte(BMA180_ADDRESS, 0x35);
  control &= 0xF1; // Mask offset_x calibration bits, smp_skip and clear range
  control |= (0x05 << 1); // Set range to 8g
  i2c_write_byte(BMA180_ADDRESS, 0x35, control);
  imu.acc_1g = 1024;
}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, BMA180_ADDRESS, 0x02, sensor_buff.raw, 6);
}

void ACC_getADC() {
  if (!i2c_trn_error()) {
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /8 => 11 bit resolution
    ACC_ORIENTATION(sensor_buff.bma_180.x >> 2, sensor_buff.bma_180.y >> 2, sensor_buff.bma_180.z >> 2);
    acc_common();
  }
}
#endif

// ************************************************************************************************************
// contribution from Point65 and mgros (rc-groups)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
#if (ACC == _BMA020_)
void ACC_init() {
  i2c_write_byte(0x70,0x15,0x80);
  uint8_t control = i2c_read_byte(0x70, 0x14);
  control = control & 0xE0;
//  control = control | (0x00 << 3); //Range 2G
  control = control | (0x03 << 3); //Range 8G
  control = control | 0x06;        //Bandwidth 1500 Hz
  i2c_write_byte(0x70,0x14,control);
  acc_1G = 255;
}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, 0x70, 0x02, sensor_buff.raw, 6);
}

void ACC_getADC() {
  i2c_getSixRawADC(0x70,0x02);
  ACC_ORIENTATION(    ((rawADC[1]<<8) | rawADC[0])/64 ,
                      ((rawADC[3]<<8) | rawADC[2])/64 ,
                      ((rawADC[5]<<8) | rawADC[4])/64 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer MPU6050
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// ************************************************************************************************************
#if (ACC == _MPU6050_)

#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0xd0
#endif

void ACC_init () {
  __delay_ms(10);
  i2c_write_byte(MPU6050_ADDRESS, 0x1C, 0x10);  //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  imu.acc_1g = 2048;
}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, MPU6050_ADDRESS, 0x3B, sensor_buff.raw, 6);
}

void ACC_getADC () {
  if (!i2c_trn_error()) {
    ACC_ORIENTATION(bswap_16(sensor_buff.mpu6050.x), bswap_16(sensor_buff.mpu6050.y), bswap_16(sensor_buff.mpu6050.z));
    acc_common();
  }
}
#endif

// ************************************************************************************************************
// I2C Accelerometer LSM303DLHC
// ************************************************************************************************************
#if (ACC == _LSM303DLHC_)

#define LSM303DLHC_I2C_ADDRESS               0x32
#define LSM303DLHC_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303DLHC_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303DLHC_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303DLHC_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303DLHC_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303DLHC_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303DLHC_REFERENCE_A               0x26  /* Reference register acceleration */
#define LSM303DLHC_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303DLHC_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303DLHC_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303DLHC_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303DLHC_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */
#define LSM303DLHC_FIFO_CTRL_REG_A           0x2E  /* Fifo control Register acceleration */
#define LSM303DLHC_FIFO_SRC_REG_A            0x2F  /* Fifo src Register acceleration */

#define LSM303DLHC_X_ENABLE                ((uint8_t)0x01)
#define LSM303DLHC_Y_ENABLE                ((uint8_t)0x02)
#define LSM303DLHC_Z_ENABLE                ((uint8_t)0x04)
#define LSM303DLHC_AXES_ENABLE             ((uint8_t)0x07)
#define LSM303DLHC_AXES_DISABLE            ((uint8_t)0x00)

#define LSM303DLHC_ODR_1_HZ                ((uint8_t)0x10)  /*!< Output Data Rate = 1 Hz */
#define LSM303DLHC_ODR_10_HZ               ((uint8_t)0x20)  /*!< Output Data Rate = 10 Hz */
#define LSM303DLHC_ODR_25_HZ               ((uint8_t)0x30)  /*!< Output Data Rate = 25 Hz */
#define LSM303DLHC_ODR_50_HZ               ((uint8_t)0x40)  /*!< Output Data Rate = 50 Hz */
#define LSM303DLHC_ODR_100_HZ              ((uint8_t)0x50)  /*!< Output Data Rate = 100 Hz */
#define LSM303DLHC_ODR_200_HZ              ((uint8_t)0x60)  /*!< Output Data Rate = 200 Hz */
#define LSM303DLHC_ODR_400_HZ              ((uint8_t)0x70)  /*!< Output Data Rate = 400 Hz */
#define LSM303DLHC_ODR_1620_HZ_LP          ((uint8_t)0x80)  /*!< Output Data Rate = 1620 Hz only in Low Power Mode */
#define LSM303DLHC_ODR_1344_HZ             ((uint8_t)0x90)  /*!< Output Data Rate = 1344 Hz in Normal mode and 5376 Hz in Low Power Mode */

#define LSM303DLHC_HR_ENABLE               ((uint8_t)0x08)
#define LSM303DLHC_HR_DISABLE              ((uint8_t)0x00)


#define LSM303DLHC_FULLSCALE_2G            ((uint8_t)0x00)  /*!< ▒2 g */
#define LSM303DLHC_FULLSCALE_4G            ((uint8_t)0x10)  /*!< ▒4 g */
#define LSM303DLHC_FULLSCALE_8G            ((uint8_t)0x20)  /*!< ▒8 g */
#define LSM303DLHC_FULLSCALE_16G           ((uint8_t)0x30)  /*!< ▒16 g */

#define LSM303DLHC_BlockUpdate_Continous   ((uint8_t)0x00) /*!< Continuos Update */
#define LSM303DLHC_BlockUpdate_Single      ((uint8_t)0x80) /*!< Single Update: output registers not updated until MSB and LSB reading */

#define LSM303DLHC_BLE_LSB                 ((uint8_t)0x00) /*!< Little Endian: data LSB @ lower address */
#define LSM303DLHC_BLE_MSB	               ((uint8_t)0x40) /*!< Big Endian: data MSB @ lower address */

#define LSM303DLHC_HPM_NORMAL_MODE_RES     ((uint8_t)0x00)
#define LSM303DLHC_HPM_REF_SIGNAL          ((uint8_t)0x40)
#define LSM303DLHC_HPM_NORMAL_MODE         ((uint8_t)0x80)
#define LSM303DLHC_HPM_AUTORESET_INT       ((uint8_t)0xC0)

#define LSM303DLHC_HPFCF_8                 ((uint8_t)0x00)
#define LSM303DLHC_HPFCF_16                ((uint8_t)0x10)
#define LSM303DLHC_HPFCF_32                ((uint8_t)0x20)
#define LSM303DLHC_HPFCF_64                ((uint8_t)0x30)
#define LSM303DLHC_HIGHPASSFILTER_DISABLE  ((uint8_t)0x00)
#define LSM303DLHC_HIGHPASSFILTER_ENABLE   ((uint8_t)0x08)
#define LSM303DLHC_HPF_CLICK_DISABLE       ((uint8_t)0x00)
#define LSM303DLHC_HPF_CLICK_ENABLE	       ((uint8_t)0x04)
#define LSM303DLHC_HPF_AOI1_DISABLE        ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI1_ENABLE	       ((uint8_t)0x01)
#define LSM303DLHC_HPF_AOI2_DISABLE        ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI2_ENABLE	       ((uint8_t)0x02)


void ACC_init () {
  __delay_ms(10);
  i2c_write_byte(LSM303DLHC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, LSM303DLHC_AXES_ENABLE | LSM303DLHC_ODR_50_HZ);
  i2c_write_byte(LSM303DLHC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, LSM303DLHC_HR_ENABLE | LSM303DLHC_FULLSCALE_8G| LSM303DLHC_BlockUpdate_Single | LSM303DLHC_BLE_LSB);
  i2c_write_byte(LSM303DLHC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, LSM303DLHC_HPM_NORMAL_MODE | LSM303DLHC_HPFCF_16 | LSM303DLHC_HPF_AOI1_DISABLE | LSM303DLHC_HPF_AOI2_DISABLE);
  imu.acc_1g = 4096;
}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, LSM303DLHC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A | 0x80, sensor_buff.raw, 6);
}

void ACC_getADC () {
  if (!i2c_trn_error()) {
    ACC_ORIENTATION(sensor_buff.lsm303dlhc.x, sensor_buff.lsm303dlhc.y, sensor_buff.lsm303dlhc.z);
    acc_common();
  }
}

#endif

// ************************************************************************************************************
// Dummy ACC
// ************************************************************************************************************
#if (ACC == _NONE_)
void ACC_init() {}

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {return 0;}

void ACC_getADC() {}
#endif

// ************************************************************************************************************
// I2C Gyroscope ITG3200
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if ((GYRO == _ITG3200_) || (GYRO == _MPU3050_))

#if !defined(ITG3200_ADDRESS)
  #define ITG3200_ADDRESS 0xd0
#endif

#if defined(ITG3200_LPF_256HZ)
  #define ITG3200_SMPLRT_DIV 0  //8000Hz
  #define ITG3200_DLPF_CFG   0
#elif defined(ITG3200_LPF_188HZ)
  #define ITG3200_SMPLRT_DIV 0  //1000Hz
  #define ITG3200_DLPF_CFG   1
#elif defined(ITG3200_LPF_98HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   2
#elif defined(ITG3200_LPF_42HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   3
#elif defined(ITG3200_LPF_20HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   4
#elif defined(ITG3200_LPF_10HZ)
  #define ITG3200_SMPLRT_DIV 0
  #define ITG3200_DLPF_CFG   5
#else
  //Default settings LPF 256Hz/8000Hz sample
  #define ITG3200_SMPLRT_DIV 0  //8000Hz
  #define ITG3200_DLPF_CFG   0
#endif

void Gyro_init() {
  __delay_ms(30);
  i2c_write_byte(ITG3200_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
  __delay_ms(30);
//  i2c_write_byte(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
//  __delay_ms(5);
  i2c_write_byte(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
  __delay_ms(5);
  i2c_write_byte(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
  __delay_ms(30);
}

static PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt,ITG3200_ADDRESS, 0x1D, sensor_buff.raw, 6);
}

void Gyro_getADC() {
  if (!i2c_trn_error()) {
    GYRO_ORIENTATION(bswap_16(sensor_buff.itg_3200.x), bswap_16(sensor_buff.itg_3200.y), bswap_16(sensor_buff.itg_3200.z));
    gyro_common();
  } else StatusLEDToggle();
}

inline float Gyro_getLSB() {
  #if (GYRO == _ITG3200_)
    return 14.375f;
  #endif
  #if (GYRO == _MPU3050_)
    return 16.4f;
  #endif
}

#endif

// ************************************************************************************************************
// I2C Gyroscope MPU6050
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// ************************************************************************************************************
#if ((GYRO == _MPU6050_))

#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0xd0
#endif

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    #define MPU6050_DLPF_CFG   0
#endif

void Gyro_init() {
  __delay_ms(30);
  i2c_write_byte(MPU6050_ADDRESS, 0x6B, 0x80); //register: Power Management  --  value: reset device
  __delay_ms(30);
  i2c_write_byte(MPU6050_ADDRESS, 0x6B, 0x03); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_write_byte(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG  -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_write_byte(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  __delay_ms(30);
}

static PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, MPU6050_ADDRESS, 0x43, sensor_buff.raw, 6);
}

void Gyro_getADC() {
  if (!i2c_trn_error()) {
    GYRO_ORIENTATION(bswap_16(sensor_buff.mpu6050.x), bswap_16(sensor_buff.mpu6050.y), bswap_16(sensor_buff.mpu6050.z));
    gyro_common();
  } else StatusLEDToggle();
}

inline float Gyro_getLSB() {
  return 16.4f;
}
#endif

// ************************************************************************************************************
// L3GD20 three-axis digital output gyroscope
// ************************************************************************************************************
#if (GYRO == _L3GD20_SPI_)
void Gyro_init() {
  uint8_t ctrl1 = (uint8_t)(L3GD20_MODE_ACTIVE | L3GD20_OUTPUT_DATARATE_3 | L3GD20_AXES_ENABLE | L3GD20_BANDWIDTH_4);
  uint8_t ctrl4 = (uint8_t)(L3GD20_BlockDataUpdate_Single | L3GD20_BLE_LSB | L3GD20_FULLSCALE_2000);
  L3GD20_CS_LOW();
  spi_write_byte(L3GD20_CTRL_REG1_ADDR, ctrl1);
  L3GD20_CS_HIGH();
  L3GD20_CS_LOW();
  spi_write_byte(L3GD20_CTRL_REG4_ADDR, ctrl4);
  L3GD20_CS_HIGH();
}

inline PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt)) {
  L3GD20_CS_LOW();
  return spi_read_buffer_pt(pt, L3GD20_OUT_X_L_ADDR, sensor_buff.raw, 6);
}

void Gyro_getADC() {
  L3GD20_CS_HIGH();
  GYRO_ORIENTATION(sensor_buff.l3gd20.x, sensor_buff.l3gd20.y, sensor_buff.l3gd20.z);
  gyro_common();
}

inline float Gyro_getLSB() {return 14.285f;}
#endif

// ************************************************************************************************************
// Dummy Gyro
// ************************************************************************************************************
#if (GYRO == _NONE_)
void Gyro_init() {}

inline PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt)) {return 0;}

void Gyro_getADC() {}

inline float Gyro_getLSB() {return 0.0f;}
#endif

// ************************************************************************************************************
// I2C Compass HMC5843
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if  (MAG == _HMC5843_)
#define HMC5843_ADDRESS     0x3c
#define HMC5843_GAIN        970.0f
#define HMC5843_POS_BIAS_X  0.55f
#define HMC5843_POS_BIAS_Y  0.55f
#define HMC5843_POS_BIAS_Z  0.55f

void Mag_init() {
  __delay_ms(100);
  i2c_write_byte(HMC5843_ADDRESS, 0x00, 0x18);  // 50Hz, Normal
  i2c_write_byte(HMC5843_ADDRESS, 0x01, 0x40);  // 1.5 GA range, 970 cnt/Ga gain
  i2c_write_byte(HMC5843_ADDRESS, 0x02, 0x00);  // Continous conversion mode
  __delay_ms(100);
}

void Mag_getADC() {
  if (!i2c_trn_error()) {
    MAG_ORIENTATION(bswap_16(sensor_buff.hmc5843.x), bswap_16(sensor_buff.hmc5843.y), bswap_16(sensor_buff.hmc5843.z));
  } else StatusLEDToggle();
}

static PT_THREAD(ThreadMag_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, HMC5843_ADDRESS, 0x03, sensor_buff.raw, 6);
}

void Mag_calibrate_gain_start() {
  i2c_write_byte(HMC5843_ADDRESS, 0x00, 0x19);  // 50Hz, Positive Bias
  i2c_write_byte(HMC5843_ADDRESS, 0x01, 0x40);  // 1.5 GA range, 970 cnt/Ga gain
  i2c_write_byte(HMC5843_ADDRESS, 0x02, 0x00);  // ??? Continous conversion mode ???
}

void Mag_calibrate_gain_end() {
  i2c_write_byte(HMC5843_ADDRESS, 0x00, 0x18);  // 50Hz, Normal
  i2c_write_byte(HMC5843_ADDRESS, 0x01, 0x40);  // 1.5 GA range, 970 cnt/Ga gain
  i2c_write_byte(HMC5843_ADDRESS, 0x02, 0x00);  // Continous conversion mode
  ahrs.setup.mag_gain.x = (HMC5843_POS_BIAS_X * HMC5843_GAIN) / abs(imu.mag.fr.x);
  ahrs.setup.mag_gain.y = (HMC5843_POS_BIAS_Y * HMC5843_GAIN) / abs(imu.mag.fr.y);
  ahrs.setup.mag_gain.z = (HMC5843_POS_BIAS_Z * HMC5843_GAIN) / abs(imu.mag.fr.z);
}
#endif

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if  (MAG == _HMC5883_)
#define HMC5883_ADDRESS     0x3c
#define HMC5883_GAIN        660.0f
#define HMC5883_POS_BIAS_X  1.16f
#define HMC5883_POS_BIAS_Y  1.08f
#define HMC5883_POS_BIAS_Z  1.08f

void Mag_init() {
  __delay_ms(100);
  i2c_write_byte(HMC5883_ADDRESS, 0x00, 0x74);  // 8 samples avg, 30Hz, Normal
  i2c_write_byte(HMC5883_ADDRESS, 0x01, 0x40);  // 1.9 GA range, 820 cnt/Ga gain
  i2c_write_byte(HMC5883_ADDRESS, 0x02, 0x00);  // Continous conversion mode
}

void Mag_getADC() {
  if (!i2c_trn_error()) {
    MAG_ORIENTATION(bswap_16(sensor_buff.hmc5883.x), bswap_16(sensor_buff.hmc5883.y), bswap_16(sensor_buff.hmc5883.z));
  } else StatusLEDToggle();
}

static PT_THREAD(ThreadMag_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, HMC5883_ADDRESS, 0x03, sensor_buff.raw, 6);
}

void Mag_calibrate_gain_start() {
  i2c_write_byte(HMC5883_ADDRESS, 0x00, 0x75);  // 8 samples avg, 30Hz, Positive Bias
  i2c_write_byte(HMC5883_ADDRESS, 0x01, 0x60);  // 2.5 GA range, 660 cnt/Ga gain
  i2c_write_byte(HMC5883_ADDRESS, 0x02, 0x01);  // Single conversion mode
}

void Mag_calibrate_gain_end() {
  i2c_write_byte(HMC5883_ADDRESS, 0x00, 0x74);  // 8 samples avg, 30Hz, Normal
  i2c_write_byte(HMC5883_ADDRESS, 0x01, 0x60);  // 1.9 GA range, 820 cnt/Ga gain
  i2c_write_byte(HMC5883_ADDRESS, 0x02, 0x00);  // Continous conversion mode
  ahrs.setup.mag_gain.x = (HMC5883_POS_BIAS_X * HMC5883_GAIN) / abs(imu.mag.fr.x);
  ahrs.setup.mag_gain.y = (HMC5883_POS_BIAS_Y * HMC5883_GAIN) / abs(imu.mag.fr.y);
  ahrs.setup.mag_gain.z = (HMC5883_POS_BIAS_Z * HMC5883_GAIN) / abs(imu.mag.fr.z);
}
#endif


// ************************************************************************************************************
// I2C Compass LSM303DLHC
// ************************************************************************************************************
#if  (MAG == _LSM303DLHC_)

#define LSM303DLHC_MAG_I2C_ADDRESS          0x3C
#define LSM303DLHC_ODR_0_75_HZ              ((uint8_t) 0x00)  /*!< Output Data Rate = 0.75 Hz */
#define LSM303DLHC_ODR_1_5_HZ               ((uint8_t) 0x04)  /*!< Output Data Rate = 1.5 Hz */
#define LSM303DLHC_ODR_3_0_HZ               ((uint8_t) 0x08)  /*!< Output Data Rate = 3 Hz */
#define LSM303DLHC_ODR_7_5_HZ               ((uint8_t) 0x0C)  /*!< Output Data Rate = 7.5 Hz */
#define LSM303DLHC_ODR_15_HZ                ((uint8_t) 0x10)  /*!< Output Data Rate = 15 Hz */
#define LSM303DLHC_ODR_30_HZ                ((uint8_t) 0x14)  /*!< Output Data Rate = 30 Hz */
#define LSM303DLHC_ODR_75_HZ                ((uint8_t) 0x18)  /*!< Output Data Rate = 75 Hz */
#define LSM303DLHC_ODR_220_HZ               ((uint8_t) 0x1C)  /*!< Output Data Rate = 220 Hz */

#define  LSM303DLHC_FS_1_3_GA               ((uint8_t) 0x20)  /*!< Full scale = ▒1.3 Gauss */
#define  LSM303DLHC_FS_1_9_GA               ((uint8_t) 0x40)  /*!< Full scale = ▒1.9 Gauss */
#define  LSM303DLHC_FS_2_5_GA               ((uint8_t) 0x60)  /*!< Full scale = ▒2.5 Gauss */
#define  LSM303DLHC_FS_4_0_GA               ((uint8_t) 0x80)  /*!< Full scale = ▒4.0 Gauss */
#define  LSM303DLHC_FS_4_7_GA               ((uint8_t) 0xA0)  /*!< Full scale = ▒4.7 Gauss */
#define  LSM303DLHC_FS_5_6_GA               ((uint8_t) 0xC0)  /*!< Full scale = ▒5.6 Gauss */
#define  LSM303DLHC_FS_8_1_GA               ((uint8_t) 0xE0)  /*!< Full scale = ▒8.1 Gauss */

#define LSM303DLHC_M_SENSITIVITY_XY_1_3Ga     1100  /*!< magnetometer X Y axes sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_1_9Ga     855   /*!< magnetometer X Y axes sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_2_5Ga     670   /*!< magnetometer X Y axes sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_4Ga       450   /*!< magnetometer X Y axes sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_4_7Ga     400   /*!< magnetometer X Y axes sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_5_6Ga     330   /*!< magnetometer X Y axes sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_8_1Ga     230   /*!< magnetometer X Y axes sensitivity for 8.1 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_1_3Ga      980   /*!< magnetometer Z axis sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_1_9Ga      760   /*!< magnetometer Z axis sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_2_5Ga      600   /*!< magnetometer Z axis sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_4Ga        400   /*!< magnetometer Z axis sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_4_7Ga      355   /*!< magnetometer Z axis sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_5_6Ga      295   /*!< magnetometer Z axis sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_8_1Ga      205   /*!< magnetometer Z axis sensitivity for 8.1 Ga full scale [LSB/Ga] */

#define LSM303DLHC_CONTINUOS_CONVERSION      ((uint8_t) 0x00)   /*!< Continuous-Conversion Mode */
#define LSM303DLHC_SINGLE_CONVERSION         ((uint8_t) 0x01)   /*!< Single-Conversion Mode */
#define LSM303DLHC_SLEEP                     ((uint8_t) 0x02)   /*!< Sleep Mode */

#define LSM303DLHC_TEMPSENSOR_ENABLE         ((uint8_t) 0x80)   /*!< Temp sensor Enable */
#define LSM303DLHC_TEMPSENSOR_DISABLE        ((uint8_t) 0x00)   /*!< Temp sensor Disable */

/* Magnetic field Registers */
#define LSM303DLHC_CRA_REG_M                 0x00  /* Control register A magnetic field */
#define LSM303DLHC_CRB_REG_M                 0x01  /* Control register B magnetic field */
#define LSM303DLHC_MR_REG_M                  0x02  /* Control register MR magnetic field */
#define LSM303DLHC_OUT_X_H_M                 0x03  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_X_L_M                 0x04  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_Z_H_M                 0x05  /* Output Register Z magnetic field */
#define LSM303DLHC_OUT_Z_L_M                 0x06  /* Output Register Z magnetic field */
#define LSM303DLHC_OUT_Y_H_M                 0x07  /* Output Register Y magnetic field */
#define LSM303DLHC_OUT_Y_L_M                 0x08  /* Output Register Y magnetic field */


void Mag_init() {
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;
  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_TEMPSENSOR_DISABLE | LSM303DLHC_ODR_75_HZ);
  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_FS_2_5_GA);
  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_CONTINUOS_CONVERSION);
  /* Write value to Mag MEMS CRA_REG regsister */
  i2c_write_byte(LSM303DLHC_MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, cra_regm);
  /* Write value to Mag MEMS CRB_REG regsister */
  i2c_write_byte(LSM303DLHC_MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, crb_regm);
  /* Write value to Mag MEMS MR_REG regsister */
  i2c_write_byte(LSM303DLHC_MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, mr_regm);
}

static PT_THREAD(ThreadMag_GetADC_pt(struct pt *pt)) {
  return i2c_read_buffer_pt(pt, LSM303DLHC_MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, sensor_buff.raw, 6);
}

void Mag_getADC() {
  if (!i2c_trn_error()) {
    MAG_ORIENTATION(bswap_16(sensor_buff.lsm303dlhc.x), bswap_16(sensor_buff.lsm303dlhc.y), bswap_16(sensor_buff.lsm303dlhc.z));
  } else StatusLEDToggle();
}

void Mag_calibrate_gain_start() {}

void Mag_calibrate_gain_end() {
  ahrs.setup.mag_gain.x = 1.0f;
  ahrs.setup.mag_gain.y = 1.0f;
  ahrs.setup.mag_gain.z = 1.0f;
}
#endif

// ************************************************************************************************************
// Dummy Compass
// ************************************************************************************************************
#if  (MAG == _NONE_)
void Mag_init() {}

static PT_THREAD(ThreadMag_GetADC_pt(struct pt *pt)) {return 0;}

void Mag_getADC() {}

void Mag_calibrate_gain_start() {}

void Mag_calibrate_gain_end() {}
#endif

void Sensors_Init() {
  if (GYRO != _NONE_) Gyro_init();
  if (ACC  != _NONE_) ACC_init();
  if (MAG  != _NONE_) Mag_init();
}

