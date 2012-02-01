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

/*** I2C address ***/
#if !defined(ADXL345_ADDRESS) 
  #define ADXL345_ADDRESS 0x3A
#endif

#if !defined(BMA180_ADDRESS) 
  #define BMA180_ADDRESS 0x80
#endif

#if !defined(ITG3200_ADDRESS) 
  #define ITG3200_ADDRESS 0XD0
#endif

#if !defined(MS561101BA_ADDRESS) 
  #define MS561101BA_ADDRESS 0xEE 
#endif

//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
  #if defined(ITG3200_LPF_256HZ)
    #define ITG3200_SMPLRT_DIV 0  //8000Hz
    #define ITG3200_DLPF_CFG   0
  #endif
  #if defined(ITG3200_LPF_188HZ)
    #define ITG3200_SMPLRT_DIV 0  //1000Hz
    #define ITG3200_DLPF_CFG   1
  #endif
  #if defined(ITG3200_LPF_98HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   2
  #endif
  #if defined(ITG3200_LPF_42HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   3
  #endif
  #if defined(ITG3200_LPF_20HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   4
  #endif
  #if defined(ITG3200_LPF_10HZ)
    #define ITG3200_SMPLRT_DIV 0
    #define ITG3200_DLPF_CFG   5
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define ITG3200_SMPLRT_DIV 0  //8000Hz
    #define ITG3200_DLPF_CFG   0
#endif

union {
  uint8_t raw[6];
  struct {int16_t x, y, z;} bma_180;
  struct {int16_t x, y, z;} itg_3200;
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

// ****************
// GYRO common part
// ****************
void gyro_calc_offset() {
  static int32_t g[3];
  uint16_t gyro_off_cal_cache = imu.gyro_off_cal;
  for (uint8_t i = 0; i < 3; i++) {
    if (gyro_off_cal_cache  == 512) g[i] = 0;
    g[i] += imu.gyro_raw.raw[i];
    if (gyro_off_cal_cache == 1) {
      imu.gyro_offset.raw[i] = g[i] / 512;
      AHRS_Init();
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
  gyro_correct_offset();
  if (imu.gyro_off_cal) gyro_calc_offset();
}  

// ****************
// ACC common part
// ****************
void acc_calc_offset() {
  static int32_t a[3];  
  uint16_t acc_off_cal_cache = imu.acc_off_cal;
  for (uint8_t i = 0; i < 3; i++) {
    if (acc_off_cal_cache == 100) a[i] = 0;
    a[i] += imu.acc.raw[i];
    if (acc_off_cal_cache == 1)
      imu.acc_offset.raw[i] = a[i] / 100;
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
  acc_correct_offset();
  if (imu.acc_off_cal) acc_calc_offset();
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
#if defined(ADXL345)
void ACC_init () {
  __delay_ms(10);
  i2c_write_byte(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_write_byte(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_write_byte(ADXL345_ADDRESS,0x2C,8+4+2+1); // register: BW_RATE     -- value: 1600Hz sampling (see table 5 of the spec)
  //i2c_write_byte(ADXL345_ADDRESS,0x2C,8+4+2); // register: BW_RATE     -- value: 800Hz sampling (see table 5 of the spec)
  acc_1G = 256;
}

void ACC_getADC () {
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  //wait for data
  while (((i2c_read_byte(ADXL345_ADDRESS, 0x30) & 0x80) == 0) && !i2c_trn_error()) {};
  i2c_getSixRawADC(ADXL345_ADDRESS,0x32);
  if (!i2c_trn_error()) {
  ACC_ORIENTATION( - ((rawADC[3]<<8) | rawADC[2]) ,
                     ((rawADC[1]<<8) | rawADC[0]) ,
                     ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
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
#if defined(BMA180)
void ACC_init () {
  i2c_write_byte(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
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

inline PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt)) {return i2c_read_buffer_pt(pt, BMA180_ADDRESS, 0x02, sensor_buff.raw, 6);}  

void ACC_getADC(){
  if (!i2c_trn_error()) {
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /8 => 11 bit resolution
    ACC_ORIENTATION( sensor_buff.bma_180.x >> 2,
                     sensor_buff.bma_180.y >> 2, 
                     sensor_buff.bma_180.z >> 2);
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
#if defined(BMA020)
void ACC_init(){
  i2c_write_byte(0x70,0x15,0x80);
  uint8_t control = i2c_read_byte(0x70, 0x14);
  control = control & 0xE0;
//  control = control | (0x00 << 3); //Range 2G
  control = control | (0x03 << 3); //Range 8G
  control = control | 0x06;        //Bandwidth 1500 Hz
  i2c_write_byte(0x70,0x14,control); 
  acc_1G = 255;
}

void ACC_getADC(){
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  //wait for data
  while ((i2c_read_byte(0x70, 0x02) & 0x01) == 0) {};
  i2c_getSixRawADC(0x70,0x02);
  ACC_ORIENTATION(    ((rawADC[1]<<8) | rawADC[0])/64 ,
                      ((rawADC[3]<<8) | rawADC[2])/64 ,
                      ((rawADC[5]<<8) | rawADC[4])/64 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// standalone I2C Nunchuk
// ************************************************************************************************************
#if defined(NUNCHACK)
void ACC_init() {
  i2c_write_byte(0xA4 ,0xF0 ,0x55 );
  i2c_write_byte(0xA4 ,0xFB ,0x00 );
  __delay_ms(250);
  acc_1G = 200;
}

void ACC_getADC() {
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_getSixRawADC(0xA4,0x00);

  ACC_ORIENTATION(  ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) ) ,
                  - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) ) ,
                    ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) ));
  ACC_Common();
}
#endif

// ************************************************************************
// LIS3LV02 I2C Accelerometer
//contribution from adver (http://multiwii.com/forum/viewtopic.php?f=8&t=451)
// ************************************************************************
#if defined(LIS3LV02)
#define LIS3A  0x3A // I2C adress: 0x3A (8bit)

void i2c_ACC_init(){
  i2c_write_byte(LIS3A ,0x20 ,0xD7 ); // CTRL_REG1   1101 0111 Pwr on, 160Hz 
  i2c_write_byte(LIS3A ,0x21 ,0x50 ); // CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
  acc_1G = 256;
}

void i2c_ACC_getADC(){
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(LIS3A,0x28+0x80);
  ACC_ORIENTATION(  (rawADC[3]<<8 | rawADC[2])/4 ,
                   -(rawADC[1]<<8 | rawADC[0])/4 ,
                   -(rawADC[5]<<8 | rawADC[4])/4);
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer LSM303DLx
// contribution from wektorx (http://www.multiwii.com/forum/viewtopic.php?f=8&t=863)
// ************************************************************************************************************
#if defined(LSM303DLx_ACC)
void ACC_init () {
  __delay_ms(10);
  i2c_write_byte(0x30,0x20,0x27);
  i2c_write_byte(0x30,0x23,0x30);
  i2c_write_byte(0x30,0x21,0x00);

  acc_1G = 256;
}

  void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x30,0xA8);

  ACC_ORIENTATION( - ((rawADC[3]<<8) | rawADC[2])/16 ,
                     ((rawADC[1]<<8) | rawADC[0])/16 ,
                     ((rawADC[5]<<8) | rawADC[4])/16 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************
#if defined(L3G4200D)
void Gyro_init() {
  __delay_ms(100);
  i2c_write_byte(0XD2+0 ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  __delay_ms(5);
  i2c_write_byte(0XD2+0 ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(0XD2,0x80|0x28);

  GYRO_ORIENTATION(  ((rawADC[1]<<8) | rawADC[0])/20  ,
                     ((rawADC[3]<<8) | rawADC[2])/20  ,
                    -((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
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
#if defined(ITG3200)
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

static PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt)) {return i2c_read_buffer_pt(pt,ITG3200_ADDRESS, 0x1D, sensor_buff.raw, 6);}  

void Gyro_getADC() {
  if (!i2c_trn_error()) {
    //GYRO_ORIENTATION(bswap_16(sensor_buff.itg_3200.x) >> 2, bswap_16(sensor_buff.itg_3200.y) >> 2, bswap_16(sensor_buff.itg_3200.z) >> 2); 
    GYRO_ORIENTATION(bswap_16(sensor_buff.itg_3200.x), bswap_16(sensor_buff.itg_3200.y), bswap_16(sensor_buff.itg_3200.z)); 
    gyro_common();
  } else StatusLEDToggle(); 
}
#endif

void Sensors_Init() {
  Gyro_init();
  if (ACC) ACC_init();
}

