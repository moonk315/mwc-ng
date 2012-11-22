/**
 * MultiWii NG 0.1 - 2012
 * Global definitions
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

#ifndef WMC_global_h
#define WMC_global_h

////////////////////////////////////////////////////////////////////////////////
/// Setup GCC environment
//  Turn on/off warnings of interest.
//
// These warnings are normally suppressed by the Arduino IDE,
// but with some minor hacks it's possible to have warnings
// emitted.  This helps greatly when diagnosing subtle issues.
//
#pragma GCC diagnostic warning "-Wall"
#pragma GCC diagnostic warning "-Winline"
#pragma GCC diagnostic warning "-Wextra"
#pragma GCC diagnostic warning "-Wlogical-op"
#pragma GCC diagnostic ignored "-Wredundant-decls"

////////////////////////////////////////////////////////////////////////////////
///
//  Preprocessor constants
//
#define _NONE_         000
#define _PROMINI_      100
#define _AFROFLIGHT32_ 110
#define _PROMINI_HEX_  200
#define _PPM_          300
#define _PPM_SERIAL_   400
#define _GIMBAL_       500
#define _FLYING_WING_  600
#define _BI_           700
#define _TRI_          800
#define _QUADP_        900
#define _QUADX_       1000
#define _ITG3200_     1100
#define _BMA180_      1200
#define _ADXL345_     1300
#define _HMC5843_     1400
#define _BMA020_      1500
#define _HMC5883_     1600
#define _DSM_         1700
#define _DSMX_        1800
#define _MPU3050_     1900
#define _MPU6050_     2000
#define _TAROT450_    2100
#define _MINIX_       2200
#define _SHURICUS_    2300

#define STICK_STATE_TH_LOW     0
#define STICK_STATE_TH_HIGH    1
#define STICK_STATE_ROLL_LOW   2
#define STICK_STATE_ROLL_HIGH  3
#define STICK_STATE_PITCH_LOW  4
#define STICK_STATE_PITCH_HIGH 5
#define STICK_STATE_YAW_LOW    6
#define STICK_STATE_YAW_HIGH   7

#define STICK_STATE_IDLE       (_BV(STICK_STATE_TH_LOW))
#define STICK_STATE_ARM        (_BV(STICK_STATE_TH_LOW) | _BV(STICK_STATE_ROLL_HIGH))
#define STICK_STATE_DISARM     (_BV(STICK_STATE_TH_LOW) | _BV(STICK_STATE_ROLL_LOW))
#define STICK_STATE_ENTER_CONF (_BV(STICK_STATE_TH_LOW) | _BV(STICK_STATE_YAW_HIGH) | _BV(STICK_STATE_PITCH_HIGH))
#define STICK_STATE_EXIT_CONF  (_BV(STICK_STATE_TH_LOW) | _BV(STICK_STATE_YAW_LOW)  | _BV(STICK_STATE_PITCH_HIGH))
#define STICK_STATE_ENTER_TRIM (_BV(STICK_STATE_TH_HIGH))
#define STICK_STATE_EXIT_TRIM  (_BV(STICK_STATE_TH_LOW))

#define INNER_CTRL_LOOP_TIME    (4000L)
#define OUTER_CTRL_LOOP_TIME    (INNER_CTRL_LOOP_TIME * 4)
#define ACC_CTRL_LOOP_TIME      (20000L)
#define SERVICE_LOOP_TIME       (200000L)

#define __PACKED__ __attribute__((__packed__))

enum enym_system_states {
    SYS_STATE_IDLE,
    SYS_STATE_CALIBRATING,
    SYS_STATE_ARM_REQ,
    SYS_STATE_ARMED,
    SYS_STATE_FLIGHT,
    SYS_STATE_DISARM_REQ,
    SYS_STATE_CONFIG,
    SYS_STATE_LEVEL_TRIM,
    SYS_STATE_FAILSAFE,
    SYS_STATE_LAST
};

enum enum_rx_channels {
    RX_CHANNEL_THROTTLE,
    RX_CHANNEL_ROLL,
    RX_CHANNEL_PITCH,
    RX_CHANNEL_YAW,
    RX_CHANNEL_AUX1,
    RX_CHANNEL_AUX2,
    RX_CHANNEL_AUX3,
    RX_CHANNEL_AUX4,
    RX_CHANNEL_LAST,
};

#define RX_NUMBER_OF_CHANNELS (RX_CHANNEL_LAST)

enum enum_control_channels {
    CTRL_CHANNEL_THROTTLE,
    CTRL_CHANNEL_ROLL,
    CTRL_CHANNEL_PITCH,
    CTRL_CHANNEL_YAW,
    CTRL_CHANNEL_LAST,
};
#define CTRL_NUMBER_OF_CHANNELS (CTRL_CHANNEL_LAST)

enum enum_led_patterns {
    LED_PATTERN_OFF               = 0b00000000,
    LED_PATTERN_FAST_BLINK        = 0b00000010,
    LED_PATTERN_SLOW_BLINK        = 0b11001100,
    LED_PATTERN_SHORT_BLINK       = 0b00000001,
    LED_PATTERN_ON                = 0b11111111,
    LED_PATTERN_CALIBRATION_END   = 0b00010101,
    LED_PATTERN_CALIBRATION_START = 0b11011011,
    LED_PATTERN_SHORT_BANK        = 0b11100111,
};

enum enum_beep_patterns {
    BEEP_PATTERN_OFF              = 0b00000000,
    BEEP_PATTERN_FAST_BLINK       = 0b00000010,
    BEEP_PATTERN_SLOW_BLINK       = 0b11001100,
    BEEP_PATTERN_SHORT_BLINK      = 0b00000001,
    BEEP_PATTERN_ON               = 0b11111111,
    BEEP_PATTERN_CALIBRATION_END  = 0b00010101,
    BEEP_PATTERN_VBAT_W1          = 0b01001001,
    BEEP_PATTERN_VBAT_W2          = 0b01010101,
};

typedef struct rx_data rx_data_t;
struct rx_data {
  union {
    struct {uint16_t throttle, roll, pitch, yaw, aux1, aux2, aux3, aux4;};
    uint16_t raw[RX_NUMBER_OF_CHANNELS];
  };
};
rx_data_t rx_data;

typedef struct control_data control_data_t;
struct control_data {
  union {
    struct {int16_t throttle, roll, pitch, yaw;};
    int16_t raw[CTRL_NUMBER_OF_CHANNELS];
  };
};

typedef struct crd_fr crd_fr_t;
struct crd_fr {
  int16_t x, y, z;
};

typedef struct crd_fr32 crd_fr32_t;
struct crd_fr32 {
  int32_t x, y, z;
};

typedef struct crd_eul   crd_eul_t;
struct crd_eul {
  int16_t roll, pitch, yaw;
};

typedef struct crd_eul32   crd_eul32_t;
struct crd_eul32 {
  int32_t roll, pitch, yaw;
};

typedef struct crd_eulfp   crd_eulfp_t;
struct crd_eulfp {
  float roll, pitch, yaw;
};

typedef union {crd_eul_t eul; crd_fr_t fr; int16_t raw[3];} gyro_data_t;
typedef union {crd_eul32_t eul; crd_fr32_t fr; int32_t raw[3];} gyro_data32_t;


typedef struct imu_data imu_data_t;
struct imu_data {
  gyro_data_t gyro_raw;
  union {crd_fr_t fr; int16_t raw[3];} acc;
  union {crd_fr_t fr; int16_t raw[3];} mag;
  union {crd_eul_t eul; int16_t raw[3];} gyro_offset;
  union {crd_fr_t fr; int16_t raw[3];} acc_offset;
  union {crd_fr_t fr; int16_t raw[3];} mag_offset;
  gyro_data_t gyro_prev;
  gyro_data32_t gyro_ahrs;
  gyro_data_t gyro;
  uint16_t acc_1g;
  uint16_t acc_off_cal;
  uint16_t gyro_off_cal;
  uint16_t mag_off_cal;
  uint8_t mag_gain_cal;
};
imu_data_t imu;

typedef struct {float x, y, z;} fp_vector_t;

typedef struct ahrs_data ahrs_data_t;
struct ahrs_data {
  struct {
    crd_eul_t level_trim;
    fp_vector_t mag_gain;
  } setup;
  fp_vector_t acc_grav;
  fp_vector_t est_grav;
  fp_vector_t mag_mag;
  fp_vector_t est_mag;
  fp_vector_t acc_err;
  crd_eulfp_t eul_ref;
  crd_eul_t ctrl_ref;
};
ahrs_data_t ahrs;

typedef struct input_control_data input_control_data_t;
struct input_control_data {
  struct {
    uint8_t ctrl_rate;
    uint8_t ctrl_exp;
    uint8_t profile_switch;
    uint8_t profile_map[4];
  } setup;
  control_data_t ctrl;
  uint8_t stick_state;
  int16_t control_expo_lookup[7];
  uint8_t profile_val;
};
input_control_data_t input;

typedef struct pid_terms pid_terms_t;
struct pid_terms {uint8_t P, I, D, FF; int32_t i_windup;};

typedef struct pid_rt pid_rt_t;
struct pid_rt {
  int16_t last_pr_error;
  int32_t i_term;
};

typedef struct pid_channels pid_channels_t;
struct pid_channels {
  pid_terms_t roll, pitch, yaw, throttle;
};

typedef struct pid_profile pid_profile_t;
struct pid_profile {
  union {
    struct {pid_terms_t roll, pitch, yaw, throttle;};
    pid_terms_t ch[CTRL_NUMBER_OF_CHANNELS];
  } inner;
  union {
    struct {pid_terms_t roll, pitch, yaw, throttle;};
    pid_terms_t ch[CTRL_NUMBER_OF_CHANNELS];
  } outer;
};

typedef struct pid_control_data pid_control_data_t;
struct pid_control_data {
  struct {pid_profile_t profile[4];} setup;
  struct {
    union {
      struct {pid_rt_t roll, pitch, yaw, throttle;};
      pid_rt_t ch[CTRL_NUMBER_OF_CHANNELS];
    } inner;
    union {
      struct {pid_rt_t roll, pitch, yaw, throttle;};
      pid_rt_t ch[CTRL_NUMBER_OF_CHANNELS];
    } outer;
    struct {int16_t roll, pitch, yaw, throttle;} outer_pid;
  } rt;
  control_data_t ctrl;
  control_data_t ictrl_last;
  pid_profile_t *active_profile;
  unsigned locked:1;
};
pid_control_data_t pid;

typedef struct out_control_data out_control_data_t;
struct out_control_data {
  struct {} setup;
  int16_t motor[8];
  int16_t servo[8];
  uint8_t motor_cnt;
  uint8_t servo_cnt;
  unsigned motors_armed:1;
};
out_control_data_t out;

typedef struct vbat_data vbat_data_t;
struct vbat_data {
  uint8_t  voltage_scaler;
  int16_t  voltage_warn1;
  int16_t  voltage_warn2;
} __PACKED__;


typedef struct flight_control_data flight_control_data_t;
struct flight_control_data {
  struct {
    vbat_data_t vbat;
  } setup;
  uint8_t  sys_state;
  uint8_t  delay_cnt;
  uint8_t  led_pattern_req;
  uint8_t  led_pattern;
  uint8_t  beep_pattern_req;
  uint8_t  beep_pattern;

};
flight_control_data_t flight;


static uint32_t current_time_us;
static uint32_t current_time_ms;
static uint8_t  cpu_util_pct;
static uint8_t  sys_param_values_cnt;
static int16_t  batt_voltage;

// Core Function prototypes

// GUI Serial
void GUI_serial_open(uint32_t baud);
void GUI_serial_close();
uint8_t GUI_serial_available();
uint8_t GUI_serial_read();
void GUI_serial_write(uint8_t c);

// RX Serial
void RX_serial_open(uint32_t baud);
void RX_serial_close();
uint8_t RX_serial_available();
uint8_t RX_serial_read();
void RX_serial_write(uint8_t c);

// GPS Serial
void GPS_serial_open(uint32_t baud);
void GPS_serial_close();
uint8_t GPS_serial_available();
uint8_t GPS_serial_read();
void GPS_serial_write(uint8_t c);

// CLI/DEBUG Serial
void CLI_serial_open(uint32_t baud);
void CLI_serial_close();
uint8_t CLI_serial_available();
uint8_t CLI_serial_read();
void CLI_serial_write(uint8_t c);

// I2C
uint8_t i2c_read_byte(uint8_t add, uint8_t reg);
void i2c_write_byte(uint8_t add, uint8_t reg, uint8_t val);

// PWM
void PWMOut(uint8_t ch, uint16_t val); // Motors
void PWCOut(uint8_t ch, uint16_t val); // Servos

// RX input capture
void AttachPPM();
void AttachPPMSerial();
inline void rx_ppm_serial_callback(uint16_t time);
inline void PPMCallback(uint8_t ch, uint16_t time, uint8_t state);

// SysTick
inline uint16_t __systick();
inline uint16_t __interval(uint16_t i_start, uint16_t i_end);
inline uint16_t __interval(uint16_t i_start);

// Init
void Board_Init();

typedef struct timer_small timer_small_t;
struct timer_small { uint16_t elapsed, interval; uint16_t last_systick;};
typedef struct timer_big timer_big_t;
struct timer_big   { uint32_t elapsed, interval; uint16_t last_systick;};

uint8_t timer_expired(timer_small_t *t, uint16_t systick = __systick()) {
  uint16_t dt = __interval(t->last_systick, systick);
  t->last_systick = systick;
  if (t->elapsed < dt) {
    t->elapsed = t->interval;
    return 1;
  } ;
  t->elapsed -= dt;
  return 0;
};

uint8_t timer_expired(timer_big_t *t, uint16_t systick = __systick()) {
  uint16_t dt = __interval(t->last_systick, systick);
  t->last_systick = systick;
  if (t->elapsed < dt) {
    t->elapsed = t->interval;
    return 1;
  } ;
  t->elapsed -= dt;
  return 0;
};

inline static PT_THREAD(ThreadGyro_GetADC_pt(struct pt *pt));
inline static PT_THREAD(ThreadACC_GetADC_pt(struct pt *pt));

enum enum_param_type_kind {
    PARAM_TYPE_KIND_U8,
    PARAM_TYPE_KIND_U16,
    PARAM_TYPE_KIND_I16,
    PARAM_TYPE_KIND_U32,
    PARAM_TYPE_KIND_I32,
    PARAM_TYPE_KIND_STRUCT,
    PARAM_TYPE_KIND_ARRAY,
};

enum enum_param_type_encoding {
    PARAM_TYPE_ENC_GENERIC,
    PARAM_TYPE_ENC_FP_4x4,
    PARAM_TYPE_ENC_FP_0x10,
    PARAM_TYPE_ENC_FP_1x7,
    PARAM_TYPE_ENC_RCR,
    PARAM_TYPE_ENC_FPD_1000,
};

typedef struct rtti_type_info rtti_type_info_t;
typedef struct rtti_member_list rtti_member_list_t;
typedef struct rtti_type_info rtti_type_info_t;
typedef struct rtti_struct_member rtti_struct_member_t;
typedef struct param_data param_data_t;
typedef struct struct_node_search_rec struct_node_search_rec_t;
typedef struct param_search_rec param_search_rec_t;

struct rtti_struct_member {
  char  name[5];
  const rtti_type_info_t *type;
  uint8_t  encoding;
};

struct rtti_member_list {
  uint8_t  cnt;
  const rtti_struct_member_t *memb;
};

struct rtti_type_info {
  uint8_t  kind;
  uint8_t  _size;
  rtti_member_list_t members;
};

struct param_data {
  char  name[7];
  const rtti_type_info_t *type;
  void *var;
};

struct struct_node_search_rec {
  rtti_type_info_t const *type;
  uint8_t idx;
  uint8_t encoding;
};

struct param_search_rec {
  char  name[16];
  uint8_t idx: 5;
  uint8_t level: 3;
  void *inst;
  param_data_t p;
  struct struct_node_search_rec stack[8];
};

static struct pt_sem i2c_bus_mutex;

void reset_pid_state();
inline void blink_led(uint8_t pattern);
inline void beep(uint8_t pattern);
void read_storage();
void write_storage();

#endif

