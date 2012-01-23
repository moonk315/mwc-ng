#ifndef WMC_global_h
#define WMC_global_h

#include <avr/pgmspace.h>

#define NONE         000 
#define PROMINI      100
#define PROMINI_HEX  200
#define PPM          300
#define PPM_SERIAL   400
#define GIMBAL       500
#define FLYING_WING  600
#define BI           700
#define TRI          800
#define QUADP        900
#define QUADX       1000

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


// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734 
#ifdef PROGMEM 
  #undef PROGMEM 
  #define PROGMEM __attribute__((section(".progmem.data"))) 
#endif 

enum enym_system_states {
    SYS_STATE_IDLE,
    SYS_STATE_ARM_REQ,
    SYS_STATE_ARMED,
    SYS_STATE_FLIGHT,
    SYS_STATE_DISARM_REQ,
    SYS_STATE_CONFIG,
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

//#define ROLL       0
//#define PITCH      1
//#define YAW        2

typedef struct rx_data rx_data_t;
struct rx_data { 
  union {
    struct {uint16_t throttle, roll, pitch, yaw, aux1, aux2, aux3, aux4;};
    uint16_t raw[RX_NUMBER_OF_CHANNELS]; 
  }; 
};
static rx_data_t rx_data;

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

typedef struct crd_eul   crd_eul_t;
struct crd_eul { 
  int16_t roll, pitch, yaw; 
};

typedef union {crd_eul_t eul; crd_fr_t fr; int16_t raw[3];} gyro_data_t;


typedef struct imu_data imu_data_t;
struct imu_data {
  gyro_data_t gyro_raw;
  union {crd_fr_t fr; int16_t raw[3];} acc;
  union {crd_fr_t fr; int16_t raw[3];} mag;
  union {crd_eul_t eul; int16_t raw[3];} gyro_offset;
  union {crd_fr_t fr; int16_t raw[3];} acc_offset;
  union {crd_fr_t fr; int16_t raw[3];} mag_offset;
  gyro_data_t gyro_decim;
  gyro_data_t gyro;
  uint8_t decim_cnt;
  uint16_t acc_1g;
  uint16_t acc_off_cal;
  uint16_t gyro_off_cal;
  uint16_t mag_off_cal;
};  
static imu_data_t imu;


typedef struct {float x, y, z;} fp_vector_t;

typedef struct ahrs_data ahrs_data_t;
struct ahrs_data {
  fp_vector_t acc_grav;
  fp_vector_t est_grav;
  fp_vector_t est_mag;
  crd_eul_t eul_ref;
};  
static ahrs_data_t ahrs;

typedef struct input_control_data input_control_data_t;
struct input_control_data {
  struct {uint8_t ctrl_rate; uint8_t ctrl_exp;} setup;
  control_data_t ctrl;
  uint8_t stick_state;
  int16_t control_expo_lookup[7];
  unsigned level_mode:1;
  unsigned mag_hh_mode:1;
  unsigned alt_hold_mode:1; 
};  
input_control_data_t input;

typedef struct pid_terms pid_terms_t;
struct pid_terms {uint8_t P, I, D; int32_t windup_min, windup_max;};  

typedef struct pid_rt pid_rt_t;
struct pid_rt {
  int16_t last_pr_error;
  int32_t i_term;
  int16_t d_term_fir[3];
  uint8_t d_term_fir_ptr; 
};  

typedef struct pid_profile pid_profile_t;
struct pid_profile {
  union {
    struct {pid_terms_t vel, roll, pitch, yaw;};
    pid_terms_t ch[CTRL_NUMBER_OF_CHANNELS]; 
  };
  pid_terms_t mag_hh;
  pid_terms_t alt_hold; 
};  

typedef struct pid_control_data pid_control_data_t;
struct pid_control_data {
  struct {pid_profile_t profile[8];} setup;
  struct {
    union {
      struct {pid_rt_t vel, roll, pitch, yaw;};
      pid_rt_t ch[CTRL_NUMBER_OF_CHANNELS]; 
    };
    pid_rt_t mag_hh;
    pid_rt_t alt_hold; 
  } rt;
  control_data_t ctrl;
  pid_profile_t *active_profile;
  unsigned locked:1;
};
pid_control_data_t pid;


typedef struct out_control_data out_control_data_t;
struct out_control_data {
  struct {} setup;
  uint16_t motor[8];
  uint16_t servo[8];
  uint8_t motor_cnt;
  uint8_t servo_cnt;
  unsigned motors_armed:1;
};
out_control_data_t out;

typedef struct flight_control_data flight_control_data_t;
struct flight_control_data {
  uint8_t  sys_state;
  uint8_t  delay_cnt;
};
flight_control_data_t flight;


static uint16_t currentTime;

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

void __delay_ms(double __ms);
void __delay_us(double __us);

// SysTick
uint32_t __micros();
uint16_t __systick();
uint8_t __systick8();
uint16_t __interval(uint16_t i_start, uint16_t i_end);
uint16_t __interval(uint16_t i_start);

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


#endif

