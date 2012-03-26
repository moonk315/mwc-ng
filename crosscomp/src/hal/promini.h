// Arduino Pro Mini 16 Mhz, ATMega328

#include "WProgram.h"
#include <avr/pgmspace.h>

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
  #undef PROGMEM
  #define PROGMEM __attribute__((section(".progmem.data")))
#endif

#undef PSTR
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];}))

// EEPROM
#include <avr/eeprom.h>
inline void __eeprom_write_byte(uint8_t *__p, uint8_t __value) {eeprom_write_byte(__p, __value);}
inline uint8_t __eeprom_read_byte(const uint8_t *__p) {return eeprom_read_byte(__p);}
inline void __eeprom_read_block (void *__dst, const void *__src, size_t __n) {eeprom_read_block(__dst, __src, __n);}

// GUI Serial
#define AVR_USART_PORT  0
#define USART_RX_BUFFER_SIZE 32
#define USART_TX_BUFFER_SIZE 32
#include "avr_usart.h"
inline void GUI_serial_open(uint32_t baud) {avr_UsartOpen_0(baud);}
inline void GUI_serial_close() {avr_UsartClose_0();}
inline uint8_t GUI_serial_available() {return avr_UsartAvailable_0();}
inline uint8_t GUI_serial_tx_full() {return avr_UsartTXFull_0();}
inline uint8_t GUI_serial_read() {return avr_UsartRead_0();}
inline void GUI_serial_write(uint8_t c) {avr_UsartWrite_0(c);}

inline void CLI_serial_open(uint32_t baud) {avr_UsartOpen_0(baud);}
inline void CLI_serial_close() {avr_UsartClose_0();}
inline uint8_t CLI_serial_available() {return avr_UsartAvailable_0();}
inline uint8_t CLI_serial_read() {return avr_UsartRead_0();}
inline void CLI_serial_write(uint8_t c) {avr_UsartWrite_0(c);}


// TWI
#define AVR_TWI_PORT  0;
#include "avr_twi.h"
inline void i2c_write_byte(uint8_t add, uint8_t reg, uint8_t val) {avr_i2c_write_byte(add, reg, val);}
inline uint8_t i2c_read_byte(uint8_t add, uint8_t reg) {return avr_i2c_read_byte(add, reg);}

#define PT_TWI_WAIT_ACTION(act, res) \
  tm = i2c_timeout = 50; \
  TWCR = act; \
  PT_WAIT_UNTIL(pt, ((TWCR & _BV(TWINT)) || (!(tm)))); \
  if (!tm) TWCR = 0; \
  if ((!tm) || (TW_STATUS != res)) {twi_failure(); PT_EXIT(pt);};

static uint8_t i2c_timeout;
static PT_THREAD(i2c_read_buffer_pt(struct pt *pt, uint8_t add, uint8_t reg, uint8_t *buff, uint8_t len)) {
  uint8_t tm = --i2c_timeout;
  PT_BEGIN(pt);
  i2c_io_result = 0;
  // Send START
  //PT_TWI_WAIT_ACTION(TWI_ACT_START, TWI_RES_START_OK);
  // In single master mode start finishes really fast
  if (twi_act(TWI_ACT_START)  != TWI_RES_START_OK) {twi_failure(); PT_EXIT(pt);};
  // Send Device Address
  TWDR = add + 0;
  PT_TWI_WAIT_ACTION(TWI_ACT_W_ADDR, TWI_RES_W_ADDR_OK);
  // Send Device Sub-Address
  TWDR = reg;
  PT_TWI_WAIT_ACTION(TWI_ACT_W_DATA, TWI_RES_W_DATA_ACK);
  // Send RE-START
  //PT_TWI_WAIT_ACTION(TWI_ACT_RESTART, TWI_RES_RESTART_OK);
  if (twi_act(TWI_ACT_RESTART)  != TWI_RES_RESTART_OK) {twi_failure(); PT_EXIT(pt);};
  // Send Device Address (read)
  TWDR = add + 1;
  PT_TWI_WAIT_ACTION(TWI_ACT_W_ADDR, TWI_RES_R_ADDR_OK);
  // Read buffer -1
  static uint8_t i;
  for (i = 0; i < len - 1; i++) {
    PT_TWI_WAIT_ACTION(TWI_ACT_R_ACK, TWI_RES_R_ACK_OK);
    buff[i] = TWDR;
  };
  // Read buffer last
  PT_TWI_WAIT_ACTION(TWI_ACT_R_NACK, TWI_RES_R_NACK_OK);
  buff[i] = TWDR;
  // Send STOP
  TWCR = TWI_ACT_STOP;
  PT_END(pt);
}

// Delay support
#include <util\delay.h>
inline void __delay_ms(double __ms) {_delay_ms(__ms);};
inline void __delay_us(double __us) {_delay_us(__us);};


// Timers
volatile uint16_t timer1_overflow_count = 0;
static uint16_t Timer1Ovf = 2040*2-1;//2020*2;
//*22500*2;//*2020*2;


// RX input capture
#if (RX == _PPM_)
ISR(PCINT2_vect) {
  uint16_t time = TCNT1;
  uint16_t val = timer1_overflow_count;
  if ((TIFR1 & _BV(TOV1)) && (time < Timer1Ovf)) val++;
  time += val*Timer1Ovf;
  uint8_t pin  = PIND;
  static uint8_t PCintLast;
  uint8_t mask = pin ^ PCintLast;
  PCintLast = pin;
  if (mask & _BV(2)) PPMCallback(RX_CHANNEL_THROTTLE, time, (pin & _BV(2)));
  if (mask & _BV(4)) PPMCallback(RX_CHANNEL_ROLL,     time, (pin & _BV(4)));
  if (mask & _BV(5)) PPMCallback(RX_CHANNEL_PITCH,    time, (pin & _BV(5)));
  if (mask & _BV(6)) PPMCallback(RX_CHANNEL_YAW,      time, (pin & _BV(6)));
  if (mask & _BV(7)) PPMCallback(RX_CHANNEL_AUX1,     time, (pin & _BV(7)));
}
#endif

#if (RX == _PPM_SERIAL_)
ISR(INT0_vect) {
  uint16_t time = TCNT1;
  uint16_t val = timer1_overflow_count;
  if ((TIFR1 & _BV(TOV1)) && (time < Timer1Ovf)) val++;
  time += val*Timer1Ovf;
  rx_ppm_serial_callback(time);
}
#endif

inline void AttachPPM() {
  PORTD  |= _BV(2) | _BV(4) | _BV(5) | _BV(6) | _BV(7);
  PCMSK2 |= _BV(2) | _BV(4) | _BV(5) | _BV(6) | _BV(7);
  PCICR  |= _BV(2);
}

inline void AttachPPMSerial() {
  PORTD  |= _BV(2);
  EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (3 << ISC00);
  EIMSK |= (1 << INT0);
}

ISR(TIMER1_OVF_vect) {
  timer1_overflow_count++;
}

#if defined(PWM_ESC_EXT_RANGE)
void PWMOut(uint8_t ch, uint16_t val) {
  //val = val >> 3;
  uint8_t __sreg = SREG;
  cli();
  switch (ch) {
    case 0: OCR1A = (val - 1000 + 8) << 2; break;
    case 1: OCR1B = (val - 1000 + 8) << 2; break;
    case 2: OCR2A = ((val >> 2) - 250) + 2; break;
    case 3: OCR2B = ((val >> 2) - 250) + 2; break;
  }
  SREG = __sreg;
}

#else

void PWMOut(uint8_t ch, uint16_t val) {
  //val = val >> 3;
  cli();
  switch (ch) {
    case 0: OCR0A = val; break;
    case 1: OCR0B = val; break;
    case 2: OCR2A = val; break;
    case 3: OCR2B = val; break;
  }
}

#endif


void PWCOut(uint8_t ch, uint16_t val) {
  /*
  uint8_t __sreg = SREG;
  cli();
  switch (ch) {
    case 0: OCR1A = val; break;
    case 1: OCR1B = val; break;
  }
  SREG = __sreg;
  */
}

uint32_t __micros() {
  union {uint32_t val; uint8_t raw[4]; } res;
  uint8_t __sreg = SREG;
  cli();
  uint16_t t = TCNT1;
  res.val = timer1_overflow_count;
  if ((TIFR1 & _BV(TOV1)) && (t < 0xFFFF))
   res.val++;
  SREG = __sreg;
  res.val *= (Timer1Ovf >> 1);
  res.val += (t >> 1);
  res.raw[3] = 0;//  res &= 0x00FFFFFF;
  return res.val;
}

uint16_t __systick() {
  uint8_t __sreg = SREG;
  cli();
  uint16_t res = TCNT1;
  SREG = __sreg;
  return res;
}

inline uint16_t __interval(uint16_t i_start) {
  uint8_t __sreg = SREG;
  cli();
  uint16_t res = __interval(i_start, TCNT1);
  SREG = __sreg;
  return res;
}

uint16_t __interval(uint16_t i_start, uint16_t i_end) {
  if (i_end < i_start) i_end +=  Timer1Ovf;
  return (i_end - i_start);
}

inline void StatusLEDOn() {
  PORTB |= _BV(5);
}

inline void StatusLEDOff() {
  PORTB &= ~_BV(5);
}

inline void StatusLEDToggle() {
  PINB |= _BV(5);
}


inline void BeepOn() {
  PORTB |= _BV(0);
}

inline void BeepOff() {
  PORTB &= ~_BV(0);
}

inline void BeepToggle() {
  PINB |= _BV(0);
}

inline void DebugLEDOn() {
  PORTC |= _BV(1);
}

inline void DebugLEDOff() {
  PORTC &= ~_BV(1);
}

inline void DebugLEDToggle() {
  PINC |= _BV(1);
}

inline void StartBatteryVoltageMeasurement() {
  ADMUX = _BV(REFS0);
  ADCSRA |= _BV(ADSC);
}

inline uint8_t IsBatteryVoltageMeasurementFinished(){
  return !bit_is_set(ADCSRA, ADSC);
}

int16_t GetBatteryVoltage(){
  static int16_t lpf;
  uint8_t low, high;
  low  = ADCL;  high = ADCH;
  int16_t val = (((high << 8) | low) << 4);
  lpf += (val -  lpf) >> 2;
  return lpf;
}

void Board_Idle() {
  avr_UsartPollWrite_0();
};

inline void Board_Init() {
  // Timer0
  OCR0A = 2; OCR0B = 2;
  TCCR0A = (_BV(COM0A1) | _BV(COM0B1) | _BV(WGM00));
  TIMSK0 = 0;
  DDRD |= _BV(5) | _BV(6);

  // Timer1
  TCCR1A = _BV(COM1A1) |
           _BV(COM1B1) |
           _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | /* Fast PWM, ICR1 is top */
           _BV(CS11);                /* div 8 clock prescaler */
  ICR1 = Timer1Ovf; OCR1A = 0; OCR1B = 0;
  TIMSK1 |= _BV(TOIE1);
  DDRB |= _BV(1) | _BV(2);

  // Timer2
  OCR2A = 0; OCR2B = 0;
  TCCR2A |= (_BV(COM2A1) | _BV(COM2B1));
  DDRD |= _BV(3); DDRB |= _BV(3);
  // Sync Timers
  GTCCR |= _BV(PSRSYNC);
  TCNT0 = 1; TCNT2 = 0; TCNT1 = 16;

  // TWI init
  PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  TWSR = 0;
  I2C_SET_CLOCK(I2C_SPEED);
  TWCR = _BV(TWEN);  // enable twi module, no interrupt

  // Serial Init
  GUI_serial_open(SERIAL_COM_SPEED);

  // LED
  pinMode (13, OUTPUT);
  // Beeper
  pinMode (8, OUTPUT);
  // Debug
  pinMode (A1, OUTPUT);
  // Battery Monitor
  pinMode (A0, INPUT);
  DIDR0 |= _BV(ADC0D);
  ADCSRA |= _BV(ADEN);
  ADCSRA |= _BV(ADSC);
}
