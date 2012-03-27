#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"

// Cycle counter stuff - these should be defined by CMSIS, but they aren't
#define DWT_CTRL	(*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT	((volatile uint32_t *)0xE0001004)
#define CYCCNTENA	(1 << 0)

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware GPIO
#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    GPIO_Pin_12
#define BARO_GPIO   GPIOC
#define BARO_PIN    GPIO_Pin_13

// Helpful macros
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);

#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalLo(BEEP_GPIO, BEEP_PIN);

#define BARO_OFF                 digitalLo(BARO_GPIO, BARO_PIN);
#define BARO_ON                  digitalHi(BARO_GPIO, BARO_PIN);

#ifdef __cplusplus
#define ISR(x) extern "C" void x(void)
#else
#define ISR(x) void x(void)
#endif

#define _BV(bit) (1 << (bit))

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI M_PI
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

typedef const unsigned char prog_uchar;
#define pgm_read_byte_near(x) (*(prog_uchar*)x)
#define pgm_read_byte(x) (*(prog_uchar*)x)
#define __attr_flash __attribute__((section ("FLASH")))
#define PROGMEM
//__attr_flash
#define memcpy_P memcpy
#define PSTR(x) x
#define strcat_P strcat
#define printf_P printf

#include "drv_uart.h"

static struct TIM_Channel {
  TIM_TypeDef *tim;
  uint16_t channel;
  uint16_t cc;
} Channels[8] = {
  { TIM2, TIM_Channel_1, TIM_IT_CC1 },
  { TIM2, TIM_Channel_2, TIM_IT_CC2 },
  { TIM2, TIM_Channel_3, TIM_IT_CC3 },
  { TIM2, TIM_Channel_4, TIM_IT_CC4 },
  { TIM3, TIM_Channel_1, TIM_IT_CC1 },
  { TIM3, TIM_Channel_2, TIM_IT_CC2 },
  { TIM3, TIM_Channel_3, TIM_IT_CC3 },
  { TIM3, TIM_Channel_4, TIM_IT_CC4 },
};

static volatile uint16_t *OutputChannels[] = {
  &(TIM1->CCR1),
  &(TIM1->CCR4),
  &(TIM4->CCR1),
  &(TIM4->CCR2),
  &(TIM4->CCR3),
  &(TIM4->CCR4),
// Extended use during CPPM input
  &(TIM3->CCR1),
  &(TIM3->CCR2),
  &(TIM3->CCR3),
  &(TIM3->CCR4),
};


TIM_ICInitTypeDef TIM_ICInitStructure = { 0, };
GPIO_InitTypeDef GPIO_InitStructure = { 0, };
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0, };
NVIC_InitTypeDef NVIC_InitStructure = { 0, };
TIM_OCInitTypeDef TIM_OCInitStructure = { 0, };

#if (RX == _PPM_)
static void TIMXX_IRQHandler(TIM_TypeDef *tim) {
  uint16_t val;
  static uint32_t pwm_state[8] = {1, 1, 1, 1, 1, 1, 1, 1};
  for (uint8_t i = 0; i < 8; i++) {
    if ((Channels[i].tim == tim) && (TIM_GetITStatus(tim, Channels[i].cc) == SET)) {
      switch (Channels[i].channel) {
      case TIM_Channel_1:
        val = TIM_GetCapture1(tim);
        break;
      case TIM_Channel_2:
        val = TIM_GetCapture2(tim);
        break;
      case TIM_Channel_3:
        val = TIM_GetCapture3(tim);
        break;
      case TIM_Channel_4:
        val = TIM_GetCapture4(tim);
        break;
      default:
        val = 0;
      }
      TIM_ClearITPendingBit(tim, Channels[i].cc);
      //if ((val > 750 * 2) && (val < 2250 * 2))
      PPMCallback(i, val, pwm_state[i]);
      pwm_state[i] = !pwm_state[i];
      if (pwm_state[i])
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      else
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
      TIM_ICInit(tim, &TIM_ICInitStructure);
    }
  }
}

ISR(TIM2_IRQHandler) {
  TIMXX_IRQHandler(TIM2);
}

ISR(TIM3_IRQHandler) {
  TIMXX_IRQHandler(TIM3);
}
#endif


inline void AttachPPM() {
  uint32_t i;
  // Configure TIM2, TIM3 all 4 channels
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // Input timers on TIM2 and TIM3 for PWM
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  // TIM2 and TIM3 timebase
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  // PWM Input capture
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  for (i = 0; i < 8; i++) {
    TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
    TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
  }
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

#if (RX == _PPM_SERIAL_)
ISR(TIM2_IRQHandler) {
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {
    rx_ppm_serial_callback(TIM_GetCapture1(TIM2));
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
  }
}
#endif


inline void AttachPPMSerial() {
  // Configure TIM2_CH1 for PPM input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // Input timer on TIM2 only for PPM
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // TIM2 timebase
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  // Input capture on TIM2_CH1 for PPM
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  // TIM2_CH1 capture compare interrupt enable
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

void PWMOut(uint8_t ch, uint16_t val) {
  if (ch < 6) *OutputChannels[ch] = (val << 1);
}

void PWCOut(uint8_t ch, uint16_t val) {
}

static uint32_t SysTickOvf;

// SysTick
ISR(SysTick_Handler) {
}

uint16_t __systick() {
  return (SysTickOvf - SysTick->VAL) / (72 / 2);
}

inline uint16_t __interval(uint16_t i_start) {
  return __interval(i_start, __systick());
}

uint16_t __interval(uint16_t i_start, uint16_t i_end) {
  if (i_end < i_start) i_end +=  (SysTickOvf / (72 / 2));
  return (i_end - i_start);
}

inline void __delay_us(uint16_t __us) {
  __us -= 4;
  uint16_t i_start = __systick();
  while (__interval(i_start) < __us) {};
};

inline void __delay_ms(uint32_t __ms) {
  while (__ms--) __delay_us(1000);
}

inline void cli() {
  //__disable_irq();
}

inline void sei() {
  //__enable_irq();
}

inline void StatusLEDOn() {
  LED0_ON
}

inline void StatusLEDOff() {
  LED0_OFF
}

inline void StatusLEDToggle() {
  LED0_TOGGLE
}

inline void DebugLEDOn() {
  LED1_ON
}

inline void DebugLEDOff() {
  LED1_OFF
}

inline void DebugLEDToggle() {
  LED1_TOGGLE
}

inline void BeepOn() {
  BEEP_ON
}

inline void BeepOff() {
  BEEP_OFF
}

inline void BeepToggle() {
  BEEP_TOGGLE
}

void Board_Idle() {
  uartTXCheck();
};

#define PULSE_1MS       (1000) // 1ms pulse width
#define PULSE_PERIOD    (2500) // pulse period (400Hz)
#define PULSE_PERIOD_SERVO_DIGITAL  (5000) // pulse period for digital servo (200Hz)
#define PULSE_PERIOD_SERVO_ANALOG  (20000) // pulse period for analog servo (50Hz)

inline void PWM_Init(bool useServos) {
  // Output pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // Output timers
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  if (useServos) {
    // 50Hz/200Hz period on ch1, 2 for servo
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD_SERVO_ANALOG*2 - 1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD*2 - 1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  } else {
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD*2 - 1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  }
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = PULSE_1MS*2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // PWM1,2
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  // PWM3,4,5,6
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  //
  TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

#include "drv_i2c.h"

extern uint16_t twi_err_cnt __attribute__((alias("i2cErrorCount")));

inline void i2c_write_byte(uint8_t add, uint8_t reg, uint8_t val) {
  error = !i2cWrite(add, reg, val);
}

inline uint8_t i2c_read_byte(uint8_t add, uint8_t reg) {
  uint8_t buff;
  i2cReadBuffer_start(add, reg, 1, &buff);
  i2cReadBuffer_wait();
  return buff;
}

inline uint8_t i2c_trn_error() {
  return error;
}

static PT_THREAD(i2c_read_buffer_pt(struct pt *pt, uint8_t add, uint8_t reg, uint8_t *buff, uint8_t size)) {
  PT_BEGIN(pt);
  i2cReadBuffer_start(add, reg, size, buff);
  PT_WAIT_WHILE(pt, i2cReadBuffer_busy());
  PT_END(pt);
}

void adcInit() {
  ADC_InitTypeDef ADC_InitStructure;
  //
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  //
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_28Cycles5);
  ADC_Cmd(ADC1, ENABLE);
  // Calibrate ADC
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
  // Fire off ADC
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

inline void StartBatteryVoltageMeasurement() {
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

inline uint8_t IsBatteryVoltageMeasurementFinished() {
  return (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET);
}

int16_t GetBatteryVoltage() {
  static int16_t lpf;
  int16_t val = (ADC_GetConversionValue(ADC1) << 2);
  lpf += (val - lpf) >> 2;
  return lpf;
}

inline void Board_Init() {
  // Turn on clocks for stuff we use
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
  RCC_ClearFlag();
  // Make all GPIO in by default to save power and reduce noise
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  // Turn off JTAG port 'cause we're using the GPIO for leds
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  // Configure gpio
  // PB3, PB4 (LEDs)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  LED0_OFF;
  LED1_OFF;
  // PA12 (Buzzer)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  BEEP_OFF;
  SysTickOvf = SystemCoreClock >> 5;
  SysTick_Config(SysTickOvf);
  SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);
  //
  CLI_serial_open(SERIAL_COM_SPEED);
  // Output PWM
  PWM_Init(false);
  // i2c
  i2cInit(I2C2);
  // ADC
  adcInit();
}

void setup();
void loop();

int main(void) {
  setup();
  for (;;) loop();
  return 0;
}

// FLASH
#include <nvram.h>

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
extern "C" int _write(int file, char *ptr, int len) {
  int n;
  switch (file) {
  case STDOUT_FILENO: /*stdout*/
    for (n = 0; n < len; n++)
      CLI_serial_write((*ptr++ & (uint16_t)0x01FF));
    break;
  case STDERR_FILENO: /* stderr */
    for (n = 0; n < len; n++)
      CLI_serial_write((*ptr++ & (uint16_t)0x01FF));
    break;
  default:
    errno = EBADF;
    return -1;
  }
  return len;
}

