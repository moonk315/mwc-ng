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
#define __attr_flash __attribute__((section (".USER_FLASH")))
#define PROGMEM __attr_flash
#define memcpy_P memcpy
#define PSTR(x) x
#define strcat_P strcat

#include "drv_uart.h"

uint16_t twi_err_cnt;

inline void i2c_write_byte(uint8_t add, uint8_t reg, uint8_t val) {}
inline uint8_t i2c_read_byte(uint8_t add, uint8_t reg) {
  return 0;
}

static PT_THREAD(i2c_read_buffer_pt(struct pt *pt, uint8_t add, uint8_t reg, uint8_t *buff, uint8_t size)) {
  PT_BEGIN(pt);
  PT_SEM_WAIT(pt, &i2c_bus_mutex);
  PT_SEM_SIGNAL(pt, &i2c_bus_mutex);
  PT_END(pt);
}


inline void __delay_us(uint32_t __us) {
};

inline void __delay_ms(uint32_t __ms) {
  while (__ms--) __delay_us(1000);
}

static struct TIM_Channel {
  TIM_TypeDef *tim;
  uint16_t channel;
  uint16_t cc;
} Channels[] = {
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
};

TIM_ICInitTypeDef TIM_ICInitStructure = { 0, };
GPIO_InitTypeDef GPIO_InitStructure = { 0, };
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0, };
NVIC_InitTypeDef NVIC_InitStructure = { 0, };

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
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);
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
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);
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
}

void PWCOut(uint8_t ch, uint16_t val) {
}

// SysTick
void SysTick_Handler(void) {
}

uint16_t __systick() {
  return SysTick->VAL;
}

uint8_t __systick8() {
  return SysTick->VAL;
}

inline uint16_t __interval(uint16_t i_start) {
  return __interval(i_start, __systick());
}

uint16_t __interval(uint16_t i_start, uint16_t i_end) {
  //if (i_end < i_start) i_end +=  Timer1Ovf;
  return (i_end - i_start);
}

uint8_t i2c_trn_error() {
  return 0;
}

inline void cli() {
  __disable_irq();
}

inline void sei() {
  __enable_irq();
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
};

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
  SysTick_Config(SystemCoreClock);
  //
  CLI_serial_open(SERIAL_COM_SPEED);
}

void setup();
void loop();

int main(void) {
  setup();
  for (;;) loop();
  return 0;
}

inline void __eeprom_write_byte(uint8_t *__p, uint8_t __value) {}
inline uint8_t __eeprom_read_byte(const uint8_t *__p) {return 0;}
inline void __eeprom_read_block (void *__dst, const void *__src, size_t __n) {}

inline void StartBatteryVoltageMeasurement() {
}

inline uint8_t IsBatteryVoltageMeasurementFinished(){
  return 1;
}

int16_t GetBatteryVoltage(){
  return 0;
}

