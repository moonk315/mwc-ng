#ifndef STM32F3DISC_H_INCLUDED
#define STM32F3DISC_H_INCLUDED

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "stm32f3disc_drv\stm32f3_discovery_l3gd20.h"
#include <nvram.h>
#include <vcp.h>



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

#define BSRR_VAL 0xC000

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware GPIO
#define LED0_GPIO   GPIOE
#define LED0_PIN    GPIO_Pin_9
#define LED1_GPIO   GPIOE
#define LED1_PIN    GPIO_Pin_13
#define BEEP_GPIO   GPIOE
#define BEEP_PIN    GPIO_Pin_11

//#define USB_DISCONNECT                      GPIOE
//#define USB_DISCONNECT_PIN                  GPIO_Pin_14
//#define RCC_AHBPeriph_GPIO_DISCONNECT       RCC_AHBPeriph_GPIOE


// Helpful macros
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalLo(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalHi(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalLo(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalHi(LED1_GPIO, LED1_PIN);

#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalLo(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalHi(BEEP_GPIO, BEEP_PIN);

//#include "drv_uart.h"

extern uint16_t twi_err_cnt __attribute__((alias("i2c_err_cnt")));


TIM_ICInitTypeDef TIM_ICInitStructure = { 0, };
GPIO_InitTypeDef GPIO_InitStructure = { 0, };
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = { 0, };
NVIC_InitTypeDef NVIC_InitStructure = { 0, };
TIM_OCInitTypeDef TIM_OCInitStructure = { 0, };
EXTI_InitTypeDef EXTI_InitStructure = { 0, };
SPI_InitTypeDef  SPI_InitStructure= { 0, };

#include "stm32f3disc_drv\drv_i2c.h"
#include "stm32f3disc_drv\drv_spi.h"

inline void GUI_serial_open(uint32_t baud) {
}

inline void GUI_serial_close() {}

inline uint8_t GUI_serial_available() {
  return VCP_Available();
}
inline uint8_t GUI_serial_read() {
  return  VCP_GetChar();
}

inline void GUI_serial_write(uint8_t c) {
  VCP_PutChar(c);
}

inline uint8_t GUI_serial_tx_full() {
  return VCP_Tx_Full();
}

static uint32_t SysTickOvf;

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

static void __delay_us(uint16_t __us) {
  __us = (__us << 1) - 4;
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
};

void PWMOut(uint8_t ch, uint16_t val) {
}

void PWCOut(uint8_t ch, uint16_t val) {
}

inline void StartBatteryVoltageMeasurement() {
  ADC_StartConversion(ADC1);
}

inline uint8_t IsBatteryVoltageMeasurementFinished() {
  return (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET);
}

int16_t GetBatteryVoltage() {
  static int16_t lpf;
  int16_t val = (ADC_GetConversionValue(ADC1) << 3);
  lpf += (val - lpf) >> 2;
  return lpf;
}

inline void init_led() {
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED0_PIN | LED1_PIN | BEEP_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

inline void init_systick() {
  SysTickOvf = SystemCoreClock >> 5;
  SysTick_Config(SysTickOvf);
  SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);
}

void USB_Cable_Config (uint8_t NewState) {
  if (NewState != 0)   {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }  else {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
}


void init_usb () {
 /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Enable the SYSCFG module clock (used for the USB disconnect feature) */

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Enable the USB disconnect GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);

 /*Set PA11,12 as IN - USB_DM,DP*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
  __delay_ms(2000);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*SET PA11,12 for USB: USB_DM,DP*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_14);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_14);

  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; /*USB resume from suspend mode*/
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  USB_Config();
}

inline void Board_Init() {
  init_led();
  init_systick();
  init_usb();
  init_spi();
  I2C_Init();
  RCC_ClearFlag();
}

void setup();
void loop();

int main(void) {
  setup();
  for (;;) loop();
  return 0;
}

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
      VCP_PutChar((*ptr++ & (uint16_t)0x01FF));
    break;
  case STDERR_FILENO: /* stderr */
    for (n = 0; n < len; n++)
     VCP_PutChar((*ptr++ & (uint16_t)0x01FF));
    break;
  default:
    errno = EBADF;
    return -1;
  }
  return len;
}


#endif // STM32F3DISC_H_INCLUDED
