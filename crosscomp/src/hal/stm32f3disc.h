/**
 * MultiWii NG 0.1 - 2012
 * HAL for STM32f3Discovery
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

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
void NMI_Handler(void) {
}

void HardFault_Handler(void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1);
}

void BusFault_Handler(void){
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1);
}

void UsageFault_Handler(void){
  while (1);
}

void SVC_Handler(void){
}

void DebugMon_Handler(void){
}

void PendSV_Handler(void){
}
/******************************************************************************/

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
#include "stm32f3disc_drv\drv_pwm.h"
#include "stm32f3disc_drv\drv_icp.h"

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

inline void init_adc() {
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  /* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  /* Enable ADC1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  /* Configure ADC Channel7 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* ADC Channel configuration */
  ADC_StructInit(&ADC_InitStructure);
  /* Calibration procedure */
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  __delay_ms(10);
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
  //calibration_value = ADC_GetCalibrationValue(ADC1);
  /* Common config */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  /* Channel config */
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
  ADC_InitStructure.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_7Cycles5);
  ADC_Cmd(ADC1, ENABLE);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  ADC_StartConversion(ADC1);
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
 /* GPIO config */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 /* Force host re-enumeration */
  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
  __delay_ms(2000);
 /* GPIO config */
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
  init_pwm(false);
  I2C_Init();
  init_adc();
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
