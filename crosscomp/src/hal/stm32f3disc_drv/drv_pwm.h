/**
 * MultiWii NG 0.1 - 2012
 * HAL for STM32f3Discovery: PWM driver
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
#ifndef DRV_PWM_H_INCLUDED
#define DRV_PWM_H_INCLUDED

#define PULSE_1MS       (1000) // 1ms pulse width
#define PULSE_PERIOD    (2500) // pulse period (400Hz)
#define PULSE_PERIOD_SERVO_DIGITAL  (5000) // pulse period for digital servo (200Hz)
#define PULSE_PERIOD_SERVO_ANALOG  (20000) // pulse period for analog servo (50Hz)

/*
 * 	OUTPUTS
	1:  TIM17_CH1 (PB5)
	2:  TIM4_CH1  (PD12)
	3:  TIM4_CH2  (PD13)
	4:  TIM4_CH3  (PD14)
	5:  TIM4_CH4  (PD15)
	6:  TIM8_CH1  (PA15)
	7:  TIM8_CH2  (PC7)
	8:  TIM8_CH3  (PC8)
	9:  TIM15_CH1 (PF9)
	10: TIM15_CH2 (PF10)
 */

static volatile uint32_t* OutputChannels[] = {
  &(TIM17->CCR1),
  &(TIM4->CCR1),
  &(TIM4->CCR2),
  &(TIM4->CCR3),
  &(TIM4->CCR4),
  &(TIM8->CCR1),

  &(TIM8->CCR2),
  &(TIM8->CCR3),
  &(TIM15->CCR1),
  &(TIM15->CCR2),
};

static void init_pwm(bool useServos) {
  // Clocks
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
  // GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  // Remap
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_10);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_3);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_3);
  // Output timers
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  if (useServos) {
    // 50Hz/200Hz period on ch1, 2 for servo
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD_SERVO_ANALOG * 2 - 1;
    TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD * 2 - 1;
    TIM_TimeBaseInit(TIM4,  &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM8,  &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  } else {
    TIM_TimeBaseStructure.TIM_Period = PULSE_PERIOD * 2 - 1;
    TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4,  &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM8,  &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  }
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = PULSE_1MS * 2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  // OC
  TIM_OC1Init(TIM17, &TIM_OCInitStructure);
  TIM_OC1Init(TIM4,  &TIM_OCInitStructure);
  TIM_OC2Init(TIM4,  &TIM_OCInitStructure);
  TIM_OC3Init(TIM4,  &TIM_OCInitStructure);
  TIM_OC4Init(TIM4,  &TIM_OCInitStructure);
  TIM_OC1Init(TIM8,  &TIM_OCInitStructure);
  TIM_OC2Init(TIM8,  &TIM_OCInitStructure);
  TIM_OC3Init(TIM8,  &TIM_OCInitStructure);
  TIM_OC1Init(TIM15, &TIM_OCInitStructure);
  TIM_OC2Init(TIM15, &TIM_OCInitStructure);
  // Enable
  TIM_Cmd(TIM4, ENABLE);
  TIM_Cmd(TIM8, ENABLE);
  TIM_Cmd(TIM15, ENABLE);
  TIM_Cmd(TIM17, ENABLE);
  // Sync Timers
  //TIM1->CNT = 0;
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
  TIM_CtrlPWMOutputs(TIM15, ENABLE);
  TIM_CtrlPWMOutputs(TIM17, ENABLE);
}

#if defined(PWM_ESC_EXT_RANGE)
void PWMOut(uint8_t ch, uint16_t val) {
  if (ch < 6) *OutputChannels[ch] = (val << 2) - 4000 + 32;
}
#else
void PWMOut(uint8_t ch, uint16_t val) {
  if (ch < 6) *OutputChannels[ch] = (val << 1);
}
#endif

void PWCOut(uint8_t ch, uint16_t val) {
}

#endif // DRV_PWM_H_INCLUDED
