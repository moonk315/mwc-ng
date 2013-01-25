/**
 * MultiWii NG 0.1 - 2012
 * HAL for STM32f3Discovery: ICP driver
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

#ifndef DRV_ICP_H_INCLUDED
#define DRV_ICP_H_INCLUDED

/*
 * 	INPUTS
	1:  TIM16_CH1 (PB8) (PPM_SERIAL)
	2:  TIM1_CH1  (PA8)
	3:  TIM1_CH2  (PA9)
	4:  TIM2_CH2  (PA1)
	5:  TIM2_CH3  (PB10)
	6:  TIM2_CH4  (PB11)
	7:  TIM3_CH1  (PC6)
	8:  TIM3_CH2  (PA4)
	9:  TIM3_CH3  (PB0)
	10: TIM3_CH4  (PB1)
 */

#define __ICP_CHANNELS_CNT__      (10)

static struct TIM_Channel {
  TIM_TypeDef *tim;
  uint16_t channel;
  uint16_t cc;
  bool pwm_state;
} Channels[__ICP_CHANNELS_CNT__] = {
  { TIM16, TIM_Channel_1, TIM_IT_CC1, 1 },
  { TIM1,  TIM_Channel_1, TIM_IT_CC1, 1 },
  { TIM1,  TIM_Channel_2, TIM_IT_CC2, 1 },
  { TIM2,  TIM_Channel_2, TIM_IT_CC2, 1 },
  { TIM2,  TIM_Channel_3, TIM_IT_CC3, 1 },
  { TIM2,  TIM_Channel_4, TIM_IT_CC4, 1 },
  { TIM3,  TIM_Channel_1, TIM_IT_CC1, 1 },
  { TIM3,  TIM_Channel_2, TIM_IT_CC2, 1 },
  { TIM3,  TIM_Channel_3, TIM_IT_CC3, 1 },
  { TIM3,  TIM_Channel_4, TIM_IT_CC4, 1 },
};

#if (RX == _PPM_SERIAL_)
ISR(TIM1_UP_TIM16_IRQHandler) {
  if (TIM_GetITStatus(TIM16, TIM_IT_CC1) == SET) {
    rx_ppm_serial_callback(TIM_GetCapture1(TIM16));
    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
  }
}
#endif

#if (RX == _PPM_)
static void TIMXX_IRQHandler(TIM_TypeDef *tim) {
  uint16_t val;
  for (uint8_t i = 0; i < __ICP_CHANNELS_CNT__; i++) {
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
      if (i < 8) PPMCallback(i, val, Channels[i].pwm_state);
      Channels[i].pwm_state = !Channels[i].pwm_state;
      if (Channels[i].pwm_state)
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      else
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
      TIM_ICInit(tim, &TIM_ICInitStructure);
    }
  }
}

ISR(TIM1_CC_IRQHandler) {
  TIMXX_IRQHandler(TIM1);
}

ISR(TIM2_IRQHandler) {
  TIMXX_IRQHandler(TIM2);
}

ISR(TIM3_IRQHandler) {
  TIMXX_IRQHandler(TIM3);
}

ISR(TIM1_UP_TIM16_IRQHandler) {
  TIMXX_IRQHandler(TIM16);
}
#endif

inline void AttachPPMSerial() {
  // Clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  // GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // Remap
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
  // Input timer on TIM16 only for PPM
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // TIM16 timebase
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
  // Input capture on TIM16_CH1 for PPM
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM16, &TIM_ICInitStructure);
  // TIM16_CH1 capture compare interrupt enable
  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
  TIM_Cmd(TIM16, ENABLE);
}

inline void AttachPPM() {
  // Clocks
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM16, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
  // GPIO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  // Remap
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
  // Input timers on TIM1,2,3,16
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  // TIM1,2,3,16 timebase
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = ((72 / 2) - 1);
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
  // PWM Input capture
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  for (uint32_t i = 0; i < __ICP_CHANNELS_CNT__; i++) {
    TIM_ICInitStructure.TIM_Channel = Channels[i].channel;
    TIM_ICInit(Channels[i].tim, &TIM_ICInitStructure);
  }
  TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
  //
  TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM16, ENABLE);
}
#endif // DRV_ICP_H_INCLUDED
