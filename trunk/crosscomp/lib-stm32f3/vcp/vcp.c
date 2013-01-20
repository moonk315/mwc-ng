#include "platform_config.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_conf.h"
#include "usb_pwr.h"
#include "vcp.h"
#include "usb_type.h"
#include "usb_istr.h"

ErrorStatus HSEStartUpStatus;

void Set_USBClock(void) {
  /* USBCLK = PLLCLK = 48 MHz */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

void USB_Interrupts_Config(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /* Enable the USB interrupt */
#if defined (USB_INT_DEFAULT)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
#endif
#if defined (USB_INT_REMAP)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
#endif
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the USB Wake-up interrupt */
#if defined (USB_INT_DEFAULT)
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
#endif
#if defined (USB_INT_REMAP)
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_RMP_IRQn;
#endif
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void USB_Config() {
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
}

void VCP_PutChar(uint8_t ch) {
  if (bDeviceState != CONFIGURED) return;
  while (VCP_Tx_Full());
  NVIC_DisableIRQ (USB_LP_CAN1_RX0_IRQn);
  VCP_Rx_Buffer[VCP_Rx_ptr_in++] = ch;
  if(VCP_Rx_ptr_in == VCP_RX_DATA_SIZE) VCP_Rx_ptr_in = 0;
  NVIC_EnableIRQ (USB_LP_CAN1_RX0_IRQn);
}

uint8_t VCP_Tx_Full() {
  return (USB_Tx_State == 1);
}

uint8_t VCP_Available() {
  return (pGetChar_ptr_out != pGetChar_ptr_max);
}

uint8_t VCP_GetChar() {
  if (bDeviceState != CONFIGURED) return 0;
  uint8_t Symb = 0;
  NVIC_DisableIRQ (USB_LP_CAN1_RX0_IRQn);
  if  (pGetChar_ptr_out  < pGetChar_ptr_max)
    Symb = *(pGetChar_ptr_out++);
  NVIC_EnableIRQ (USB_LP_CAN1_RX0_IRQn);
  return Symb;
}

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f30x.s).                                            */
/******************************************************************************/
#if defined (USB_INT_DEFAULT)
void USB_LP_CAN1_RX0_IRQHandler(void)
#elif defined (USB_INT_REMAP)
void USB_LP_IRQHandler(void)
#endif
{
   USB_Istr();
}

#if defined (USB_INT_DEFAULT)
void USBWakeUp_IRQHandler(void)
#elif defined (USB_INT_REMAP)
void USBWakeUp_RMP_IRQHandler(void)
#endif
{
  /* Initiate external resume sequence (1 step) */
  Resume(RESUME_EXTERNAL);
  EXTI_ClearITPendingBit(EXTI_Line18);
}


