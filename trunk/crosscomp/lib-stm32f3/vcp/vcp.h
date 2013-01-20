/******************** (C) RUAL 2011  ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __VCP_H
#define __VCP_H

#include "platform_config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported define -----------------------------------------------------------*/
#define VCP_RX_DATA_SIZE   2048

/* Exported functions ------------------------------------------------------- */
void USB_Config(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);

void    VCP_PutChar(uint8_t Symb);
uint8_t VCP_GetChar();
uint8_t VCP_Available();
uint8_t VCP_Tx_Full();

/* External variables --------------------------------------------------------*/

#ifdef __cplusplus
 }
#endif

#endif  /*__VCP_H*/
