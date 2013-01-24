/**
 * MultiWii NG 0.1 - 2012
 * HAL for STM32f3Discovery: CPAL i2c
 * http://code.google.com/p/afrodevices/wiki/AfroFlight32
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

#ifndef DRV_I2C_H_INCLUDED
#define DRV_I2C_H_INCLUDED

#include <stm32f30x_i2c_cpal.h>

CPAL_TransferTypeDef i2c_RxStructure, i2c_TxStructure;
uint16_t i2c_err_cnt;

bool i2c_busy() {
  return (I2C1_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C1_DevStructure.CPAL_State != CPAL_STATE_ERROR);
}

void I2C_Init() {
  /* Configure the peripheral structure */
  CPAL_I2C_StructInit(&I2C1_DevStructure); /* Set all fields to default values */
  I2C1_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
  I2C1_DevStructure.wCPAL_Options =  0;//CPAL_OPT_I2C_NOSTOP_MODE;
  I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  I2C1_DevStructure.pCPAL_I2C_Struct->I2C_Timing = 0x00902025;
  I2C1_DevStructure.pCPAL_TransferRx = &i2c_RxStructure;
  I2C1_DevStructure.pCPAL_TransferTx = &i2c_TxStructure;
  /* Initialize CPAL peripheral with the selected parameters */
  CPAL_I2C_Init(&I2C1_DevStructure);
  i2c_err_cnt = 0;
}

void i2c_write_byte(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_TxStructure.pbBuffer = &val;
  i2c_TxStructure.wAddr1 = add;
  i2c_TxStructure.wAddr2 = reg;
  i2c_TxStructure.wNumData = 1;
  if (CPAL_I2C_Write(&I2C1_DevStructure) == CPAL_PASS)
    while(i2c_busy());
}

uint8_t i2c_read_byte(uint8_t add, uint8_t reg) {
  uint8_t buff;
  i2c_RxStructure.pbBuffer = &buff;
  i2c_RxStructure.wAddr1 = add;
  i2c_RxStructure.wAddr2 = reg;
  i2c_RxStructure.wNumData = 1;
  if (CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS)
    while(i2c_busy());
  return buff;
}

inline uint8_t i2c_trn_error() {
  return (I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR);
}

static PT_THREAD(i2c_read_buffer_pt(struct pt *pt, uint8_t add, uint8_t reg, uint8_t *buff, uint8_t size)) {
  PT_BEGIN(pt);
  i2c_RxStructure.pbBuffer = buff;
  i2c_RxStructure.wAddr1 = add;
  i2c_RxStructure.wAddr2 = reg;
  i2c_RxStructure.wNumData = size;
  if (CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS) {
    PT_WAIT_WHILE(pt,  i2c_busy());
  } else {
    I2C1_DevStructure.CPAL_State = CPAL_STATE_ERROR;
    i2c_err_cnt++;
  }
  PT_END(pt);
}

#endif // DRV_I2C_H_INCLUDED
