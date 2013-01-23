#ifndef DRV_SPI_H_INCLUDED
#define DRV_SPI_H_INCLUDED

DMA_InitTypeDef	DMA_InitStruct;

inline void init_spi() {
  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(L3GD20_SPI_CLK, ENABLE);
  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_SCK_GPIO_CLK | L3GD20_SPI_MISO_GPIO_CLK | L3GD20_SPI_MOSI_GPIO_CLK, ENABLE);
  /* Enable CS  GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_CS_GPIO_CLK, ENABLE);
  /* Enable INT1 GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_INT1_GPIO_CLK, ENABLE);
  /* Enable INT2 GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(L3GD20_SPI_SCK_GPIO_PORT, L3GD20_SPI_SCK_SOURCE, L3GD20_SPI_SCK_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MISO_GPIO_PORT, L3GD20_SPI_MISO_SOURCE, L3GD20_SPI_MISO_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MOSI_GPIO_PORT, L3GD20_SPI_MOSI_SOURCE, L3GD20_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_SCK_PIN;
  GPIO_Init(L3GD20_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  L3GD20_SPI_MOSI_PIN;
  GPIO_Init(L3GD20_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_MISO_PIN;
  GPIO_Init(L3GD20_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(L3GD20_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(L3GD20_SPI, &SPI_InitStructure);
  /* Configure the RX FIFO Threshold */
  SPI_RxFIFOThresholdConfig(L3GD20_SPI, SPI_RxFIFOThreshold_QF);
  /* Enable SPI1  */
  SPI_Cmd(L3GD20_SPI, ENABLE);
  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
  /* Deselect : Chip Select high */
  GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN);
  /* Setup DMA */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_PeripheralBaseAddr =	(uint32_t)&L3GD20_SPI->DR;
	DMA_InitStruct.DMA_MemoryBaseAddr = 0;
	DMA_InitStruct.DMA_BufferSize =  0;
	/* write */
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Init(DMA1_Channel3, &DMA_InitStruct);
	/* read */
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Init(DMA1_Channel2, &DMA_InitStruct);
	/* enable DMA */
	SPI_I2S_DMACmd(L3GD20_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(L3GD20_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
}

void _start_spi_dma(uint8_t* pBuffer, uint16_t NumByteToWrite) {
  // Setup buffer
  DMA1_Channel2->CMAR = (uint32_t)pBuffer;
  DMA1_Channel3->CMAR = (uint32_t)pBuffer;
  DMA1_Channel2->CNDTR = NumByteToWrite;
  DMA1_Channel3->CNDTR = NumByteToWrite;
	//
	DMA_ClearFlag(DMA1_FLAG_GL2);
	DMA_ClearFlag(DMA1_FLAG_GL3);
	/* start */
	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

void _disable_spi_dma() {
	DMA_Cmd(DMA1_Channel2, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);
}

static uint8_t _spi_xchange(uint8_t byte) {
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_TXE) == RESET) {}
  SPI_SendData8(L3GD20_SPI, byte);
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_RXNE) == RESET) {}
  return (uint8_t)SPI_ReceiveData8(L3GD20_SPI);
}

void spi_write_buff(uint8_t* buff, uint8_t reg, uint16_t size) {
  if(size > 0x01)
    reg |= (uint8_t)MULTIPLEBYTE_CMD;
  _spi_xchange(reg);
  while(size >= 0x01) {
    _spi_xchange(*buff);
    size--; buff++;
  }
}

inline void spi_write_byte(uint8_t reg, uint8_t val) {
  spi_write_buff(&val, reg, 1);
}

void spi_read_buff(uint8_t* buff, uint8_t reg, uint16_t size) {
  if(size > 0x01)
    reg |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  else
    reg |= (uint8_t)READWRITE_CMD;
  _spi_xchange(reg);
  while(size > 0x00) {
    *buff = _spi_xchange(0x00);
    size--; buff++;
  }
}

inline uint8_t spi_read_byte(uint8_t reg) {
  uint8_t buff;
  spi_read_buff(&buff, reg, 1);
  return buff;
}

static PT_THREAD(spi_read_buffer_pt(struct pt *pt, uint8_t reg, uint8_t *buff, uint8_t size)) {
  PT_BEGIN(pt);
  PT_WAIT_WHILE(pt, (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_TXE) == RESET));
  if(size > 0x01) reg |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  else reg |= (uint8_t)READWRITE_CMD;
  SPI_SendData8(L3GD20_SPI, reg);
  PT_WAIT_WHILE(pt, (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_RXNE) == RESET));
  _start_spi_dma(buff, size);
  PT_WAIT_WHILE(pt, DMA_GetCurrDataCounter(DMA1_Channel2));
  _disable_spi_dma();
  PT_END(pt);
}


#endif // DRV_SPI_H_INCLUDED
