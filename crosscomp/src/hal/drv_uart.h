#define UART_BUFFER_SIZE    64
// Receive buffer, circular DMA
static volatile uint8_t rxBuffer[UART_BUFFER_SIZE];
static volatile uint32_t rxDMAPos = 0;

static volatile uint8_t txBuffer[UART_BUFFER_SIZE];
static volatile uint32_t txBufferTail = 0;
static volatile uint32_t txBufferTailDMA = 0;
static volatile uint32_t txBufferHead = 0;

static void uartTxDMA(void) {
  DMA1_Channel4->CMAR = (uint32_t)&txBuffer[txBufferTail];
  if (txBufferHead > txBufferTail) {
    DMA1_Channel4->CNDTR = txBufferHead - txBufferTail;
    txBufferTailDMA = txBufferHead;
  } else {
    DMA1_Channel4->CNDTR = UART_BUFFER_SIZE - txBufferTail;
    txBufferTailDMA = 0;
  }

  DMA_Cmd(DMA1_Channel4, ENABLE);
}

ISR(DMA1_Channel4_IRQHandler) {
  DMA_ClearITPendingBit(DMA1_IT_TC4);
  DMA_Cmd(DMA1_Channel4, DISABLE);
  txBufferTail = txBufferTailDMA;
  if (txBufferHead != txBufferTail)
    uartTxDMA();
}

void uartInit(uint32_t baud) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  // UART
  // USART1_TX    PA9
  // USART1_RX    PA10

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // DMA TX Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // Receive DMA into a circular buffer
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel5, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  rxDMAPos = DMA_GetCurrDataCounter(DMA1_Channel5);

  // Transmit DMA
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  DMA1_Channel4->CNDTR = 0;
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

  USART_Cmd(USART1, ENABLE);
}

bool uartAvailable(void) {
  return (DMA_GetCurrDataCounter(DMA1_Channel5) != rxDMAPos) ? true : false;
}

uint8_t uartRead(void) {
  uint8_t ch;
  ch = rxBuffer[UART_BUFFER_SIZE - rxDMAPos];
  // go back around the buffer
  if (--rxDMAPos == 0)
    rxDMAPos = UART_BUFFER_SIZE;
  return ch;
}

uint8_t uartReadPoll(void) {
  while (!uartAvailable()); // wait for some bytes
  return uartRead();
}

void uartTXCheck() {
  if (txBufferHead != txBufferTail) {
    // if DMA wasn't enabled, fire it up
    if (!(DMA1_Channel4->CCR & 1)) uartTxDMA();
  }
}

void uartWrite(uint8_t ch) {
  uint32_t i = (txBufferHead + 1) % UART_BUFFER_SIZE;
  // Prevent buffer overrun
  while (i == txBufferTail) uartTXCheck();
  txBuffer[txBufferHead] = ch;
  txBufferHead = i;
}

bool uartTXFull() {
  uint32_t i = (txBufferHead + 1) % UART_BUFFER_SIZE;
  return (i == txBufferTail);
}

// Receive buffer, circular DMA
static volatile uint8_t rx2Buffer[UART_BUFFER_SIZE];
static volatile uint32_t rx2DMAPos = 0;

static volatile uint8_t tx2Buffer[UART_BUFFER_SIZE];
static volatile uint32_t tx2BufferTail = 0;
static volatile uint32_t tx2BufferTailDMA = 0;
static volatile uint32_t tx2BufferHead = 0;

static void uart2TxDMA(void) {
  DMA1_Channel7->CMAR = (uint32_t)&tx2Buffer[tx2BufferTail];
  if (tx2BufferHead > tx2BufferTail) {
    DMA1_Channel7->CNDTR = tx2BufferHead - tx2BufferTail;
    tx2BufferTailDMA = tx2BufferHead;
  } else {
    DMA1_Channel7->CNDTR = UART_BUFFER_SIZE - tx2BufferTail;
    tx2BufferTailDMA = 0;
  }
  DMA_Cmd(DMA1_Channel7, ENABLE);
}

ISR(DMA1_Channel7_IRQHandler) {
  DMA_ClearITPendingBit(DMA1_IT_TC7);
  DMA_Cmd(DMA1_Channel7, DISABLE);
  tx2BufferTail = tx2BufferTailDMA;
  if (tx2BufferHead != tx2BufferTail)
    uart2TxDMA();
}

void uart2Init(uint32_t baud) {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // USART2_TX    PA2
  // USART2_RX    PA3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // DMA TX Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_DeInit(USART2);
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  // Receive DMA into a circular buffer
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rx2Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_BufferSize = UART_BUFFER_SIZE;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel6, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  rx2DMAPos = DMA_GetCurrDataCounter(DMA1_Channel6);

  // Transmit DMA
  DMA_DeInit(DMA1_Channel7);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
  DMA1_Channel7->CNDTR = 0;
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

  USART_Cmd(USART2, ENABLE);
}

bool uart2Available(void) {
  return (DMA_GetCurrDataCounter(DMA1_Channel6) != rx2DMAPos) ? true : false;
}

uint8_t uart2Read(void) {
  uint8_t ch;
  ch = rx2Buffer[UART_BUFFER_SIZE - rx2DMAPos];
  // go back around the buffer
  if (--rx2DMAPos == 0)
    rx2DMAPos = UART_BUFFER_SIZE;
  return ch;
}

uint8_t uart2ReadPoll(void) {
  while (!uart2Available()); // wait for some bytes
  return uart2Read();
}

void uart2TXCheck() {
  if (tx2BufferHead != tx2BufferTail) {
    // if DMA wasn't enabled, fire it up
    if (!(DMA1_Channel7->CCR & 1)) uart2TxDMA();
  }
}

void uart2Write(uint8_t ch) {
  uint32_t i = (tx2BufferHead + 1) % UART_BUFFER_SIZE;
  // Prevent buffer overrun
  while (i == tx2BufferTail) uart2TXCheck();
  tx2Buffer[tx2BufferHead] = ch;
  tx2BufferHead = i;
}

bool uart2TXFull() {
  uint32_t i = (tx2BufferHead + 1) % UART_BUFFER_SIZE;
  return (i == tx2BufferTail);
}

inline void GUI_serial_open(uint32_t baud) {
  uartInit(baud);
}
inline void GUI_serial_close() {}
inline uint8_t GUI_serial_available() {
  return uartAvailable();
}
inline uint8_t GUI_serial_read() {
  return uartRead();
}
inline void GUI_serial_write(uint8_t c) {
  uartWrite(c);
}
inline uint8_t GUI_serial_tx_full() {
  return uartTXFull();
}

inline void CLI_serial_open(uint32_t baud) {
  uartInit(baud);
}
inline void CLI_serial_close() {}
inline uint8_t CLI_serial_available() {
  return uartAvailable();
}
inline uint8_t CLI_serial_read() {
  return uartRead();
}
inline void CLI_serial_write(uint8_t c) {
  uartWrite(c);
}

inline void SPK_serial_open(uint32_t baud) {
  uart2Init(baud);
}
inline void SPK_serial_close() {}
inline uint8_t SPK_serial_available() {
  return uart2Available();
}
inline uint8_t SPK_serial_read() {
  return uart2Read();
}
inline void SPK_serial_write(uint8_t c) {
  uart2Write(c);
}
inline uint8_t SPK_serial_tx_full() {
  return uart2TXFull();
}

