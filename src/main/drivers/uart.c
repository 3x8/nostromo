#include "uart.h"

uart_t serialPort;

void uartStartTxDMA(void) {
  LL_DMA_SetMemoryAddress(DMA1, USART_TX_DMA_CHANNEL, (uint32_t)&serialPort.txBuf[serialPort.txTail]);
  if (serialPort.txHead > serialPort.txTail) {
    LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, serialPort.txHead - serialPort.txTail);
    serialPort.txTail = serialPort.txHead;
  } else {
    LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, SERIAL_TX_BUFSIZE - serialPort.txTail);
    serialPort.txTail = 0;
  }
  LL_DMA_EnableChannel(DMA1, USART_TX_DMA_CHANNEL);
}

void uartWrite(char ch) {
  LL_USART_SetTransferDirection(USART,LL_USART_DIRECTION_TX);
  serialPort.txBuf[serialPort.txHead] = ch;
  serialPort.txHead = (serialPort.txHead + 1) % SERIAL_TX_BUFSIZE;

  if (!LL_DMA_IsEnabledChannel(DMA1, USART_TX_DMA_CHANNEL)) {
    uartStartTxDMA();
  }
  LL_USART_SetTransferDirection(USART, LL_USART_DIRECTION_RX);
}

void uartPrint(const char *str) {
  char ch;

  while ((ch = *(str++)) != 0) {
    uartWrite(ch);
  }
}

void uartPrintInteger(uint32_t n, uint8_t base, uint8_t arg) {
  char buf[8 * sizeof(long) + 1];
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';
  if (base < 2) base = 10;
  if(base != 8 || n>=16) arg = 0;

  do {
    char c = n % base;
    n /= base;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  if(arg == 1)*--str = '0';

  uartPrint(str);
}

void uartInit(void) {
  LL_AHB1_GRP1_EnableClock(USART_TX_GPIO_CLK);
  #if (defined(TYPHOON32V2))
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  #else
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  #endif

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_USART_SetTransferDirection(USART, LL_USART_DIRECTION_RX);

  // Configure USART Tx GPIO
  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Alternate = USART_TX_AF;
  GPIO_InitStructure.Pin = USART_TX_PIN;
  LL_GPIO_Init(USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // USARTx Configuration 115200, 8, 1, n, no flow control,rx-tx enabled
  LL_USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.BaudRate = 115200;
  USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
  USART_InitStructure.Parity = LL_USART_PARITY_NONE;
  USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART, &USART_InitStructure);

  serialPort.txHead = serialPort.txTail = 0;

  // DMA Configuration
  LL_DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;

  // DMA channel Tx of USART Configuration
  LL_DMA_DeInit(DMA1, USART_TX_DMA_CHANNEL);
  DMA_InitStructure.PeriphOrM2MSrcAddress = USART_TDR_ADDRESS;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  LL_DMA_Init(DMA1, USART_TX_DMA_CHANNEL, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(DMA1, USART_TX_DMA_CHANNEL);
  LL_DMA_SetDataLength(DMA1, USART_TX_DMA_CHANNEL, 0);
  LL_USART_EnableDMAReq_TX(USART);
  LL_USART_Enable(USART);
}
