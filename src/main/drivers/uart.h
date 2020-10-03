#pragma once

#include "main.h"

#if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
  #define SERIAL_TX_BUFSIZE 64
  #define SERIAL_RX_BUFSIZE 64
#else
  #define SERIAL_TX_BUFSIZE 313
  #define SERIAL_RX_BUFSIZE 313
#endif

typedef struct {
  volatile char txBuf[SERIAL_TX_BUFSIZE];
  uint16_t  txHead, txTail;
  bool InitDone;
} uartStructure;

extern uartStructure serialPort;

void uartStartTxDMA(void);
void uartInit(void);
void uartWrite(char ch);
void uartPrint(const char  *str);
void uartPrintInteger(uint32_t n, uint8_t base, uint8_t arg);
