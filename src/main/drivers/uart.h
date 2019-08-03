#pragma once

#include "main.h"

#define SERIAL_TX_BUFSIZE   256
#define SERIAL_RX_BUFSIZE   256


typedef struct {
  volatile char txBuf[SERIAL_TX_BUFSIZE];
  uint16_t  txHead, txTail;

  volatile char rxBuf[SERIAL_RX_BUFSIZE];
  volatile uint16_t rxHead, rxTail;

  unsigned int rxPos;
  bool InitDone;
} uart_t;


extern uart_t serialPort;


void uartInit(void);
void uartWrite(char ch);
void uartPrint(const char  *str);
void uartPrintInteger(uint32_t n, uint8_t base, uint8_t arg);
bool uartAvailable(void);
char uartRead(void);

/*
void uartOn (void);
void uartOff (void);*/
