#pragma once

#include "main.h"

#if (!defined(LED_INVERTED))
  #define LED_ON(X)       LL_GPIO_SetOutputPin(X ## _GPIO, X ## _PIN)
  #define LED_OFF(X)      LL_GPIO_ResetOutputPin(X ## _GPIO, X ## _PIN)
#else
  #define LED_ON(X)       LL_GPIO_ResetOutputPin(X ## _GPIO, X ## _PIN)
  #define LED_OFF(X)      LL_GPIO_SetOutputPin(X ## _GPIO, X ## _PIN)
#endif

#define LED_TOGGLE(X)   LL_GPIO_TogglePin(X ## _GPIO, X ## _PIN)


void ledInit(void);
void ledOff(void);
