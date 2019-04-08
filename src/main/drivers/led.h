#pragma once

#include "main.h"

#define LED_ON(X)       LL_GPIO_SetOutputPin(X ## _GPIO, X ## _PIN)
#define LED_OFF(X)      LL_GPIO_ResetOutputPin(X ## _GPIO, X ## _PIN)
#define LED_TOGGLE(X)   LL_GPIO_TogglePin(X ## _GPIO, X ## _PIN)

void ledInit(void);
