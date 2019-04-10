#include "watchdog.h"

IWDG_HandleTypeDef watchdog;

void watchdogInit(uint32_t timeout) {
  watchdog.Instance = IWDG;
  watchdog.Init.Prescaler = IWDG_PRESCALER_16;
  watchdog.Init.Window = IWDG_WINDOW_DISABLE;
  watchdog.Init.Reload = timeout;
  while (HAL_IWDG_Init(&watchdog) != HAL_OK);
}

void watchdogFeed(void) {
  while (HAL_IWDG_Refresh(&watchdog) != HAL_OK);
}
