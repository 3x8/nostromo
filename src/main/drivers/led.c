#include "led.h"

void ledInit(void) {
  LL_GPIO_InitTypeDef gpioInit;

  LL_AHB1_GRP1_EnableClock(LED_MASK);

  LL_GPIO_ResetOutputPin(LED_RED_GPIO, LED_RED_PIN);
  LL_GPIO_ResetOutputPin(LED_GREEN_GPIO, LED_GREEN_PIN);
  LL_GPIO_ResetOutputPin(LED_BLUE_GPIO, LED_BLUE_PIN);

  gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
  gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpioInit.Pull = LL_GPIO_PULL_NO;
  gpioInit.Alternate = LL_GPIO_AF_0;

  gpioInit.Pin = LED_RED_PIN;
  LL_GPIO_Init(LED_RED_GPIO, &gpioInit);

  gpioInit.Pin = LED_GREEN_PIN;
  LL_GPIO_Init(LED_GREEN_GPIO, &gpioInit);

  gpioInit.Pin = LED_BLUE_PIN;
  LL_GPIO_Init(LED_BLUE_GPIO, &gpioInit);
}

void ledOff(void) {
  LED_OFF(LED_RED);
  LED_OFF(LED_GREEN);
  LED_OFF(LED_BLUE);
}
