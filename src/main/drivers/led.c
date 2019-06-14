#include "led.h"

void ledInit(void) {
  LL_GPIO_InitTypeDef gpioInit;

  LL_AHB1_GRP1_EnableClock(RED_PERIPHERAL | GREEN_PERIPHERAL | BLUE_PERIPHERAL);

  LL_GPIO_ResetOutputPin(RED_GPIO, RED_PIN);
  LL_GPIO_ResetOutputPin(GREEN_GPIO, GREEN_PIN);
  LL_GPIO_ResetOutputPin(BLUE_GPIO, BLUE_PIN);

  gpioInit.Mode = LL_GPIO_MODE_OUTPUT;
  gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpioInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpioInit.Pull = LL_GPIO_PULL_NO;
  gpioInit.Alternate = LL_GPIO_AF_0;

  gpioInit.Pin = RED_PIN;
  LL_GPIO_Init(RED_GPIO, &gpioInit);

  gpioInit.Pin = GREEN_PIN;
  LL_GPIO_Init(GREEN_GPIO, &gpioInit);

  gpioInit.Pin = BLUE_PIN;
  LL_GPIO_Init(BLUE_GPIO, &gpioInit);
}

void ledOff(void) {
  LED_OFF(RED);
  LED_OFF(GREEN);
  LED_OFF(BLUE);
}
