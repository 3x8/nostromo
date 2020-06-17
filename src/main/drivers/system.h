#pragma once

#include "main.h"

extern TIM_HandleTypeDef msTimerHandle;

void systemClockConfig(void);
void systemDmaInit(void);
void systemAdcInit(void);
#if !defined(STSPIN32F0)
  void systemBemfComparatorInit(void);
#endif
void systemMotorPwmTimerInit(void);
void systemMotorCommutationTimerInit(void);
void systemInputTimerInit(void);
void systemMsTimerInit(void);

#if (defined(USE_BOOTLOADER))
  #define APPLICATION_ADDRESS (uint32_t)0x08002000
  void systemInitAfterBootloaderJump();
#endif

#if defined(STSPIN32F0)
  void systemGpioInit(void);
#endif
