#pragma once

#include "main.h"

#include "stm32f0xx_ll_adc.h"

void systemClockConfig(void);
void systemDmaInit(void);
void systemAdcInit(void);
void systemBemfComparatorInit(void);
void systemMotorPwmTimerInit(void);
void systemMotorCommutationTimerInit(void);
void systemInputTimerInit(void);
