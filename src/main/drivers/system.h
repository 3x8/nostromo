#pragma once

#include "main.h"

#include "stm32f0xx_ll_adc.h"

void systemClockConfig(void);
void systemDmaInit(void);
void systemAdcInit(void);
void systemComparator1Init(void);
void systemTimer1Init(void);
void systemTimer3Init(void);
void systemTimer15Init(void);
