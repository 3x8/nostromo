#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_adc.h"

void adcInit(void);
void adcRead(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
