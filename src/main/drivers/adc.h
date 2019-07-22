#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_adc.h"

typedef struct {
    uint32_t currentRaw;
    uint32_t voltageRaw;
    uint32_t temperatureRaw;
} adcData_t;

void adcRead(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
