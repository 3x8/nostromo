#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_adc.h"


typedef struct {
    uint32_t voltage;
    uint32_t current;
    uint32_t temperature;
} adcData_t;


extern ADC_HandleTypeDef adcHandle;
extern DMA_HandleTypeDef adcDmaHandle;
extern uint32_t adcDmaBuffer[3];
extern adcData_t adcRaw, adcFiltered;


void adcRead(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
