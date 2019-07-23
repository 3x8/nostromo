#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_adc.h"


typedef struct {
    int32_t voltage;  //0.1 V resolution
    int32_t current;  //0.1 A resolution
    int32_t temperature;  //1 °C resolution
} adcData_t;


extern ADC_HandleTypeDef adcHandle;
extern DMA_HandleTypeDef adcDmaHandle;
extern uint32_t adcDmaBuffer[3];
extern adcData_t adcRaw, adcScaled, adcFiltered;


void adcRead(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
