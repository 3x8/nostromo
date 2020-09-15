#pragma once

#include "main.h"

typedef struct {
  int32_t voltage;      //0.1 V resolution
  int32_t current;      //0.01 A resolution
  int32_t currentFast;
  int32_t temperature;  //1 °C resolution
} adcDataStructure;

extern ADC_HandleTypeDef adcHandle;
extern DMA_HandleTypeDef adcDmaHandle;
extern uint32_t adcDmaBuffer[3];
extern adcDataStructure adcRaw, adcScaled;
float consumptionMah;

void adcRead(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle);
