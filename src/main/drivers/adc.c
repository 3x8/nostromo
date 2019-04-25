#include "adc.h"

ADC_HandleTypeDef adcHandle;
DMA_HandleTypeDef adcDmaHandle;

uint32_t adcVoltageRaw;
uint32_t adcCurrentRaw;
uint32_t adcValue[2];

void adcInit(void) {
  while (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcValue, 2) != HAL_OK);
}

void adcRead(void){
  adcVoltageRaw = adcValue[0];
  adcCurrentRaw = adcValue[1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
  adcRead();
}
