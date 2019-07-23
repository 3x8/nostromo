#include "adc.h"

ADC_HandleTypeDef adcHandle;
DMA_HandleTypeDef adcDmaHandle;

uint32_t adcDmaBuffer[3];
adcData_t adcRaw, adcScaled, adcFiltered;

void adcRead(void){
  adcRaw.voltage = adcDmaBuffer[0];
  adcRaw.current = adcDmaBuffer[1];
  adcRaw.temperature = adcDmaBuffer[2];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
  adcRead();
}
