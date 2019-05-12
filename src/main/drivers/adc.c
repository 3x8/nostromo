#include "adc.h"

ADC_HandleTypeDef adcHandle;
DMA_HandleTypeDef adcDmaHandle;

uint32_t adcValue[3];
uint32_t adcVoltageRaw, adcCurrentRaw, adcTemperatureRaw;
uint32_t adcVoltage, adcCurrent, adcTemperature;


void adcRead(void){
  //adcVoltageRaw = adcValue[0];
  //adcTemperatureRaw = adcValue[0];
  adcCurrentRaw = adcValue[0];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
  adcRead();
}
