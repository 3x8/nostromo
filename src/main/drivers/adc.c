#include "adc.h"

uint32_t adcVoltageRaw;
uint32_t adcCurrentRaw;
uint32_t adcValue[2];
extern ADC_HandleTypeDef hadc;


void adcInit(void) {
  while (HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcValue, 2) != HAL_OK);
}

void adcRead(void){
  adcVoltageRaw = adcValue[0];
  adcCurrentRaw = adcValue[1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  adcRead();
}
