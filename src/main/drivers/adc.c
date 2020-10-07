#include "adc.h"

ADC_HandleTypeDef adcHandle;
DMA_HandleTypeDef adcDmaHandle;

uint32_t adcDmaBuffer[3];
adcDataStructure adcRaw, adcScaled;
float consumptionMah;

void adcRead(void) {
  #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2) || defined(FURLING45MINI) || defined(SUCCEX50AV2) || defined(RAZOR32V2))
    adcRaw.voltage = adcDmaBuffer[0];
    adcRaw.current = adcDmaBuffer[1];
    adcRaw.temperature = adcDmaBuffer[2];
  #endif

  #if (defined(WRAITH32MINI) || defined(SUCCEXMINI40A))
    adcRaw.temperature = adcDmaBuffer[0];
  #endif

  #if (defined(DYS35ARIA))
    adcRaw.current = adcDmaBuffer[0];
    adcRaw.temperature = adcDmaBuffer[1];
  #endif

  #if (defined(KISS24A))
    adcRaw.voltage = adcDmaBuffer[1];
    adcRaw.current = adcDmaBuffer[0];
    adcRaw.temperature = adcDmaBuffer[2];
  #endif
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle) {
  adcRead();
}
