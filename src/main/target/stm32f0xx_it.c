#include "stm32f0xx_it.h"


void NMI_Handler(void) {
}

void HardFault_Handler(void) {
  while (true);
}

void SVC_Handler(void) {
}

void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&adcDmaHandle);
}

void DMA1_Channel2_3_IRQHandler(void) {
  if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(DMA1, USART_TX_DMA_CHANNEL);

    if (serialPort.txHead != serialPort.txTail) {
      uartStartTxDMA();
    }
  } else {
    // ToDo
    HAL_DMA_IRQHandler(&inputTimerDmaHandle);
    inputCallbackDMA();
  }
}

void DMA1_Channel4_5_IRQHandler(void) {
  #if (defined(TYPHOON32V2))
    if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
      LL_DMA_ClearFlag_TC4(DMA1);
      LL_DMA_DisableChannel(DMA1, USART_TX_DMA_CHANNEL);
      if (serialPort.txHead != serialPort.txTail) {
        uartStartTxDMA();
      }
    } else {
      HAL_DMA_IRQHandler(&inputTimerDmaHandle);
      inputCallbackDMA();
    }
  #else
    HAL_DMA_IRQHandler(&inputTimerDmaHandle);
    inputCallbackDMA();
  #endif
}

void ADC1_COMP_IRQHandler(void) {
  HAL_ADC_IRQHandler(&adcHandle);
  HAL_COMP_IRQHandler(&motorBemfComparatorHandle);
}

void TIM1_CC_IRQHandler(void) {
  HAL_TIM_IRQHandler(&motorPwmTimerHandle);
}
