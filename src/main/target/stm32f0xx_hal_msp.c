#include "stm32f0xx_hal.h"
#include "target.h"

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if (adcHandle->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitStruct.Pin = ADC_MASK;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    adcDmaHandle.Instance = DMA1_Channel1;
    adcDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adcDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    adcDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    adcDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    adcDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    adcDmaHandle.Init.Mode = DMA_CIRCULAR;
    adcDmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    while (HAL_DMA_Init(&adcDmaHandle) != HAL_OK);

    __HAL_LINKDMA(adcHandle,DMA_Handle,adcDmaHandle);
    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
  if (adcHandle->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, ADC_MASK);
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
}

#if (defined(FD6288) || defined(NCP3420))
  void HAL_COMP_MspInit(COMP_HandleTypeDef* comparatorHandle) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (comparatorHandle->Instance == COMPARATOR) {
      GPIO_InitStruct.Pin = COMPARATOR_MASK;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
    }
  }

  void HAL_COMP_MspDeInit(COMP_HandleTypeDef* comparatorHandle) {
    if (comparatorHandle->Instance == COMPARATOR) {
      HAL_GPIO_DeInit(GPIOA, COMPARATOR_MASK);
    }
  }
#endif

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* timerHandle) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if (timerHandle->Instance == motorPwmTimerHandle.Instance) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (timerHandle->Instance==msTimerHandle.Instance) {
    __HAL_RCC_TIM16_CLK_ENABLE();
  }
  else if (timerHandle->Instance == motorCommutationTimerHandle.Instance) {
    __HAL_RCC_TIM14_CLK_ENABLE();
  }
  else if (timerHandle->Instance == inputTimerHandle.Instance) {
    if (INPUT_TIMER == TIM2){
      __HAL_RCC_TIM2_CLK_ENABLE();
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    }
    if (INPUT_TIMER == TIM3){
      __HAL_RCC_TIM3_CLK_ENABLE();
      GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    }
    #if !defined(STSPIN32F0)
      if (INPUT_TIMER == TIM15){
        __HAL_RCC_TIM15_CLK_ENABLE();
        GPIO_InitStruct.Alternate = GPIO_AF0_TIM15;
      }
    #endif

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = INPUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(INPUT_GPIO, &GPIO_InitStruct);

    if (INPUT_TIMER == TIM2){
      inputTimerDmaHandle.Instance = DMA1_Channel3;
    }
    if (INPUT_TIMER == TIM3){
      inputTimerDmaHandle.Instance = DMA1_Channel4;
    }
    #if !defined(STSPIN32F0)
      if (INPUT_TIMER == TIM15){
        inputTimerDmaHandle.Instance = DMA1_Channel5;
      }
    #endif
    inputTimerDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    inputTimerDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    inputTimerDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    inputTimerDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    inputTimerDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    inputTimerDmaHandle.Init.Mode = DMA_NORMAL;
    inputTimerDmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    while (HAL_DMA_Init(&inputTimerDmaHandle) != HAL_OK);

    //  there is only one channel to perform all the requested DMAs.
    #if (INPUT_TIMER_CH == TIM_CHANNEL_1)
      __HAL_LINKDMA(timerHandle,hdma[TIM_DMA_ID_CC1],inputTimerDmaHandle);
    #endif
    #if (INPUT_TIMER_CH == TIM_CHANNEL_2)
      __HAL_LINKDMA(timerHandle,hdma[TIM_DMA_ID_CC2],inputTimerDmaHandle);
    #endif
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timerHandle) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if (timerHandle->Instance == motorPwmTimerHandle.Instance) {
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;

    #if (defined(FD6288) || defined(STSPIN32F0))
      GPIO_InitStruct.Pin = A_FET_LO_PIN;
      HAL_GPIO_Init(A_FET_LO_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = A_FET_HI_PIN;
      HAL_GPIO_Init(A_FET_HI_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = B_FET_LO_PIN;
      HAL_GPIO_Init(B_FET_LO_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = B_FET_HI_PIN;
      HAL_GPIO_Init(B_FET_HI_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = C_FET_LO_PIN;
      HAL_GPIO_Init(C_FET_LO_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = C_FET_HI_PIN;
      HAL_GPIO_Init(C_FET_HI_GPIO, &GPIO_InitStruct);
    #endif

    #if defined(NCP3420)
      GPIO_InitStruct.Pin = A_FET_OE_PIN;
      HAL_GPIO_Init(A_FET_OE_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = A_FET_IN_PIN;
      HAL_GPIO_Init(A_FET_IN_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = B_FET_OE_PIN;
      HAL_GPIO_Init(B_FET_OE_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = B_FET_IN_PIN;
      HAL_GPIO_Init(B_FET_IN_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = C_FET_OE_PIN;
      HAL_GPIO_Init(C_FET_OE_GPIO, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = C_FET_IN_PIN;
      HAL_GPIO_Init(C_FET_IN_GPIO, &GPIO_InitStruct);
    #endif
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* timerHandle) {
  if (timerHandle->Instance == motorPwmTimerHandle.Instance) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  }
  else if (timerHandle->Instance==msTimerHandle.Instance) {
    __HAL_RCC_TIM16_CLK_DISABLE();
  }
  else if (timerHandle->Instance == motorCommutationTimerHandle.Instance) {
    __HAL_RCC_TIM14_CLK_DISABLE();
  }
  else if (timerHandle->Instance == inputTimerHandle.Instance) {
    if (INPUT_TIMER == TIM2) {
      __HAL_RCC_TIM2_CLK_DISABLE();
    }
    if (INPUT_TIMER == TIM3) {
      __HAL_RCC_TIM3_CLK_DISABLE();
    }
    #if !defined(STSPIN32F0)
      if (INPUT_TIMER == TIM15) {
        __HAL_RCC_TIM15_CLK_DISABLE();
      }
    #endif

    // input timer
    HAL_GPIO_DeInit(INPUT_GPIO, INPUT_PIN);
    #if (INPUT_TIMER_CH == TIM_CHANNEL_1)
      HAL_DMA_DeInit(timerHandle->hdma[TIM_DMA_ID_CC1]);
    #endif
    #if (INPUT_TIMER_CH == TIM_CHANNEL_2)
      HAL_DMA_DeInit(timerHandle->hdma[TIM_DMA_ID_CC2]);
    #endif
  }
}
