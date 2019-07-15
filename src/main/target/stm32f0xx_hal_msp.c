#include "stm32f0xx_hal.h"
#include "target.h"

extern DMA_HandleTypeDef adcDmaHandle;
extern TIM_HandleTypeDef inputTimerHandle;
extern DMA_HandleTypeDef inputTimerDmaHandle;
extern TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle;

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if(adcHandle->Instance == ADC1) {
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

  if(adcHandle->Instance == ADC1) {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, ADC_MASK);
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
}

void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if(hcomp->Instance == COMP1) {
    GPIO_InitStruct.Pin = COMPARATOR_MASK;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp) {

  if(hcomp->Instance == COMP1) {
    HAL_GPIO_DeInit(GPIOA, COMPARATOR_MASK);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if(htim_base->Instance == motorPwmTimerHandle.Instance) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    //HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
  else if(htim_base->Instance == motorCommutationTimerHandle.Instance) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if(htim_base->Instance == inputTimerHandle.Instance) {
    __HAL_RCC_TIM15_CLK_ENABLE();
    // TIM15 -> PA2 GPIO
    GPIO_InitStruct.Pin = INPUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM15;
    HAL_GPIO_Init(INPUT_GPIO, &GPIO_InitStruct);

    // TIM15 DMA Init, TIM15_CH1_UP_TRIG_COM Init
    inputTimerDmaHandle.Instance = DMA1_Channel5;
    inputTimerDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    inputTimerDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    inputTimerDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    inputTimerDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    inputTimerDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    inputTimerDmaHandle.Init.Mode = DMA_NORMAL;
    inputTimerDmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    while (HAL_DMA_Init(&inputTimerDmaHandle) != HAL_OK);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
       Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],inputTimerDmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_UPDATE],inputTimerDmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_TRIGGER],inputTimerDmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_COMMUTATION],inputTimerDmaHandle);
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if(htim->Instance == motorPwmTimerHandle.Instance) {
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;

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
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {

  if(htim_base->Instance == motorPwmTimerHandle.Instance) {
    __HAL_RCC_TIM1_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  }
  else if(htim_base->Instance == motorCommutationTimerHandle.Instance) {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(htim_base->Instance == inputTimerHandle.Instance) {
    __HAL_RCC_TIM15_CLK_DISABLE();

    // TIM15 -> PA2 GPIO
    HAL_GPIO_DeInit(INPUT_GPIO, INPUT_PIN);

    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_TRIGGER]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_COMMUTATION]);
  }
}
