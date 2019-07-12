#include "stm32f0xx_hal.h"
#include "target.h"

extern DMA_HandleTypeDef adcDmaHandle;
extern DMA_HandleTypeDef timer15Channel1DmaHandle;

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

  if(htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    //HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  }
  else if(htim_base->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if(htim_base->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_ENABLE();
    /**TIM15 GPIO Configuration
       PA2     ------> TIM15_CH1
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_TIM15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM15 DMA Init */
    /* TIM15_CH1_UP_TRIG_COM Init */
    timer15Channel1DmaHandle.Instance = DMA1_Channel5;
    timer15Channel1DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    timer15Channel1DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    timer15Channel1DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    timer15Channel1DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    timer15Channel1DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    timer15Channel1DmaHandle.Init.Mode = DMA_NORMAL;
    timer15Channel1DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    while (HAL_DMA_Init(&timer15Channel1DmaHandle) != HAL_OK);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
       Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],timer15Channel1DmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_UPDATE],timer15Channel1DmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_TRIGGER],timer15Channel1DmaHandle);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_COMMUTATION],timer15Channel1DmaHandle);
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct;

  if(htim->Instance == TIM1) {
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

  if(htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();

    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  }
  else if(htim_base->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(htim_base->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_DISABLE();

    /**TIM15 GPIO Configuration
       PA2     ------> TIM15_CH1
     */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);

    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_UPDATE]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_TRIGGER]);
    HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_COMMUTATION]);
  }
}
