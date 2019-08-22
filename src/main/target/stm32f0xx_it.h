#pragma once

#include "main.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_3_IRQHandler(void);
void DMA1_Channel4_5_IRQHandler(void);
void ADC1_COMP_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
