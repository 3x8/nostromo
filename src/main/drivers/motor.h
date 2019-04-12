#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_comp.h"

typedef enum {
  pwm = 1,
  floating,
  lowside
} motorHbridgeStateEnum;

void advanceDivisor();

void phaseA(uint8_t newPhase);
void phaseB(uint8_t newPhase);
void phaseC(uint8_t newPhase);
void commutationStep(uint8_t newStep);
void allOff();
void fullBrake();
void proBrake();
void changeCompInput();
void commutate();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void startMotor();
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
void changeDutyCycleWithSin();
void zc_found_routine();
void playStartupTune();
void playInputTune();
