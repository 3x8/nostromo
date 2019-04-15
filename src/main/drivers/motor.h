#pragma once

#include "main.h"

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_comp.h"

typedef enum {
  HBRIDGE_PWM = 1,
  HBRIDGE_FLOATING,
  HBRIDGE_LOWSIDE
} motorHbridgeStateEnum;

typedef enum {
  BRAKE_OFF = 0,
  BRAKE_PROPORTIONAL,
  BRAKE_FULL
} motorBrakeStateEnum;

void advanceDivisor();

void motorPhaseA(uint8_t newPhase);
void motorPhaseB(uint8_t newPhase);
void motorPhaseC(uint8_t newPhase);
void motorCommutationStep(uint8_t newStep);
void motorBrakeOff();
void motorBrakeFull();
void motorBrakeProportional();
void motorChangeCompInput();
void motorCommutate();
void motorStart();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
void motorChangeDutyCycleWithSin();
void zc_found_routine();
void motorStartupTune();
void motorInputTune();