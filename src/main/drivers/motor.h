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

typedef enum {
  SPIN_CW = 0,
  SPIN_CCW
} motorDirectionEnum;

void motorStartupTune();
void motorInputTune();

void motorPhaseA(uint8_t phaseBuffer);
void motorPhaseB(uint8_t phaseBuffer);
void motorPhaseC(uint8_t phaseBuffer);

void motorStart();
void motorCommutate();
void motorCommutationStep(uint8_t stepBuffer);
void motorChangeComparatorInput();

void motorBrakeOff();
void motorBrakeFull();
void motorBrakeProportional();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
