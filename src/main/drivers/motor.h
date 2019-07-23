#pragma once

#include "main.h"


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
  SPIN_CCW = 0,
  SPIN_CW
} motorDirectionEnum;

typedef struct {
  bool Startup;
  bool Running;
  bool Direction;
  bool SlowDecay;
  bool BrakeActiveProportional;
  bool BemfRising;
  uint32_t BemfCounter;
  uint32_t BemfZeroCrossTimestamp;
  uint32_t BemfFilterLevel;
  uint32_t BemfFilterDelay;
  uint32_t BemfZeroCounterTimeout;
  uint32_t BemfZeroCounterTimeoutThreshold;
  uint32_t CommutationInterval, motorCommutationDelay;
  uint32_t CommutationDelay;
} motor_t;


extern TIM_HandleTypeDef motorPwmTimerHandle;
extern COMP_HandleTypeDef motorBemfComparatorHandle;
extern TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, inputTimerHandle;

extern motor_t motor;

//extern bool motorStartup, motorRunning;
extern bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;
extern uint32_t  motorBemfCounter, motorBemfZeroCrossTimestamp;
extern uint32_t motorBemfFilterLevel, motorBemfFilterDelay;
extern uint32_t motorBemfZeroCounterTimeout, motorBemfZeroCounterTimeoutThreshold;
extern uint32_t motorCommutationInterval, motorCommutationDelay;


void motorStartupTune();
void motorInputTune(uint8_t motorStepDebug);

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
