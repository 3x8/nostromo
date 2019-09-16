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
  uint8_t Step;
  uint32_t BemfCounter;
  uint32_t BemfZeroCrossTimestamp;
  uint8_t BemfFilterLevel;
  uint8_t BemfFilterDelay;
  uint32_t BemfZeroCounterTimeout;
  uint32_t BemfZeroCounterTimeoutThreshold;
  uint32_t CommutationInterval;
  uint32_t CommutationDelay;
} motor_t;

extern TIM_HandleTypeDef motorPwmTimerHandle;
extern COMP_HandleTypeDef motorBemfComparatorHandle;
extern TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, inputTimerHandle;
extern motor_t motor;

void motorTuneStartup();
void motorTuneInput(uint8_t motorStepDebug);
void motorPhaseA(uint8_t phaseBuffer);
void motorPhaseB(uint8_t phaseBuffer);
void motorPhaseC(uint8_t phaseBuffer);
void motorStart();
void motorCommutate();
void motorCommutationStep(uint8_t stepBuffer);
void motorComparatorInputChange();
void motorBrakeOff();
void motorBrakeFull();
void motorBrakeProportional();
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *comparatorHandle);
