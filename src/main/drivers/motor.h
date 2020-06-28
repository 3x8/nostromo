#pragma once

#include "main.h"

#define MOTOR_POLES 14 // ToDo move to eeprom
#define BLDC_STEPS  6

typedef enum {
  HBRIDGE_PWM = 0,
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
  bool ComplementaryPWM;
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
  float RpmFactor;
} motorStructure;

extern TIM_HandleTypeDef motorPwmTimerHandle;
extern COMP_HandleTypeDef motorBemfComparatorHandle;
extern TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, inputTimerHandle;
extern motorStructure motor;

void motorPhaseA(uint8_t phaseBuffer);
void motorPhaseB(uint8_t phaseBuffer);
void motorPhaseC(uint8_t phaseBuffer);

void motorCommutationStep(uint8_t stepBuffer);
void motorComparatorInputChange();
void motorCommutate();

void motorStart();

void motorBrakeOff();
void motorBrakeFull();
void motorBrakeProportional();

void motorTuneStartup();
void motorTuneInput(uint8_t motorStepDebug);

void motorInputUpdate(void);
uint32_t motorGetRpm(void);
