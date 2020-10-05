#pragma once

#include "main.h"

#define MOTOR_POLES 14
#define MOTOR_BLDC_STEPS 6
#define MOTOR_BLDC_MEDIAN (MOTOR_BLDC_STEPS * 8)
#define MOTOR_ONE_ROTATION  (MOTOR_POLES * MOTOR_BLDC_STEPS)
#define MOTOR_START_THRESHOLD 27
#define MOTOR_START_POWER 73
#define MOTOR_PWM_RESOLUTION  1003
#define MOTOR_AUTOTIMING_DELAY_MIN  0
#define MOTOR_AUTOTIMING_DELAY_MAX  77
#define MOTOR_ERPM_FACTOR 360000000
//MOTOR_ERPM_FACTOR = (60000000 / ((motorCommutationTimerHandle.Init.Prescaler + 1) / (HAL_RCC_GetSysClockFreq() * 0.000001)));

typedef enum {
  HBRIDGE_PWM = 0,
  HBRIDGE_FLOATING,
  HBRIDGE_LOWSIDE
} motorHbridgeStateEnum;

typedef enum {
  BRAKE_OFF = 0,
  BRAKE_FULL
} motorBrakeStateEnum;

typedef enum {
  SPIN_CCW = 0,
  SPIN_CW
} motorDirectionEnum;

typedef struct {
  bool Start;
  bool Running;
  bool Direction;
  bool ComplementaryPWM;
  bool BemfRising;
  uint8_t Step;
  uint32_t BemfCounter;
  uint32_t BemfZeroCrossTimestamp;
  uint8_t BemfFilterLevel;
  uint8_t BemfFilterDelay;
  uint32_t BemfZeroCounterTimeout;
  uint32_t BemfZeroCounterTimeoutThreshold;
  uint32_t OneErpmTime;
  uint32_t oneDegree;
  uint32_t CommutationDelay;
  uint32_t CommutationTime;
  uint32_t Debug;
} motorStructure;

extern TIM_HandleTypeDef motorPwmTimerHandle, motorAutotimingTimerHandle;
extern COMP_HandleTypeDef motorBemfComparatorHandle;
extern TIM_HandleTypeDef motorCommutationTimerHandle, inputTimerHandle;
extern motorStructure motor;

void motorPhaseA(uint8_t phaseBuffer);
void motorPhaseB(uint8_t phaseBuffer);
void motorPhaseC(uint8_t phaseBuffer);
void motorCommutationStep(uint8_t stepBuffer);
void motorComparatorInputChange();
void motorCommutate();
void motorBemfZeroCrossCallback(void);
void motorComutateAutotimingCallback(void);

void motorStart();
void motorBrakeOff();
void motorBrakeFull();
void motorTuneReady();
void motorTuneInput(uint8_t motorStepDebug);
void motorInputUpdate(void);
uint32_t motorGetErpm(void);
uint32_t motorGetRpm(void);
