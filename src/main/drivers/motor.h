#pragma once

#include "main.h"

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
  #if defined(STSPIN32F0)
    uint8_t ExtiPin;
  #endif

} motor_t;

extern TIM_HandleTypeDef motorPwmTimerHandle;
#if (defined(FD6288) || defined(NCP3420))
  extern COMP_HandleTypeDef motorBemfComparatorHandle;
#endif
extern TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, inputTimerHandle;
extern motor_t motor;

void motorPhaseA(uint8_t phaseBuffer);
void motorPhaseB(uint8_t phaseBuffer);
void motorPhaseC(uint8_t phaseBuffer);

void motorCommutationStep(uint8_t stepBuffer);
#if (defined(FD6288) || defined(NCP3420))
  void motorComparatorInputChange();
#endif
#if defined(STSPIN32F0)
  void motorExtiInputChange();
  void motorExtiMaskInterrupts();
  void motorExtiCallback();
#endif
void motorCommutate();

void motorStart();

void motorBrakeOff();
void motorBrakeFull();
void motorBrakeProportional();

void motorTuneStartup();
void motorTuneInput(uint8_t motorStepDebug);

void motorInputUpdate(void);
