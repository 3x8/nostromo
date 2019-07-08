#include "motor.h"

COMP_HandleTypeDef comparator1Handle;

bool motorBemfRising;
bool motorStartup, motorRunning;
bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;

uint16_t motorStep = 1;
uint32_t motorZeroCrossTimestamp;
uint32_t motorFilterLevel, motorFilterDelay;
uint32_t motorDutyCycle, motorBemfCounter;
uint32_t motorZeroCounterTimeout, motorZeroCounterTimeoutThreshold;

// main
extern uint32_t outputPwm;
extern uint32_t motorCommutationDelay;

void motorPhaseA(uint8_t phaseBuffer) {
  switch (phaseBuffer) {
    case HBRIDGE_PWM:
      if (!motorSlowDecay || motorBrakeActiveProportional) {
        LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
      } else {
        LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
      }
      LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
      LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
      LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
      break;
  }
}

void motorPhaseB(uint8_t phaseBuffer) {
  switch (phaseBuffer) {
    case HBRIDGE_PWM:
      if(!motorSlowDecay  || motorBrakeActiveProportional) {
        LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
      } else {
        LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
      }
      LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
      LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
      LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
      break;
  }
}

void motorPhaseC(uint8_t phaseBuffer) {
  switch (phaseBuffer) {
    case HBRIDGE_PWM:
      if (!motorSlowDecay || motorBrakeActiveProportional) {
        LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
      } else {
        LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
      }
      LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
      LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
      LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
      break;
  }
}

void motorCommutationStep(uint8_t stepBuffer) {
  switch(stepBuffer) {
    case 1:
      //A-B
      motorPhaseA(HBRIDGE_LOWSIDE);
      motorPhaseB(HBRIDGE_PWM);
      motorPhaseC(HBRIDGE_FLOATING);
      break;
    case 2:
      // A-C
      motorPhaseA(HBRIDGE_LOWSIDE);
      motorPhaseB(HBRIDGE_FLOATING);
      motorPhaseC(HBRIDGE_PWM);
      break;
    case 3:
      // B-C
      motorPhaseA(HBRIDGE_FLOATING);
      motorPhaseB(HBRIDGE_LOWSIDE);
      motorPhaseC(HBRIDGE_PWM);
      break;
    case 4:
      // B-A
      motorPhaseA(HBRIDGE_PWM);
      motorPhaseB(HBRIDGE_LOWSIDE);
      motorPhaseC(HBRIDGE_FLOATING);
      break;
    case 5:
      // C-A
      motorPhaseA(HBRIDGE_PWM);
      motorPhaseB(HBRIDGE_FLOATING);
      motorPhaseC(HBRIDGE_LOWSIDE);
      break;
    case 6:
      // C-B
      motorPhaseA(HBRIDGE_FLOATING);
      motorPhaseB(HBRIDGE_PWM);
      motorPhaseC(HBRIDGE_LOWSIDE);
      break;
  }
}

void motorBrakeOff() {
  motorPhaseA(HBRIDGE_FLOATING);
  motorPhaseB(HBRIDGE_FLOATING);
  motorPhaseC(HBRIDGE_FLOATING);
}

void motorBrakeFull() {
  motorPhaseA(HBRIDGE_LOWSIDE);
  motorPhaseB(HBRIDGE_LOWSIDE);
  motorPhaseC(HBRIDGE_LOWSIDE);
}

// dutyCycle controls braking strength
void motorBrakeProportional() {
  motorPhaseA(HBRIDGE_PWM);
  motorPhaseB(HBRIDGE_PWM);
  motorPhaseC(HBRIDGE_PWM);
}

void motorChangeComparatorInput() {
  switch(motorStep) {
    case 1:
    case 4:
      // C floating
      comparator1Handle.Init.InvertingInput = COMPARATOR_PHASE_C;
      break;
    case 2:
    case 5:
      // B floating
      comparator1Handle.Init.InvertingInput = COMPARATOR_PHASE_B;
      break;
    case 3:
    case 6:
      // A floating
      comparator1Handle.Init.InvertingInput = COMPARATOR_PHASE_A;
      break;
  }

  // polarity of comp output reversed
  if (motorBemfRising) {
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  } else {
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  HAL_COMP_Init(&comparator1Handle);
}

void motorCommutate() {
  if (motorDirection == SPIN_CW) {
    if (++motorStep > 6) {
      motorStep = 1;
    }

    if ((motorStep == 1) || (motorStep == 3) || (motorStep == 5)) {
      motorBemfRising = true;
    } else {
      motorBemfRising = false;
    }
  } else {
    if (--motorStep < 1) {
      motorStep = 6;
    }

    if ((motorStep == 1) || (motorStep == 3) || (motorStep == 5)) {
      motorBemfRising = false;
    } else {
      motorBemfRising = true;
    }
  }

  motorCommutationStep(motorStep);
  motorChangeComparatorInput();
}

// for forced commutation -- open loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  //noop
}

void motorStart() {
  bool bufferDecaystate = motorSlowDecay;

  if (!motorRunning) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    motorSlowDecay = true;

    motorCommutate();

    TIM3->CNT = 0xffff;
    motorBemfCounter = 0;
    motorRunning = true;
    HAL_COMP_Start_IT(&comparator1Handle);
  }
  motorSlowDecay = bufferDecaystate;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  uint32_t motorTimestamp;

  __disable_irq();
  motorTimestamp = TIM3->CNT;

  if ((!motorRunning) || (!motorStartup)) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    __enable_irq();
    return;
  }

  while ((TIM3->CNT - motorTimestamp) < motorFilterDelay);

  for (int i = 0; i < motorFilterLevel; i++) {
    if ((motorBemfRising && HAL_COMP_GetOutputLevel(&comparator1Handle) == COMP_OUTPUTLEVEL_HIGH) ||
        (!motorBemfRising && HAL_COMP_GetOutputLevel(&comparator1Handle) == COMP_OUTPUTLEVEL_LOW)) {

      __enable_irq();
      return;
    }
  }

  #if (defined(_DEBUG_) && defined(MOTOR_TIMING))
    LED_ON(GREEN);
  #endif

  HAL_COMP_Stop_IT(&comparator1Handle);
  TIM3->CNT = 0xffff;

  motorBemfCounter++;
  motorZeroCounterTimeout = 0;
  motorZeroCrossTimestamp = motorTimestamp;

  // ToDo
  //if ((motorCommutationDelay != 0)) {
  if ((outputPwm > 37) && (outputPwm < 807) && (motorCommutationDelay != 0) && (motorCommutationDelay < 613)) {
    while (TIM3->CNT < motorCommutationDelay) {
      #if (defined(_DEBUG_) && defined(MOTOR_TIMING))
        LED_TOGGLE(BLUE);
      #endif
    }
  }

  #if (defined(_DEBUG_) && defined(MOTOR_TIMING))
    LED_OFF(GREEN);
    LED_OFF(BLUE);
  #endif

  motorCommutate();

  HAL_COMP_Start_IT(&comparator1Handle);
  __enable_irq();
}

void motorStartupTune() {
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;

  motorCommutationStep(motorStep);
  TIM1->PSC = 100;
  HAL_Delay(100);
  TIM1->PSC = 75;
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);

  TIM1->PSC = 0;
}

void motorInputTune(uint8_t motorStepDebug) {
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;

  motorCommutationStep(motorStepDebug);
  TIM1->PSC = 75;
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);

  TIM1->PSC = 0;
}
