#include "motor.h"

COMP_HandleTypeDef comparator1Handle;

uint8_t motorBemfRising;
uint8_t motorStartup, motorRunning, motorSensorless;
uint8_t motorDirection, motorSlowDecay, motorBrakeActiveProportional = true;

uint16_t motorStep, motorAdvanceDivisor;
uint32_t motorAdvance, motorBlanktime, motorWaitTime;
uint32_t motorZeroCrossTimestamp, motorCommutationInterval;
uint32_t motorFilterLevel, motorFilterDelay;
uint32_t motorDutyCycle, motorBemfCounter;
uint32_t motorZeroCounterTimeout, motorZeroCounterTimeoutThreshold;

extern uint32_t outputPwm;


void motorAdvanceDivisorCalculate() {
    motorAdvanceDivisor = map((motorCommutationInterval),100,5000, 2, 20);
}

// motorPhaseB qfn , motorPhaseA qfp
void motorPhaseA(uint8_t phaseBuffer) {
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

// motorPhaseC qfn , motorPhaseB qfp
void motorPhaseB(uint8_t phaseBuffer) {
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

// motorPhaseA qfn , motorPhaseC qfp
void motorPhaseC(uint8_t phaseBuffer) {
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

// dutyCycle controls braking strength, will turn off lower fets so only high side is active
void motorBrakeProportional() {
  motorPhaseA(HBRIDGE_PWM);
  motorPhaseB(HBRIDGE_PWM);
  motorPhaseC(HBRIDGE_PWM);
}

void motorChangeCompInput() {
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

  while (HAL_COMP_Init(&comparator1Handle) != HAL_OK);
}

void motorCommutate() {
  if (motorDirection == 1) {
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

  if (outputPwm > 0) {
    motorCommutationStep(motorStep);
  }
  motorChangeCompInput();
}

// for forced commutation -- open loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
  }
}

void motorStart() {
  bool bufferDecaystate = motorSlowDecay;
  motorSensorless = false;

  if (!motorRunning) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    motorSlowDecay = true;

    motorCommutate();

    TIM3->CNT = 0;
    motorRunning = true;
    while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
  }

  motorSlowDecay = bufferDecaystate;
  motorSensorless = true;
  motorBemfCounter = 0;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  uint32_t motorTimestamp;

  //ToDo new
  if ((!motorRunning) || (!motorSensorless) || (!motorStartup)) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    //debug
    LED_OFF(GREEN);
    LED_OFF(BLUE);
    return;
  }

  motorTimestamp = TIM3->CNT;
  //debug
  LED_TOGGLE(GREEN);

  while ((TIM3->CNT - motorTimestamp) < motorFilterDelay);

  if (motorBemfRising) {
    for (int i = 0; i < motorFilterLevel; i++) {
      if (HAL_COMP_GetOutputLevel(&comparator1Handle) == COMP_OUTPUTLEVEL_HIGH) {
        return;
      }
    }
  } else {
    for (int i = 0; i < motorFilterLevel; i++) {
      if (HAL_COMP_GetOutputLevel(&comparator1Handle) == COMP_OUTPUTLEVEL_LOW) {
        return;
      }
    }
  }
  //debug
  LED_TOGGLE(BLUE);

  motorZeroCounterTimeout = 0;
  motorZeroCrossTimestamp = motorTimestamp;
  TIM3->CNT = 0;
  HAL_COMP_Stop_IT(&comparator1Handle);

  //test   divide by two when tracking up down time independant
  motorCommutationInterval = (motorCommutationInterval + motorZeroCrossTimestamp) >> 1;
  motorAdvance = motorCommutationInterval / motorAdvanceDivisor;
  motorWaitTime = (motorCommutationInterval >> 1) - motorAdvance;
  motorBlanktime = motorCommutationInterval >> 2;

  if (motorSensorless) {
    motorCommutate();
  }

  motorBemfCounter++;
  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
}

void motorStartupTune() {
  TIM1->PSC = 75;
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;
  motorCommutationStep(2);
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);
  TIM1->PSC = 25;
  HAL_Delay(100);
  motorBrakeOff();
  TIM1->PSC = 0;
}

void motorInputTune() {
  TIM1->PSC = 100;
  TIM1->CCR1 = 5;
  TIM1->CCR2 = 5;
  TIM1->CCR3 = 5;
  motorCommutationStep(2);
  HAL_Delay(100);
  TIM1->PSC = 50;
  HAL_Delay(100);
  motorBrakeOff();
  TIM1->PSC = 0;
}
