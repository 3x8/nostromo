#include "motor.h"

COMP_HandleTypeDef motorBemfComparatorHandle;

bool motorBemfRising;
bool motorStartup, motorRunning;
bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;

uint16_t motorStep = 1;
uint32_t motorBemfZeroCrossTimestamp;
uint32_t motorBemfFilterLevel, motorBemfFilterDelay;
uint32_t motorDutyCycle, motorBemfCounter;
uint32_t motorBemfZeroCounterTimeout, motorBemfZeroCounterTimeoutThreshold;

// main
extern uint32_t outputPwm;
extern uint32_t motorCommutationDelay;

void motorPhaseA(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
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

void motorPhaseB(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
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

void motorPhaseC(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
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
      motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_C;
      break;
    case 2:
    case 5:
      // B floating
      motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_B;
      break;
    case 3:
    case 6:
      // A floating
      motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_A;
      break;
  }

  // polarity of comp output reversed
  if (motorBemfRising) {
    motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  } else {
    motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  HAL_COMP_Init(&motorBemfComparatorHandle);
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

// forced commutation -- open loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  //noop
}

void motorStart() {
  bool bufferDecaystate = motorSlowDecay;

  if (!motorRunning) {
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    motorSlowDecay = true;

    motorCommutate();

    TIM3->CNT = 0xffff;
    motorBemfCounter = 0;
    motorRunning = true;
    HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  }
  motorSlowDecay = bufferDecaystate;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  uint32_t motorTimestamp;

  __disable_irq();
  motorTimestamp = TIM3->CNT;

  if ((!motorRunning) || (!motorStartup)) {
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    __enable_irq();
    return;
  }

  while ((TIM3->CNT - motorTimestamp) < motorBemfFilterDelay);

  for (int i = 0; i < motorBemfFilterLevel; i++) {
    if ((motorBemfRising && HAL_COMP_GetOutputLevel(&motorBemfComparatorHandle) == COMP_OUTPUTLEVEL_HIGH) ||
        (!motorBemfRising && HAL_COMP_GetOutputLevel(&motorBemfComparatorHandle) == COMP_OUTPUTLEVEL_LOW)) {

      __enable_irq();
      return;
    }
  }

  #if (defined(_DEBUG_) && defined(MOTOR_TIMING))
    LED_ON(GREEN);
  #endif

  HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
  TIM3->CNT = 0xffff;

  motorBemfCounter++;
  motorBemfZeroCounterTimeout = 0;
  motorBemfZeroCrossTimestamp = motorTimestamp;

  // ToDo
  //if ((motorCommutationDelay > 31) && (motorCommutationDelay < 613) && (outputPwm > 37) && (outputPwm < 707)) {
  if (motorCommutationDelay > 17) {
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

  HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  __enable_irq();
}

void motorStartupTune() {
  motorPwmTimerHandle.Instance->CCR1 = 5;
  motorPwmTimerHandle.Instance->CCR2 = 5;
  motorPwmTimerHandle.Instance->CCR3 = 5;

  motorCommutationStep(motorStep);
  motorPwmTimerHandle.Instance->PSC = 100;
  HAL_Delay(100);
  motorPwmTimerHandle.Instance->PSC = 75;
  HAL_Delay(100);
  motorPwmTimerHandle.Instance->PSC = 50;
  HAL_Delay(100);

  motorPwmTimerHandle.Instance->PSC = 0;
}

void motorInputTune(uint8_t motorStepDebug) {
  motorPwmTimerHandle.Instance->CCR1 = 5;
  motorPwmTimerHandle.Instance->CCR2 = 5;
  motorPwmTimerHandle.Instance->CCR3 = 5;

  motorCommutationStep(motorStepDebug);
  motorPwmTimerHandle.Instance->PSC = 75;
  HAL_Delay(100);
  motorPwmTimerHandle.Instance->PSC = 50;
  HAL_Delay(100);

  motorPwmTimerHandle.Instance->PSC = 0;
}
