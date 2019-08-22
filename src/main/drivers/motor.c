#include "motor.h"

TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle;
COMP_HandleTypeDef motorBemfComparatorHandle;
motor_t motor;

void motorPhaseA(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
    case HBRIDGE_PWM:
      LL_GPIO_SetPinMode(A_FET_OE_GPIO, A_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_OE_GPIO->BSRR = A_FET_OE_PIN;
      LL_GPIO_SetPinMode(A_FET_IN_GPIO, A_FET_IN_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(A_FET_OE_GPIO, A_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_OE_GPIO->BRR = A_FET_OE_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(A_FET_OE_GPIO, A_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_OE_GPIO->BSRR = A_FET_OE_PIN;
      LL_GPIO_SetPinMode(A_FET_IN_GPIO, A_FET_IN_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_IN_GPIO->BRR = A_FET_IN_PIN;
      break;
  }
}

void motorPhaseB(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
    case HBRIDGE_PWM:
      LL_GPIO_SetPinMode(B_FET_OE_GPIO, B_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_OE_GPIO->BSRR = B_FET_OE_PIN;
      LL_GPIO_SetPinMode(B_FET_IN_GPIO, B_FET_IN_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(B_FET_OE_GPIO, B_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_OE_GPIO->BRR = B_FET_OE_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(B_FET_OE_GPIO, B_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_OE_GPIO->BSRR = B_FET_OE_PIN;
      LL_GPIO_SetPinMode(B_FET_IN_GPIO, B_FET_IN_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_IN_GPIO->BRR = B_FET_IN_PIN;
      break;
  }
}

void motorPhaseC(uint8_t hBridgeMode) {
  switch (hBridgeMode) {
    case HBRIDGE_PWM:
      LL_GPIO_SetPinMode(C_FET_OE_GPIO, C_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_OE_GPIO->BSRR = C_FET_OE_PIN;
      LL_GPIO_SetPinMode(C_FET_IN_GPIO, C_FET_IN_PIN, LL_GPIO_MODE_ALTERNATE);
      break;
    case HBRIDGE_FLOATING:
      LL_GPIO_SetPinMode(C_FET_OE_GPIO, C_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_OE_GPIO->BRR = C_FET_OE_PIN;
      break;
    case HBRIDGE_LOWSIDE:
      LL_GPIO_SetPinMode(C_FET_OE_GPIO, C_FET_OE_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_OE_GPIO->BSRR = C_FET_OE_PIN;
      LL_GPIO_SetPinMode(C_FET_IN_GPIO, C_FET_IN_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_IN_GPIO->BRR = C_FET_IN_PIN;
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
  switch(motor.Step) {
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
  if (motor.BemfRising) {
    motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  } else {
    motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  HAL_COMP_Init(&motorBemfComparatorHandle);
}

void motorCommutate() {
  if (motor.Direction == SPIN_CW) {
    if (++motor.Step > 6) {
      motor.Step = 1;
    }

    if ((motor.Step == 1) || (motor.Step == 3) || (motor.Step == 5)) {
      motor.BemfRising = true;
    } else {
      motor.BemfRising = false;
    }
  } else {
    if (--motor.Step < 1) {
      motor.Step = 6;
    }

    if ((motor.Step == 1) || (motor.Step == 3) || (motor.Step == 5)) {
      motor.BemfRising = false;
    } else {
      motor.BemfRising = true;
    }
  }

  motorCommutationStep(motor.Step);
  motorChangeComparatorInput();
}

void motorStart() {
  bool bufferDecaystate = motor.SlowDecay;

  if (!motor.Running) {
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    motor.SlowDecay = true;

    motorCommutate();

    motorCommutationTimerHandle.Instance->CNT = 0xffff;
    motor.BemfCounter = 0;
    motor.Running = true;
    HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  }
  motor.SlowDecay = bufferDecaystate;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  uint32_t motorTimestamp;

  __disable_irq();
  motorTimestamp = motorCommutationTimerHandle.Instance->CNT;

  if ((!motor.Running) || (!motor.Startup)) {
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    __enable_irq();
    return;
  }

  while ((motorCommutationTimerHandle.Instance->CNT - motorTimestamp) < motor.BemfFilterDelay);

  for (int i = 0; i < motor.BemfFilterLevel; i++) {
    if ((motor.BemfRising && HAL_COMP_GetOutputLevel(&motorBemfComparatorHandle) == COMP_OUTPUTLEVEL_HIGH) ||
        (!motor.BemfRising && HAL_COMP_GetOutputLevel(&motorBemfComparatorHandle) == COMP_OUTPUTLEVEL_LOW)) {

      __enable_irq();
      return;
    }
  }

  #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
    LED_ON(LED_GREEN);
  #endif

  HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
  motorCommutationTimerHandle.Instance->CNT = 0xffff;

  motor.BemfCounter++;
  motor.BemfZeroCounterTimeout = 0;
  motor.BemfZeroCrossTimestamp = motorTimestamp;

  // ToDo
  if ((motor.CommutationDelay > 31) && (motor.CommutationDelay < 613) && (input.PwmValue > 45) && (input.PwmValue < 707)) {
    while (motorCommutationTimerHandle.Instance->CNT < motor.CommutationDelay) {
      #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
        LED_TOGGLE(LED_BLUE);
      #endif
    }
  }

  #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
    LED_OFF(LED_GREEN);
    LED_OFF(LED_BLUE);
  #endif

  motorCommutate();

  HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  __enable_irq();
}

void motorStartupTune() {
  motorPwmTimerHandle.Instance->CCR1 = 5;
  motorPwmTimerHandle.Instance->CCR2 = 5;
  motorPwmTimerHandle.Instance->CCR3 = 5;

  motorCommutationStep(motor.Step);
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
