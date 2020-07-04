#include "motor.h"

TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle;
COMP_HandleTypeDef motorBemfComparatorHandle;
motorStructure motor;

extern medianStructure motorCommutationIntervalFilterState;

//debug
uint32_t motorDebugTime;

#pragma GCC push_options
#pragma GCC optimize("O3")
INLINE_CODE void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *comparatorHandle) {
  __disable_irq();

  uint32_t motorCommutationTimestamp = motorCommutationTimerHandle.Instance->CNT;

  //debug
  uint32_t motorDebugStart = motorCommutationTimerHandle.Instance->CNT;

  if ((!motor.Running) || (!motor.Startup)) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      __HAL_COMP_COMP1_EXTI_DISABLE_IT();
    #endif

    __enable_irq();
    return;
  }

  while ((motorCommutationTimerHandle.Instance->CNT - motorCommutationTimestamp) < motor.BemfFilterDelay);

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

  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
  #else
    __HAL_COMP_COMP1_EXTI_DISABLE_IT();
  #endif

  motor.BemfCounter++;
  motor.BemfZeroCounterTimeout = 0;
  motor.BemfZeroCrossTimestamp = motorCommutationTimestamp;
  medianPush(&motorCommutationIntervalFilterState, motorCommutationTimestamp);

  // ToDo
  /*
  if (motor.CommutationDelay > 40) {
    while (motorCommutationTimerHandle.Instance->CNT < motor.CommutationDelay) {
      #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
        LED_TOGGLE(LED_BLUE);
      #endif
    }
  }*/

  #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
    LED_OFF(LED_GREEN);
    LED_OFF(LED_BLUE);
  #endif

  motorCommutate();

  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  #else
    __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP1_EXTI_ENABLE_IT();
  #endif

  //debug
  motorDebugTime = motorCommutationTimerHandle.Instance->CNT - motorDebugStart;

  motorCommutationTimerHandle.Instance->CNT = 0;

  __enable_irq();
}

#if defined(FD6288)
  // pre computed H_BRIDGE masks
  const uint32_t aFetLoClearmask = (~((A_FET_LO_PIN * A_FET_LO_PIN) * GPIO_MODER_MODER0));
  const uint32_t aFetLoSetmaskOutput = ((A_FET_LO_PIN * A_FET_LO_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t aFetLoSetmaskAlternate = ((A_FET_LO_PIN * A_FET_LO_PIN) * LL_GPIO_MODE_ALTERNATE);
  const uint32_t aFetHiClearmask = (~((A_FET_HI_PIN * A_FET_HI_PIN) * GPIO_MODER_MODER0));
  const uint32_t aFetHiSetmaskOutput = ((A_FET_HI_PIN * A_FET_HI_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t aFetHiSetmaskAlternate = ((A_FET_HI_PIN * A_FET_HI_PIN) * LL_GPIO_MODE_ALTERNATE);
  const uint32_t bFetLoClearmask = (~((B_FET_LO_PIN * B_FET_LO_PIN) * GPIO_MODER_MODER0));
  const uint32_t bFetLoSetmaskOutput = ((B_FET_LO_PIN * B_FET_LO_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t bFetLoSetmaskAlternate = ((B_FET_LO_PIN * B_FET_LO_PIN) * LL_GPIO_MODE_ALTERNATE);
  const uint32_t bFetHiClearmask = (~((B_FET_HI_PIN * B_FET_HI_PIN) * GPIO_MODER_MODER0));
  const uint32_t bFetHiSetmaskOutput = ((B_FET_HI_PIN * B_FET_HI_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t bFetHiSetmaskAlternate = ((B_FET_HI_PIN * B_FET_HI_PIN) * LL_GPIO_MODE_ALTERNATE);
  const uint32_t cFetLoClearmask = (~((C_FET_LO_PIN * C_FET_LO_PIN) * GPIO_MODER_MODER0));
  const uint32_t cFetLoSetmaskOutput = ((C_FET_LO_PIN * C_FET_LO_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t cFetLoSetmaskAlternate = ((C_FET_LO_PIN * C_FET_LO_PIN) * LL_GPIO_MODE_ALTERNATE);
  const uint32_t cFetHiClearmask = (~((C_FET_HI_PIN * C_FET_HI_PIN) * GPIO_MODER_MODER0));
  const uint32_t cFetHiSetmaskOutput = ((C_FET_HI_PIN * C_FET_HI_PIN) * LL_GPIO_MODE_OUTPUT);
  const uint32_t cFetHiSetmaskAlternate = ((C_FET_HI_PIN * C_FET_HI_PIN) * LL_GPIO_MODE_ALTERNATE);

  INLINE_CODE void motorPhaseA(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if (!motor.ComplementaryPWM || motor.BrakeActiveProportional) {
          //LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
          A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
          A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        } else {
          //LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
          A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskAlternate);
        }
        //LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        //LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        //LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        //LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
        //LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseB(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if(!motor.ComplementaryPWM  || motor.BrakeActiveProportional) {
          //LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
          B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
          B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        } else {
          //LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
          B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskAlternate);
        }
        //LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        //LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        //LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        //LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
        //LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseC(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if (!motor.ComplementaryPWM || motor.BrakeActiveProportional) {
          //LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
          C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
          C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        } else {
          //LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
          C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskAlternate);
        }
        //LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        //LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        //LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        //LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
        //LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        break;
    }
  }
#endif

#if defined(NCP3420)
  INLINE_CODE void motorPhaseA(uint8_t hBridgeMode) {
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

  INLINE_CODE void motorPhaseB(uint8_t hBridgeMode) {
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

  INLINE_CODE void motorPhaseC(uint8_t hBridgeMode) {
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
#endif

INLINE_CODE void motorCommutationStep(uint8_t stepBuffer) {

  switch (stepBuffer) {
    case 1:
      // A-B
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

INLINE_CODE void motorComparatorInputChange() {
  switch (motor.Step) {
    case 1:
    case 4:
      // C floating
      #if (!defined(COMPARATOR_OPTIMIZE))
        motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_C;
      #else
        COMP->CSR = COMPARATOR_PHASE_C_CSR;
      #endif
      break;
    case 2:
    case 5:
      // B floating
      #if (!defined(COMPARATOR_OPTIMIZE))
        motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_B;
      #else
        COMP->CSR = COMPARATOR_PHASE_B_CSR;
      #endif
      break;
    case 3:
    case 6:
      // A floating
      #if (!defined(COMPARATOR_OPTIMIZE))
        motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_A;
      #else
        COMP->CSR = COMPARATOR_PHASE_A_CSR;
      #endif
      break;
  }

  // input comparator polarity is reversed
  if (motor.BemfRising) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
    #else
      __HAL_COMP_COMP1_EXTI_DISABLE_RISING_EDGE();
      __HAL_COMP_COMP1_EXTI_ENABLE_FALLING_EDGE();
    #endif
  } else {
    #if (!defined(COMPARATOR_OPTIMIZE))
      motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
    #else
      __HAL_COMP_COMP1_EXTI_DISABLE_FALLING_EDGE();
      __HAL_COMP_COMP1_EXTI_ENABLE_RISING_EDGE();
    #endif
  }
  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Init(&motorBemfComparatorHandle);
  #endif
}

INLINE_CODE void motorCommutate() {
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
  motorComparatorInputChange();
}
#pragma GCC pop_options

void motorStart() {
  bool bufferComplementaryPWM = motor.ComplementaryPWM;

  if (!motor.Running) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      __HAL_COMP_COMP1_EXTI_DISABLE_IT();
    #endif

    motor.ComplementaryPWM = true;

    motorCommutate();

    motorCommutationTimerHandle.Instance->CNT = 0;
    motor.BemfCounter = 0;
    motor.Running = true;
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Start_IT(&motorBemfComparatorHandle);
    #else
      __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
      __HAL_COMP_COMP1_EXTI_ENABLE_IT();
    #endif
  }
  motor.ComplementaryPWM = bufferComplementaryPWM;
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

void motorBrakeProportional() {
  motorPhaseA(HBRIDGE_PWM);
  motorPhaseB(HBRIDGE_PWM);
  motorPhaseC(HBRIDGE_PWM);
}

void motorTuneStartup() {
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

void motorTuneInput(uint8_t motorStepDebug) {
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

#pragma GCC push_options
#pragma GCC optimize("O3")
INLINE_CODE void motorInputUpdate(void) {
  if (input.Armed) {
    if (input.Data <= DSHOT_CMD_MAX) {
      motor.Startup = false;
      input.DataNormed = 0;
      input.PwmValue = 0;
      if ((!motor.Running) || (!motor.Startup)) {
        inputDshotCommandRun();
      }
    } else {
      motor.Startup = true;
      input.DataNormed = constrain((input.Data - DSHOT_CMD_MAX), INPUT_NORMED_MIN, INPUT_NORMED_MAX);

      if ((escConfig()->motor3Dmode) && (input.Protocol == PROSHOT)) {
        // up
        if (input.DataNormed >= escConfig()->input3DdeadbandHigh) {
          if (motor.Direction == !escConfig()->motorDirection) {
            motor.Direction = escConfig()->motorDirection;
            motor.BemfCounter = 0;
          }
          input.PwmValue = (input.DataNormed - escConfig()->input3Dneutral) + escConfig()->motorStartThreshold;
        }
        // down
        if ((input.DataNormed < escConfig()->input3Dneutral) && (input.DataNormed >= escConfig()->input3DdeadbandLow)) {
          if (motor.Direction == escConfig()->motorDirection) {
            motor.Direction = !escConfig()->motorDirection;
            motor.BemfCounter = 0;
          }
          input.PwmValue = input.DataNormed + escConfig()->motorStartThreshold;
        }
        // deadband
        if ((input.DataNormed < escConfig()->input3DdeadbandLow) || ((input.DataNormed < escConfig()->input3DdeadbandHigh) && ((input.DataNormed > escConfig()->input3Dneutral)))) {
          input.PwmValue = 0;
        }
      } else {
        input.PwmValue = (input.DataNormed >> 1) + escConfig()->motorStartThreshold;
      }

      // stall protection and startup kick
      if (motor.BemfCounter < motor.BemfZeroCounterTimeoutThreshold) {
        input.PwmValue = escConfig()->motorStartupPower;
      } else {
        input.PwmValue = constrain(input.PwmValue, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
      }
      motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
    }
  }
}
#pragma GCC pop_options

uint32_t motorGetErpm(void) {
  if (motor.CommutationInterval > 0) {
    return(MOTOR_ERPM_FACTOR / motor.CommutationInterval);
  } else {
    return(0);
  }
}

uint32_t motorGetRpm(void) {
  if (motor.CommutationInterval > 0) {
    return((MOTOR_ERPM_FACTOR / (motor.CommutationInterval * escConfig()->motorPoles >> 1)));
  } else {
    return(0);
  }
}
