#include "motor.h"

TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, motorSinTimerHandle;
COMP_HandleTypeDef motorBemfComparatorHandle;
motorStructure motor;

extern medianStructure motorCommutationIntervalFilterState;

// ToDo
const int pwmSin[] = {
  128,130,132,134,136,139,141,143,145,147,150,152,154,156,158,160,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,
  202,204,206,208,209,211,213,214,216,218,219,221,222,224,225,227,228,229,231,232,233,234,236,237,238,239,240,241,242,243,244,245,246,247,247,248,
  249,249,250,251,251,252,252,253,253,253,254,254,254,255,255,255,255,255,255,255,255,255,255,255,254,254,254,253,253,253,252,252,251,251,250,249,
  249,248,247,247,246,245,244,243,242,241,240,239,238,237,236,234,233,232,231,229,228,227,225,224,222,221,219,218,216,214,213,211,209,208,206,204,
  202,201,199,197,195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,160,158,156,154,152,150,147,145,143,141,139,136,134,132,130,
  128,125,123,121,119,116,114,112,110,108,105,103,101, 99, 97, 95, 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54,
   53, 51, 49, 47, 46, 44, 42, 41, 39, 37, 36, 34, 33, 31, 30, 28, 27, 26, 24, 23, 22, 21, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10,  9,  8,  8,  7,
    6,  6,  5,  4,  4,  3,  3,  2,  2,  2,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  2,  2,  2,  3,  3,  4,  4,  5,  6,
    6,  7,  8,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 27, 28, 30, 31, 33, 34, 36, 37, 39, 41, 42, 44, 46, 47, 49, 51,
   53, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 95, 97, 99,101,103,105,108,110,112,114,116,119,121,123,125
};

#if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
  uint32_t motorDebugTime;
#endif

int32_t motorPhaseAstep  = 0, motorPhaseBstep = 0, motorPhaseCstep = 0;
int32_t gateDriveOffset = 0;

#pragma GCC push_options
#pragma GCC optimize("O3")
// ISR takes 7us, 300ns jitter (5us on KISS24A)
INLINE_CODE void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *comparatorHandle) {
  __disable_irq();

  uint32_t motorCommutationTimestamp = motorCommutationTimerHandle.Instance->CNT;

  #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
    uint32_t motorDebugStart = motorCommutationTimerHandle.Instance->CNT;
  #endif

  // ToDo
  //if ((!motor.Running) || (!motor.Start)) {
  if (!motor.Running) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      #if defined(KISS24A)
        __HAL_COMP_COMP2_EXTI_DISABLE_IT();
      #else
        __HAL_COMP_COMP1_EXTI_DISABLE_IT();
      #endif
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
    #if defined(KISS24A)
      __HAL_COMP_COMP2_EXTI_DISABLE_IT();
    #else
      __HAL_COMP_COMP1_EXTI_DISABLE_IT();
    #endif
  #endif

  motor.BemfCounter++;
  motor.BemfZeroCounterTimeout = 0;
  motor.BemfZeroCrossTimestamp = motorCommutationTimestamp;
  medianPush(&motorCommutationIntervalFilterState, motorCommutationTimestamp);

  // ToDo
  /*
  if (motor.CommutationDelay > 40) {
    while (motorCommutationTimerHandle.Instance->CNT < motor.CommutationDelay) {
      // noop
    }
  }*/

  motorCommutate();

  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  #else
    #if defined(KISS24A)
      __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
      __HAL_COMP_COMP2_EXTI_ENABLE_IT();
    #else
      __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
      __HAL_COMP_COMP1_EXTI_ENABLE_IT();
    #endif
  #endif

  #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
    motorDebugTime = motorCommutationTimerHandle.Instance->CNT - motorDebugStart;
  #endif

  #if (defined(_DEBUG_) && defined(DEBUG_MOTOR_TIMING))
    LED_OFF(LED_GREEN);
  #endif

  motorCommutationTimerHandle.Instance->CNT = 0;

  __enable_irq();
}

#if defined(FD6288)
  // precomputed H_BRIDGE masks
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
          A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
          A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        } else {
          A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskAlternate);
        }
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseB(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if(!motor.ComplementaryPWM  || motor.BrakeActiveProportional) {
          B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
          B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        } else {
          B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskAlternate);
        }
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseC(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if (!motor.ComplementaryPWM || motor.BrakeActiveProportional) {
          C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
          C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        } else {
          C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskAlternate);
        }
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        break;
    }
  }
#endif

#if defined(NCP3420)
// precomputed H_BRIDGE masks
const uint32_t aFetOeClearmask = (~((A_FET_OE_PIN * A_FET_OE_PIN) * GPIO_MODER_MODER0));
const uint32_t aFetOeSetmaskOutput = ((A_FET_OE_PIN * A_FET_OE_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t aFetInClearmask = (~((A_FET_IN_PIN * A_FET_IN_PIN) * GPIO_MODER_MODER0));
const uint32_t aFetInSetmaskOutput = ((A_FET_IN_PIN * A_FET_IN_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t aFetInSetmaskAlternate = ((A_FET_IN_PIN * A_FET_IN_PIN) * LL_GPIO_MODE_ALTERNATE);
const uint32_t bFetOeClearmask = (~((B_FET_OE_PIN * B_FET_OE_PIN) * GPIO_MODER_MODER0));
const uint32_t bFetOeSetmaskOutput = ((B_FET_OE_PIN * B_FET_OE_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t bFetInClearmask = (~((B_FET_IN_PIN * B_FET_IN_PIN) * GPIO_MODER_MODER0));
const uint32_t bFetInSetmaskOutput = ((B_FET_IN_PIN * B_FET_IN_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t bFetInSetmaskAlternate = ((B_FET_IN_PIN * B_FET_IN_PIN) * LL_GPIO_MODE_ALTERNATE);
const uint32_t cFetOeClearmask = (~((C_FET_OE_PIN * C_FET_OE_PIN) * GPIO_MODER_MODER0));
const uint32_t cFetOeSetmaskOutput = ((C_FET_OE_PIN * C_FET_OE_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t cFetInClearmask = (~((C_FET_IN_PIN * C_FET_IN_PIN) * GPIO_MODER_MODER0));
const uint32_t cFetInSetmaskOutput = ((C_FET_IN_PIN * C_FET_IN_PIN) * LL_GPIO_MODE_OUTPUT);
const uint32_t cFetInSetmaskAlternate = ((C_FET_IN_PIN * C_FET_IN_PIN) * LL_GPIO_MODE_ALTERNATE);

  INLINE_CODE void motorPhaseA(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        A_FET_OE_GPIO->BSRR = A_FET_OE_PIN;
        A_FET_IN_GPIO->MODER = ((A_FET_IN_GPIO->MODER & aFetInClearmask) | aFetInSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        A_FET_OE_GPIO->BRR = A_FET_OE_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        A_FET_OE_GPIO->BSRR = A_FET_OE_PIN;
        A_FET_IN_GPIO->MODER = ((A_FET_IN_GPIO->MODER & aFetInClearmask) | aFetInSetmaskOutput);
        A_FET_IN_GPIO->BRR = A_FET_IN_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseB(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        B_FET_OE_GPIO->BSRR = B_FET_OE_PIN;
        B_FET_IN_GPIO->MODER = ((B_FET_IN_GPIO->MODER & bFetInClearmask) | bFetInSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        B_FET_OE_GPIO->BRR = B_FET_OE_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        B_FET_OE_GPIO->BSRR = B_FET_OE_PIN;
        B_FET_IN_GPIO->MODER = ((B_FET_IN_GPIO->MODER & bFetInClearmask) | bFetInSetmaskOutput);
        B_FET_IN_GPIO->BRR = B_FET_IN_PIN;
        break;
    }
  }

  INLINE_CODE void motorPhaseC(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        C_FET_OE_GPIO->BSRR = C_FET_OE_PIN;
        C_FET_IN_GPIO->MODER = ((C_FET_IN_GPIO->MODER & cFetInClearmask) | cFetInSetmaskAlternate);
        break;
      case HBRIDGE_FLOATING:
        C_FET_OE_GPIO->BRR = C_FET_OE_PIN;
        break;
      case HBRIDGE_LOWSIDE:
        C_FET_OE_GPIO->BSRR = C_FET_OE_PIN;
        C_FET_IN_GPIO->MODER = ((C_FET_IN_GPIO->MODER & cFetInClearmask) | cFetInSetmaskOutput);
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
      #if defined(KISS24A)
        __HAL_COMP_COMP2_EXTI_DISABLE_RISING_EDGE();
        __HAL_COMP_COMP2_EXTI_ENABLE_FALLING_EDGE();
      #else
        __HAL_COMP_COMP1_EXTI_DISABLE_RISING_EDGE();
        __HAL_COMP_COMP1_EXTI_ENABLE_FALLING_EDGE();
      #endif
    #endif
  } else {
    #if (!defined(COMPARATOR_OPTIMIZE))
      motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
    #else
      #if defined(KISS24A)
        __HAL_COMP_COMP2_EXTI_DISABLE_FALLING_EDGE();
        __HAL_COMP_COMP2_EXTI_ENABLE_RISING_EDGE();
      #else
        __HAL_COMP_COMP1_EXTI_DISABLE_FALLING_EDGE();
        __HAL_COMP_COMP1_EXTI_ENABLE_RISING_EDGE();
      #endif
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
  if (!motor.Running) {
    bool bufferComplementaryPWM = motor.ComplementaryPWM;

    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      #if defined(KISS24A)
        __HAL_COMP_COMP2_EXTI_DISABLE_IT();
      #else
        __HAL_COMP_COMP1_EXTI_DISABLE_IT();
      #endif
    #endif

    motor.ComplementaryPWM = true;

    motorCommutate();

    motorCommutationTimerHandle.Instance->CNT = 0;
    motor.BemfCounter = 0;
    motor.Running = true;

    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Start_IT(&motorBemfComparatorHandle);
    #else
      #if defined(KISS24A)
        __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
        __HAL_COMP_COMP2_EXTI_ENABLE_IT();
      #else
        __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
        __HAL_COMP_COMP1_EXTI_ENABLE_IT();
      #endif
    #endif

    motor.ComplementaryPWM = bufferComplementaryPWM;
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

void motorBrakeProportional() {
  motorPhaseA(HBRIDGE_PWM);
  motorPhaseB(HBRIDGE_PWM);
  motorPhaseC(HBRIDGE_PWM);
}

void motorTuneStartup() {
  #if defined(NCP3420)
    A_FET_OE_GPIO->MODER = ((A_FET_OE_GPIO->MODER & aFetOeClearmask) | aFetOeSetmaskOutput);
    B_FET_OE_GPIO->MODER = ((B_FET_OE_GPIO->MODER & bFetOeClearmask) | bFetOeSetmaskOutput);
    C_FET_OE_GPIO->MODER = ((C_FET_OE_GPIO->MODER & cFetOeClearmask) | cFetOeSetmaskOutput);
  #endif

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
      motor.Start = false;
      input.DataNormed = 0;
      input.DataNormedLast = 0;
      input.PwmValue = 0;
      if ((!motor.Running) || (!motor.Start)) {
        inputDshotCommandRun();
      }
    } else {
      input.DataNormed = constrain((input.Data - DSHOT_CMD_MAX), INPUT_NORMED_MIN, INPUT_NORMED_MAX);
      // trapezium rule
      input.DataNormed = (input.DataNormed + input.DataNormedLast) >> 1;
      input.DataNormedLast = input.DataNormed;

      if ((escConfig()->motor3Dmode) && (input.Protocol == PROSHOT)) {
        // 3D
        if (input.DataNormed >= escConfig()->input3DdeadbandHigh) {
          // up
          motor.Start = true;
          if (motor.Direction == !escConfig()->motorDirection) {
            motor.BemfCounter = 0;
            motor.Direction = escConfig()->motorDirection;
          }
          input.PwmValue = (input.DataNormed - escConfig()->input3Dneutral) + escConfig()->motorStartThreshold;
        }

        if ((input.DataNormed < escConfig()->input3Dneutral) && (input.DataNormed >= escConfig()->input3DdeadbandLow)) {
          // down
          motor.Start = true;
          if (motor.Direction == escConfig()->motorDirection) {
            motor.BemfCounter = 0;
            motor.Direction = !escConfig()->motorDirection;
          }
          input.PwmValue = input.DataNormed + escConfig()->motorStartThreshold;
        }

        /*
        if ((input.DataNormed < escConfig()->input3DdeadbandLow) || ((input.DataNormed < escConfig()->input3DdeadbandHigh) && ((input.DataNormed > escConfig()->input3Dneutral)))) {
          // deadband
        }*/
      } else {
        // 2D
        motor.Start = true;
        input.PwmValue = (input.DataNormed >> 1) + escConfig()->motorStartThreshold;
      }

      if (motor.Start) {
        if (motor.BemfCounter < MOTOR_ONE_ROTATION) {
          // stall protection and startup kick
          input.PwmValue = escConfig()->motorStartupPower;
        } else {
          input.PwmValue = constrain(input.PwmValue, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
        }
      }

      motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
    }
  } else {
    motor.Start = false;
    input.DataNormed = 0;
    input.DataNormedLast = 0;
    input.PwmValue = 0;
  }
}

INLINE_CODE uint32_t motorGetErpm(void) {
  if (motor.CommutationInterval > 0) {
    return(MOTOR_ERPM_FACTOR / motor.CommutationInterval);
  } else {
    return(0);
  }
}

INLINE_CODE uint32_t motorGetRpm(void) {
  if (motor.CommutationInterval > 0) {
    return((MOTOR_ERPM_FACTOR / (motor.CommutationInterval * escConfig()->motorPoles >> 1)));
  } else {
    return(0);
  }
}
#pragma GCC pop_options


// ToDo
void motorComutateSin() {
  if (motor.Start  && (!motor.Running)) {
    if (motor.Direction == SPIN_CW) {
      if (++motorPhaseAstep > 359) {
        motorPhaseAstep = 0 ;
      }
      if (++motorPhaseBstep > 359) {
        motorPhaseBstep = 0 ;
      }
      if (++motorPhaseCstep > 359) {
        motorPhaseCstep = 0 ;
      }
    } else {
      if (--motorPhaseAstep < 0) {
        motorPhaseAstep = 359 ;
      }
      if (--motorPhaseBstep < 0) {
        motorPhaseBstep = 359;
      }
      if (--motorPhaseCstep < 0){
        motorPhaseCstep = 359 ;
      }
    }

    motorPwmTimerHandle.Instance->CCR1 = (pwmSin[motorPhaseAstep] + gateDriveOffset);
    motorPwmTimerHandle.Instance->CCR2 = (pwmSin[motorPhaseBstep] + gateDriveOffset);
    motorPwmTimerHandle.Instance->CCR3 = (pwmSin[motorPhaseCstep] + gateDriveOffset);

    motorCommutate();

  }
}
