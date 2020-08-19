#include "motor.h"

TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle, motorSinTimerHandle;
COMP_HandleTypeDef motorBemfComparatorHandle;
motorStructure motor;

extern medianStructure motorCommutationIntervalFilterState;

#if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
  uint32_t motorDebugTime;
#endif

#pragma GCC push_options
#pragma GCC optimize("O3")
// ISR takes 7us, 300ns jitter (5us on KISS24A)
INLINE_CODE void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *comparatorHandle) {
  uint32_t motorCommutationTimestamp = motorCommutationTimerHandle.Instance->CNT;

  __disable_irq();

  #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
    uint32_t motorDebugStart = motorCommutationTimerHandle.Instance->CNT;
  #endif

  if ((!motor.Running) || (!motor.Start)) {
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

  // ToDo
  //if ((motor.Running) && (motor.BemfZeroCrossTimestamp < 8001) && (motor.BemfZeroCrossTimestamp > 51)) {
    motorSinTimerHandle.Instance->ARR = (motor.BemfZeroCrossTimestamp >> 1) & 0xfffff;
    //motorSinTimerHandle.Instance->ARR = 0xfff;
    motorSinTimerHandle.Instance->CNT = 0x0;
    LED_TOGGLE(LED_GREEN);
    HAL_TIM_Base_Start_IT(&motorSinTimerHandle);
    //HAL_NVIC_EnableIRQ(TIM17_IRQn);
  //}

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

  #if defined(KISS24A)
    __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
  #else
    __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
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
      case HBRIDGE_FLOAT:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        break;
      case HBRIDGE_LO:
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
        break;
      case HBRIDGE_HI:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BSRR = A_FET_HI_PIN;
        break;
      case HBRIDGE_PWM_LO:
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskOutput);
        A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskAlternate);
        break;
      case HBRIDGE_PWM_HI:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskOutput);
        A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskAlternate);
        break;
      case HBRIDGE_PWM_COMPLEMENTARY:
        A_FET_LO_GPIO->MODER = ((A_FET_LO_GPIO->MODER & aFetLoClearmask) | aFetLoSetmaskAlternate);
        A_FET_HI_GPIO->MODER = ((A_FET_HI_GPIO->MODER & aFetHiClearmask) | aFetHiSetmaskAlternate);
        break;
    }
  }

  INLINE_CODE void motorPhaseB(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_FLOAT:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        break;
      case HBRIDGE_LO:
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
        break;
      case HBRIDGE_HI:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BSRR = B_FET_HI_PIN;
        break;
      case HBRIDGE_PWM_LO:
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskOutput);
        B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskAlternate);
        break;
      case HBRIDGE_PWM_HI:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskOutput);
        B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskAlternate);
        break;
      case HBRIDGE_PWM_COMPLEMENTARY:
        B_FET_LO_GPIO->MODER = ((B_FET_LO_GPIO->MODER & bFetLoClearmask) | bFetLoSetmaskAlternate);
        B_FET_HI_GPIO->MODER = ((B_FET_HI_GPIO->MODER & bFetHiClearmask) | bFetHiSetmaskAlternate);
        break;
    }
  }

  INLINE_CODE void motorPhaseC(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_FLOAT:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        break;
      case HBRIDGE_LO:
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
        break;
      case HBRIDGE_HI:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BSRR = C_FET_HI_PIN;
        break;
      case HBRIDGE_PWM_LO:
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskOutput);
        C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskAlternate);
        break;
      case HBRIDGE_PWM_HI:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskOutput);
        C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskAlternate);
        break;
      case HBRIDGE_PWM_COMPLEMENTARY:
        C_FET_LO_GPIO->MODER = ((C_FET_LO_GPIO->MODER & cFetLoClearmask) | cFetLoSetmaskAlternate);
        C_FET_HI_GPIO->MODER = ((C_FET_HI_GPIO->MODER & cFetHiClearmask) | cFetHiSetmaskAlternate);
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
      case HBRIDGE_FLOAT:
        A_FET_OE_GPIO->BRR = A_FET_OE_PIN;
        break;
      case HBRIDGE_LO:
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
      case HBRIDGE_FLOAT:
        B_FET_OE_GPIO->BRR = B_FET_OE_PIN;
        break;
      case HBRIDGE_LO:
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
      case HBRIDGE_FLOAT:
        C_FET_OE_GPIO->BRR = C_FET_OE_PIN;
        break;
      case HBRIDGE_LO:
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
      motorPhaseA(HBRIDGE_LO);
      motorPhaseB(HBRIDGE_PWM_COMPLEMENTARY);
      motorPhaseC(HBRIDGE_FLOAT);
      break;
    case 2:
      // A-C
      motorPhaseA(HBRIDGE_LO);
      motorPhaseB(HBRIDGE_FLOAT);
      motorPhaseC(HBRIDGE_PWM_COMPLEMENTARY);
      break;
    case 3:
      // B-C
      motorPhaseA(HBRIDGE_FLOAT);
      motorPhaseB(HBRIDGE_LO);
      motorPhaseC(HBRIDGE_PWM_COMPLEMENTARY);
      break;
    case 4:
      // B-A
      motorPhaseA(HBRIDGE_PWM_COMPLEMENTARY);
      motorPhaseB(HBRIDGE_LO);
      motorPhaseC(HBRIDGE_FLOAT);
      break;
    case 5:
      // C-A
      motorPhaseA(HBRIDGE_PWM_COMPLEMENTARY);
      motorPhaseB(HBRIDGE_FLOAT);
      motorPhaseC(HBRIDGE_LO);
      break;
    case 6:
      // C-B
      motorPhaseA(HBRIDGE_FLOAT);
      motorPhaseB(HBRIDGE_PWM_COMPLEMENTARY);
      motorPhaseC(HBRIDGE_LO);
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

    for (uint32_t i = 1; i < MOTOR_BLDC_STEPS; i++) {
      motorCommutate();
      HAL_Delay(2);
    }

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
  motorPhaseA(HBRIDGE_FLOAT);
  motorPhaseB(HBRIDGE_FLOAT);
  motorPhaseC(HBRIDGE_FLOAT);
}

void motorBrakeFull() {
  motorPhaseA(HBRIDGE_LO);
  motorPhaseB(HBRIDGE_LO);
  motorPhaseC(HBRIDGE_LO);
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
      input.PwmValue = 0;
      input.PwmValueLast = 0;
      if ((!motor.Running) || (!motor.Start)) {
        inputDshotCommandRun();
      }
    } else {
      input.DataNormed = constrain((input.Data - DSHOT_CMD_MAX), INPUT_NORMED_MIN, INPUT_NORMED_MAX);

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

      // trapezium rule
      input.PwmValue = (input.PwmValue + input.PwmValueLast) >> 1;
      input.PwmValueLast = input.PwmValue;

      motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
    }
  } else {
    motor.Start = false;
    input.DataNormed = 0;
    input.PwmValue = 0;
    input.PwmValueLast = 0;
  }
}

INLINE_CODE uint32_t motorGetErpm(void) {
  if (motor.OneErpmTime > 0) {
    return(MOTOR_ERPM_FACTOR / motor.OneErpmTime);
  } else {
    return(0);
  }
}

INLINE_CODE uint32_t motorGetRpm(void) {
  if (motor.OneErpmTime > 0) {
    return((MOTOR_ERPM_FACTOR / (motor.OneErpmTime * escConfig()->motorPoles >> 1)));
  } else {
    return(0);
  }
}
#pragma GCC pop_options
