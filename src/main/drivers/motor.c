#include "motor.h"

TIM_HandleTypeDef motorPwmTimerHandle, motorCommutationTimerHandle;
COMP_HandleTypeDef motorBemfComparatorHandle;
motor_t motor;

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *comparatorHandle) {
  uint32_t motorTimestamp;

  __disable_irq();
  motorTimestamp = motorCommutationTimerHandle.Instance->CNT;

  if ((!motor.Running) || (!motor.Startup)) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      EXTI->IMR &= BIT_LO(21);
      EXTI->PR &= BIT_LO(21);
    #endif

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

  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
  #else
    EXTI->IMR &= BIT_LO(21);
    EXTI->PR &= BIT_LO(21);
  #endif

  motorCommutationTimerHandle.Instance->CNT = 0xffff;

  motor.BemfCounter++;
  motor.BemfZeroCounterTimeout = 0;
  motor.BemfZeroCrossTimestamp = motorTimestamp;

  // ToDo
  if (motor.CommutationDelay > 40) {
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

  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Start_IT(&motorBemfComparatorHandle);
  #else
    EXTI->IMR |= BIT_HI(21);
  #endif

  __enable_irq();
}

#if defined(FD6288)
  void motorPhaseA(uint8_t hBridgeMode) {
    switch (hBridgeMode) {
      case HBRIDGE_PWM:
        if (!motor.SlowDecay || motor.BrakeActiveProportional) {
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
        if(!motor.SlowDecay  || motor.BrakeActiveProportional) {
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
        if (!motor.SlowDecay || motor.BrakeActiveProportional) {
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
#endif

#if defined(NCP3420)
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
#endif

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

void motorComparatorInputChange() {
  switch(motor.Step) {
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

  // polarity of comp input reversed
  if (motor.BemfRising) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
    #else
      EXTI->RTSR = 0x0;
      EXTI->FTSR = 0x200000;
    #endif
  } else {
    #if (!defined(COMPARATOR_OPTIMIZE))
      motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
    #else
      EXTI->RTSR = 0x200000;
      EXTI->FTSR = 0x0;
    #endif
  }
  #if (!defined(COMPARATOR_OPTIMIZE))
    HAL_COMP_Init(&motorBemfComparatorHandle);
  #endif
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
  motorComparatorInputChange();
}

void motorStart() {
  bool bufferDecaystate = motor.SlowDecay;

  if (!motor.Running) {
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Stop_IT(&motorBemfComparatorHandle);
    #else
      EXTI->IMR &= BIT_LO(21);
      EXTI->PR &= BIT_LO(21);
    #endif

    motor.SlowDecay = true;

    motorCommutate();

    motorCommutationTimerHandle.Instance->CNT = 0xffff;
    motor.BemfCounter = 0;
    motor.Running = true;
    #if (!defined(COMPARATOR_OPTIMIZE))
      HAL_COMP_Start_IT(&motorBemfComparatorHandle);
    #else
      EXTI->IMR |= BIT_HI(21);
    #endif
  }
  motor.SlowDecay = bufferDecaystate;
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

// escConfig()->motorBrakeStrength
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

void motorInputUpdate(void) {
  if (input.Armed) {
    if (input.Data <= DSHOT_CMD_MAX) {
      motor.Startup = false;
      input.PwmValue = 0;
      if ((!motor.Running) || (!motor.Startup)) {
        inputDshotCommandRun();
      }
    } else {
      motor.Startup = true;
      motor.BrakeActiveProportional = false;
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
          if(motor.Direction == escConfig()->motorDirection) {
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
        input.PwmValue = (input.DataNormed >> 1) + (escConfig()->motorStartThreshold);
      }

      // stall protection and startup kick
      if (motor.BemfCounter < 50) {
        #if (!defined(KISS24A))
          input.PwmValue = 71;
        #else
          input.PwmValue = 81;
        #endif
      } else {
        input.PwmValue = constrain(input.PwmValue, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
      }
      motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
      motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
    }
  }
}
