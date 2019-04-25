#include "motor.h"

COMP_HandleTypeDef comparator1Handle;
extern TIM_HandleTypeDef timer1Handle;

uint32_t motorTimestamp;
uint16_t motorStep = 1;

uint32_t motorZeroCrossTimestamp, motorZeroCrossTimestampLast;

// set proportianal to commutation time. with motorAdvance divisor
uint32_t motorAdvance = 0;

// increase divisor to decrease motorAdvance
uint16_t motorAdvanceDivisor = 3;
uint32_t motorBlanktime, motorWaitTime, motorCompit;

uint32_t motorTimer2StartArr = 9000;

bool motorSensorless;
bool motorStartup;
uint32_t motorCommutationInterval;
uint32_t motorFilterLevel = 1;
uint32_t motorFilterDelay = 2;
uint32_t motorDutyCycle = 100;

extern uint32_t motorZeroCounterTimeout;
// depends on speed of main loop
extern uint32_t motorZeroCounterTimeoutThreshold;


uint32_t motorBemfCounter;

bool motorDirection = 1;
bool motorRisingBEMF = 1;
bool motorRunning;

// 1 for complementary HBRIDGE_PWM , 0 for diode freewheeling
bool motorSlowDecay = true;
bool motorBrakeActiveProportional = true;

extern uint32_t input;



void motorAdvanceDivisorCalculate() {
    motorAdvanceDivisor = map((motorCommutationInterval),100,5000, 2, 20);
}

// motorPhaseB qfnf051 , phase A qfp32
#ifdef MP6531
void motorPhaseA(uint8_t newPhase)
#endif
#ifdef FD6288
void motorPhaseB(uint8_t newPhase)
#endif
{
  if (newPhase == HBRIDGE_PWM) {
    if(!motorSlowDecay  || motorBrakeActiveProportional) {
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
    return;
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
    return;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
    return;
  }

}

// phase c qfn , phase b qfp
#ifdef MP6531
void motorPhaseB(uint8_t newPhase)
#endif
#ifdef FD6288
void motorPhaseC(uint8_t newPhase)
#endif
{
  if (newPhase == HBRIDGE_PWM) {
    if (!motorSlowDecay || motorBrakeActiveProportional) {
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
    return;
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
    return;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
    return;
  }
}

// motorPhaseA qfn , phase C qfp
#ifdef MP6531
void motorPhaseC(uint8_t newPhase)
#endif
#ifdef FD6288
void motorPhaseA(uint8_t newPhase)
#endif
{
  if (newPhase == HBRIDGE_PWM) {
    if (!motorSlowDecay || motorBrakeActiveProportional) {
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
      A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
    } else {
      LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_ALTERNATE);
    }
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_ALTERNATE);
    return;
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
    return;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
    return;
  }

}

void motorCommutationStep(uint8_t newStep) {
  switch(newStep) {
    case 1:
      //A-B
      motorPhaseA(HBRIDGE_PWM);
      motorPhaseB(HBRIDGE_LOWSIDE);
      motorPhaseC(HBRIDGE_FLOATING);
      break;
    case 2:
      // C-B
      motorPhaseA(HBRIDGE_FLOATING);
      motorPhaseB(HBRIDGE_LOWSIDE);
      motorPhaseC(HBRIDGE_PWM);
      break;
    case 3:
      // C-A
      motorPhaseA(HBRIDGE_LOWSIDE);
      motorPhaseB(HBRIDGE_FLOATING);
      motorPhaseC(HBRIDGE_PWM);
      break;
    case 4:
      // B-A
      motorPhaseA(HBRIDGE_LOWSIDE);
      motorPhaseB(HBRIDGE_PWM);
      motorPhaseC(HBRIDGE_FLOATING);
      break;
    case 5:
      // B-C
      motorPhaseA(HBRIDGE_FLOATING);
      motorPhaseB(HBRIDGE_PWM);
      motorPhaseC(HBRIDGE_LOWSIDE);
      break;
    case 6:
      // A-C
      motorPhaseA(HBRIDGE_PWM);
      motorPhaseB(HBRIDGE_FLOATING);
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
      // c floating
      comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
      break;
    case 2:
    case 5:
      // a floating
      comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
      break;
    case 3:
    case 6:
      // b floating
      comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
      break;
  }

  if (motorRisingBEMF) {
    // polarity of comp output reversed
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  } else {
    // falling bemf
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  while (HAL_COMP_Init(&comparator1Handle) != HAL_OK);
}


void motorCommutate() {
  if (motorDirection == 1) {
    if (++motorStep > 6) {
      motorStep = 1;
    }

    if (motorStep == 1 || motorStep == 3 || motorStep == 5) {
      motorRisingBEMF = 1;
    } else {
      motorRisingBEMF = 0;
    }
  } else {
    if (--motorStep < 1) {
      motorStep = 6;
    }

    if (motorStep == 1 || motorStep == 3 || motorStep == 5) {
      motorRisingBEMF = 0;
    } else {
      motorRisingBEMF = 1;
    }
  }

  if (input > DSHOT_CMD_MAX) {
    motorCommutationStep(motorStep);
  }
  motorChangeCompInput();
// TIM2->CNT = 0;
// TIM2->ARR = motorCommutationInterval;
}


// for forced commutation -- open loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
  }
}

void motorStart() {
  uint16_t decaystate = motorSlowDecay;
  motorSensorless = false;
  if (!motorRunning) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    motorSlowDecay = 1;

    motorCommutate();
    motorCommutationInterval = motorTimer2StartArr- 3000;
    TIM3->CNT = 0;
    motorRunning = true;
    while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
  }

  motorSlowDecay = decaystate;    // return to normal
  motorSensorless = true;
  motorBemfCounter = 0;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  motorTimestamp = TIM3->CNT;
  //debug
  //LED_ON(RED);

  if (motorCompit > 200) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    return;
  }
  motorCompit +=1;
  while (TIM3->CNT - motorTimestamp < motorFilterDelay);

  if (motorRisingBEMF) {
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
  motorZeroCrossTimestamp = motorTimestamp;
  TIM3->CNT = 0;
  HAL_COMP_Stop_IT(&comparator1Handle);

  motorZeroCounterTimeout = 0;

  // TEST!   divide by two when tracking up down time independant
  motorCommutationInterval = (motorCommutationInterval + motorZeroCrossTimestamp) / 2;

  motorAdvance = motorCommutationInterval / motorAdvanceDivisor;
  motorWaitTime = motorCommutationInterval / 2 - motorAdvance;
  motorBlanktime = motorCommutationInterval / 4;

  if (motorSensorless) {
    while (TIM3->CNT  < motorWaitTime) {
    }

    motorCompit = 0;
    motorCommutate();
    while (TIM3->CNT  < motorWaitTime + motorBlanktime) {
    }
  }

  motorZeroCrossTimestampLast = motorZeroCrossTimestamp;
  motorBemfCounter++;

  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
}


void zc_found_routine() {
  motorZeroCounterTimeout = 0;

  motorZeroCrossTimestamp = TIM3->CNT;

  //ToDo
  /*
  if (motorZeroCrossTimestamp < motorZeroCrossTimestampLast) {
    motorZeroCrossTimestampLast = motorZeroCrossTimestampLast - 65535;
  }*/

  if (motorZeroCrossTimestamp > motorZeroCrossTimestampLast) {
    //if (((motorZeroCrossTimestamp - motorZeroCrossTimestampLast) > (motorCommutationInterval * 2)) || ((motorZeroCrossTimestamp - motorZeroCrossTimestampLast < motorCommutationInterval/2))){
    //  motorCommutationInterval = (motorCommutationInterval * 3 + (motorZeroCrossTimestamp - motorZeroCrossTimestampLast))/4;
    //  motorCommutationInterval = (motorCommutationInterval + (motorZeroCrossTimestamp - motorZeroCrossTimestampLast))/2;
    //}else{
    motorCommutationInterval = (motorZeroCrossTimestamp - motorZeroCrossTimestampLast);       // TEST!   divide by two when tracking up down time independant
    //	}
    motorAdvance = motorCommutationInterval / motorAdvanceDivisor;
    motorWaitTime = motorCommutationInterval /2 - motorAdvance;
  }
  if (motorSensorless) {
    while (TIM3->CNT - motorZeroCrossTimestamp < motorWaitTime) {
    }
    motorCommutate();
    while (TIM3->CNT - motorZeroCrossTimestamp < motorWaitTime + motorBlanktime) {
    }
  }

  motorZeroCrossTimestampLast = motorZeroCrossTimestamp;
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
