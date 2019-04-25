#include "motor.h"

COMP_HandleTypeDef comparator1Handle;

uint32_t timestamp;
uint16_t step = 1;

uint32_t thiszctime, lastzctime;

// set proportianal to commutation time. with advance divisor
uint32_t advance = 0;

// increase divisor to decrease advance
uint16_t advancedivisor = 3;

uint32_t blanktime, waitTime, compit;

uint32_t tim2_start_arr = 9000;


extern TIM_HandleTypeDef timer1Handle;

bool motorSensorless;
uint32_t commutationInterval;
uint32_t motorFilterLevel = 1;
uint32_t motorFilterDelay = 2;

extern uint32_t zctimeout;
// depends on speed of main loop
extern uint32_t zeroCounterTimeoutThreshold;
extern uint32_t dutyCycle;

extern uint32_t bemfCounter;

bool motorDirection = 1;
bool motorRisingBEMF = 1;
bool motorRunning;

extern bool inputArmed;
extern uint32_t inputArmCounter;


// 1 for complementary HBRIDGE_PWM , 0 for diode freewheeling
bool motorSlowDecay = true;
bool motorBrakeActiveProportional = true;


extern uint32_t input;

void advanceDivisor() {
    advancedivisor = map((commutationInterval),100,5000, 2, 20);
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
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(B_FET_LO_GPIO, B_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_LO_GPIO->BSRR = B_FET_LO_PIN;
    LL_GPIO_SetPinMode(B_FET_HI_GPIO, B_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    B_FET_HI_GPIO->BRR = B_FET_HI_PIN;
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
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(C_FET_LO_GPIO, C_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_LO_GPIO->BSRR = C_FET_LO_PIN;
    LL_GPIO_SetPinMode(C_FET_HI_GPIO, C_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    C_FET_HI_GPIO->BRR = C_FET_HI_PIN;
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
  }

  if (newPhase == HBRIDGE_FLOATING) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
  }

  if (newPhase == HBRIDGE_LOWSIDE) {
    LL_GPIO_SetPinMode(A_FET_LO_GPIO, A_FET_LO_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_LO_GPIO->BSRR = A_FET_LO_PIN;
    LL_GPIO_SetPinMode(A_FET_HI_GPIO, A_FET_HI_PIN, LL_GPIO_MODE_OUTPUT);
    A_FET_HI_GPIO->BRR = A_FET_HI_PIN;
  }

}

void motorCommutationStep(uint8_t newStep) {
  //A-B
  if (newStep == 1) {
    motorPhaseA(HBRIDGE_PWM);
    motorPhaseB(HBRIDGE_LOWSIDE);
    motorPhaseC(HBRIDGE_FLOATING);
  }
  // C-B
  if (newStep == 2) {
    motorPhaseA(HBRIDGE_FLOATING);
    motorPhaseB(HBRIDGE_LOWSIDE);
    motorPhaseC(HBRIDGE_PWM);
  }
  // C-A
  if (newStep == 3) {
    motorPhaseA(HBRIDGE_LOWSIDE);
    motorPhaseB(HBRIDGE_FLOATING);
    motorPhaseC(HBRIDGE_PWM);
  }
  // B-A
  if (newStep == 4) {
    motorPhaseA(HBRIDGE_LOWSIDE);
    motorPhaseB(HBRIDGE_PWM);
    motorPhaseC(HBRIDGE_FLOATING);
  }
  // B-C
  if (newStep == 5) {
    motorPhaseA(HBRIDGE_FLOATING);
    motorPhaseB(HBRIDGE_PWM);
    motorPhaseC(HBRIDGE_LOWSIDE);
  }
  // A-C
  if (newStep == 6) {
    motorPhaseA(HBRIDGE_PWM);
    motorPhaseB(HBRIDGE_FLOATING);
    motorPhaseC(HBRIDGE_LOWSIDE);
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

// duty cycle controls braking strength
// will turn off lower fets so only high side is active
void motorBrakeProportional() {
  motorPhaseA(HBRIDGE_PWM);
  motorPhaseB(HBRIDGE_PWM);
  motorPhaseC(HBRIDGE_PWM);
}


void motorChangeCompInput() {
  // c floating
  if (step == 1 || step == 4) {
    comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
  }
  // a floating
  if (step == 2 || step == 5) {
    // if f051k6  step 2 , 5 is dac 1 ( swap comp input)
    #ifdef MP6531
    comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
    #endif
    #ifdef FD6288
    comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
    #endif
  }
  // b floating
  if (step == 3 || step == 6) {
    #ifdef MP6531
    comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
    #endif
    #ifdef FD6288
    comparator1Handle.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
    #endif
  }
  if (motorRisingBEMF) {
    // polarity of comp output reversed
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  }else{
    // falling bemf
    comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  }

  while (HAL_COMP_Init(&comparator1Handle) != HAL_OK);
}


void motorCommutate() {
  if (motorDirection == 1) {
    step++;
    if (step > 6) {
      step = 1;
    }
    if (step == 1 || step == 3 || step == 5) {
      motorRisingBEMF = 1;                                // is back emf motorRisingBEMF or falling
    }
    if (step == 2 || step == 4 || step == 6) {
      motorRisingBEMF = 0;
    }
  }
  if (motorDirection == 0) {
    step--;
    if (step < 1) {
      step = 6;
    }
    if (step == 1 || step == 3 || step == 5) {
      motorRisingBEMF = 0;
    }
    if (step == 2 || step == 4 || step == 6) {
      motorRisingBEMF = 1;
    }
  }

  if (input > 47) {
    motorCommutationStep(step);
  }
  motorChangeCompInput();
// TIM2->CNT = 0;
// TIM2->ARR = commutationInterval;
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
    commutationInterval = tim2_start_arr- 3000;
    TIM3->CNT = 0;
    motorRunning = true;
    while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
  }

  motorSlowDecay = decaystate;    // return to normal
  motorSensorless = true;
  bemfCounter = 0;
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  timestamp = TIM3->CNT;
  //debug
  //LED_ON(RED);

  if (compit > 200) {
    HAL_COMP_Stop_IT(&comparator1Handle);
    return;
  }
  compit +=1;
  while (TIM3->CNT - timestamp < motorFilterDelay);

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
  thiszctime = timestamp;
  TIM3->CNT = 0;
  HAL_COMP_Stop_IT(&comparator1Handle);

  zctimeout = 0;

  // TEST!   divide by two when tracking up down time independant
  commutationInterval = (commutationInterval + thiszctime) / 2;

  advance = commutationInterval / advancedivisor;
  waitTime = commutationInterval / 2 - advance;
  blanktime = commutationInterval / 4;

  if (motorSensorless) {
    while (TIM3->CNT  < waitTime) {
    }

    compit = 0;
    motorCommutate();
    while (TIM3->CNT  < waitTime + blanktime) {
    }
  }

  lastzctime = thiszctime;
  bemfCounter++;

  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);
}


void zc_found_routine() {
  zctimeout = 0;

  thiszctime = TIM3->CNT;

  //ToDo
  /*
  if (thiszctime < lastzctime) {
    lastzctime = lastzctime - 65535;
  }*/

  if (thiszctime > lastzctime) {
    //if (((thiszctime - lastzctime) > (commutationInterval * 2)) || ((thiszctime - lastzctime < commutationInterval/2))){
    //  commutationInterval = (commutationInterval * 3 + (thiszctime - lastzctime))/4;
    //  commutationInterval = (commutationInterval + (thiszctime - lastzctime))/2;
    //}else{
    commutationInterval = (thiszctime - lastzctime);       // TEST!   divide by two when tracking up down time independant
    //	}
    advance = commutationInterval / advancedivisor;
    waitTime = commutationInterval /2 - advance;
  }
  if (motorSensorless) {
    while (TIM3->CNT - thiszctime < waitTime) {
    }
    motorCommutate();
    while (TIM3->CNT - thiszctime < waitTime + blanktime) {
    }
  }

  lastzctime = thiszctime;
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
