#include "input.h"

uint32_t debugInputBufferDMA0,debugInputBufferDMA1,debugInputBufferDMA2,debugInputBufferDMA3,debugInputBufferDMA4,debugInputBufferDMA5,debugInputBufferDMA6,debugInputBufferDMA7;

uint8_t inputProtocol;
uint32_t inputDataNew;
bool inputDataValid;
uint8_t imputCommandDshot;

bool inputArmed;
uint32_t inputArmCounter;
uint32_t inputTimeoutCounter;

uint32_t propulse[4];
uint32_t inputBufferDMA[16];

extern TIM_HandleTypeDef htim15;
extern bool motorDirection;


void inputArmCheck(void) {
  if (!inputArmed) {
    if ((inputProtocol != AUTODETECT) && (inputDataNew < DSHOT_CMD_MAX) && inputDataValid) {
      inputArmCounter++;
      HAL_Delay(1);
      if (inputArmCounter > INPUT_ARM_COUNTER_THRESHOLD) {
        inputArmed = true;
        //debug
        LED_ON(RED);
        motorInputTune();
      }
    }
  }
}

void inputDisarm(void) {
  inputDataNew = 0;
  inputDataValid = false;
  inputArmed = false;
  inputArmCounter = 0;
  inputTimeoutCounter = 0;
  //ToDo
  inputProtocol = AUTODETECT;
  while (HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1) != HAL_OK);
  TIM15->PSC = 1;
  TIM15->CNT = 0xffff;
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
}

void inputDisarmCheck(void) {
  inputTimeoutCounter++;
  if (inputTimeoutCounter > INPUT_TIMEOUT_COUNTER_THRESHOLD ) {
    inputDisarm();
    //debug
    LED_OFF(RED);
  }
}

void inputDshotCommandRun(void) {
  switch (imputCommandDshot) {
  case DSHOT_CMD_MOTOR_STOP:
    //ToDo
    //inputDataNew = 0;
    break;
  case DSHOT_CMD_BEACON1:
    motorStartupTune();
    break;
  case DSHOT_CMD_BEACON2:
    motorInputTune();
    break;
  case DSHOT_CMD_SETTING_SPIN_DIRECTION_NORMAL:
    escConfig()->motorDirection = 1;
    inputArmed = false;
    break;
  case DSHOT_CMD_SETTING_SPIN_DIRECTION_REVERSED:
    escConfig()->motorDirection = 0;
    inputArmed = false;
    break;
  case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    motorDirection = escConfig()->motorDirection;
    break;
  case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
    motorDirection = !escConfig()->motorDirection;
    break;
  case DSHOT_CMD_SETTING_3D_MODE_OFF:
    escConfig()->motor3Dmode = 0;
    inputArmed = false;
    break;
  case DSHOT_CMD_SETTING_3D_MODE_ON:
    escConfig()->motor3Dmode = 1;
    inputArmed = false;
    break;
  case DSHOT_CMD_SETTING_SAVE:
    configWrite();
    // reset esc, iwdg timeout
    while(true);
  default:
    break;
  }
}

void inputCallbackDMA() {
  switch (inputProtocol) {
     case AUTODETECT:
      inputDetectProtocol();
      break;
    case PROSHOT:
      while (HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1) != HAL_OK);
      inputProshot();
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT) != HAL_OK);
      break;
    case SERVOPWM:
      while (HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1) != HAL_OK);
      inputServoPwm();
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM) != HAL_OK);
      break;
  }

}

void inputDetectProtocol() {
  uint32_t inputPulseWidthBuff;
  uint32_t inputPulseWidthMin = 20000;

  while (HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1) != HAL_OK);

  for (int i = 0; i < (INPUT_BUFFER_DMA_SIZE_AUTODETECT - 1); i++) {
    inputPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];
    if(inputPulseWidthBuff < inputPulseWidthMin) {
      inputPulseWidthMin = inputPulseWidthBuff;
    }
  }

  if ((inputPulseWidthMin > INPUT_PROSHOT_WIDTH_MIN_SYSTICKS ) && (inputPulseWidthMin < INPUT_PROSHOT_WIDTH_MAX_SYSTICKS)) {
    inputProtocol = PROSHOT;
    TIM15->PSC = 1;
    TIM15->CNT = 0xffff;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT) != HAL_OK);
    return;
  }

  if (inputPulseWidthMin > 2000) {
    inputProtocol = SERVOPWM;
    TIM15->PSC = 47;
    TIM15->CNT = 0xffff;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM) != HAL_OK);
    return;
  }

  // default
  if (inputProtocol == AUTODETECT) {
    TIM15->PSC = 1;
    TIM15->CNT = 0xffff;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
  }
}

void inputProshot() {
  uint8_t calcCRC = 0, checkCRC = 0;
  uint16_t telegram = 0;

  //debug
  LED_OFF(GREEN);

  for (int i = 0; i < 4; i++) {
    propulse[i] = ( (inputBufferDMA[i*2 + 1] - inputBufferDMA[i*2]) - 23)/3;
  }

  for (int i = 0; i < 4; i++) {
    calcCRC = calcCRC | ((propulse[0]^propulse[1]^propulse[2]) << i);
    checkCRC = checkCRC | (propulse[3] << i);
  }

  telegram = ((propulse[0] << 7 | propulse[1] << 3 | propulse[2] >> 1));

  if ((calcCRC == checkCRC) && (telegram >= INPUT_VALUE_MIN) && (telegram <= INPUT_VALUE_MAX)) {
    inputDataValid = true;
    inputTimeoutCounter = 0;
    inputDataNew = telegram;
    //debug
    LED_ON(GREEN);
    return;
  } else {
    inputDataValid = false;
    return;
  }
}

void inputServoPwm() {
  uint32_t inputPulseWidthBuff;
  uint32_t inputPulseWidthMin = 20000;

  for (int i = 0; i < (INPUT_BUFFER_DMA_SIZE_PWM - 1); i++) {
    inputPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];
    if(inputPulseWidthBuff < inputPulseWidthMin) {
      inputPulseWidthMin = inputPulseWidthBuff;
    }
  }

  if ((inputPulseWidthMin > INPUT_PWM_WIDTH_MIN_US ) && (inputPulseWidthMin < INPUT_PWM_WIDTH_MAX_US)) {
    inputDataValid = true;
    inputTimeoutCounter = 0;
    inputDataNew = map(inputPulseWidthMin, INPUT_PWM_WIDTH_MIN_US, INPUT_PWM_WIDTH_MAX_US, INPUT_VALUE_MIN, INPUT_VALUE_MAX);
    //debug
    LED_ON(GREEN);
    return;
  } else {
    inputDataValid = false;
    return;
  }
}
