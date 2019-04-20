#include "input.h"

uint8_t inputProtocol;
uint32_t inputDataNew;
uint8_t inputDataValid;
uint8_t imputCommandDshot;

uint8_t inputArmed;
uint32_t inputArmCounter;
uint32_t inputTimeoutCounter;

uint32_t propulse[4], dpulse[16];
uint32_t inputBufferDMA[64];

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
  inputArmed = false;
  inputArmCounter = 0;

  inputDataNew = 0;
  inputTimeoutCounter = 0;
}

void inputDisarmCheck(void) {
  if (inputArmed) {
    inputTimeoutCounter++;
    if (inputTimeoutCounter > INPUT_TIMEOUT_COUNTER_THRESHOLD ) {
      inputDisarm();
      //debug
      LED_OFF(RED);
    }
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
      //debug
      LED_ON(GREEN);
      inputProshot();
      LED_OFF(GREEN);
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT) != HAL_OK);
      break;
    case SERVOPWM:
      //debug
      LED_ON(BLUE);
      inputServoPwm();
      LED_OFF(BLUE);
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM) != HAL_OK);
      break;
  }

}

void inputDetectProtocol() {
  uint32_t inputPulseWidthBuff;
  uint32_t inputPulseWidthMin = 20000;

  for (int i = 0; i < (INPUT_BUFFER_DMA_SIZE_AUTODETECT - 1); i++) {
    inputPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];
    if(inputPulseWidthBuff < inputPulseWidthMin) {
      inputPulseWidthMin = inputPulseWidthBuff;
    }
  }

  if ((inputPulseWidthMin > INPUT_PROSHOT_WIDTH_MIN_SYSTICKS ) && (inputPulseWidthMin < INPUT_PROSHOT_WIDTH_MAX_SYSTICKS)) {
    inputProtocol = PROSHOT;
    TIM15->PSC = 1;
    TIM15->CNT = 0x0;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT) != HAL_OK);
    return;
  }

  if (inputPulseWidthMin > 2000) {
    inputProtocol = SERVOPWM;
    TIM15->PSC = 47;
    TIM15->CNT = 0x0;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM) != HAL_OK);
    return;
  }

  // default
  if (inputProtocol == AUTODETECT) {
    TIM15->PSC = 1;
    TIM15->CNT = 0x0;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
  }
}

void inputProshot() {
  uint8_t calcCRC, checkCRC;
  uint32_t lastnumber = inputBufferDMA[0];

  for (int j = 1; j < 9; j++) {
    if (((inputBufferDMA[j] - lastnumber) > 1500) && ((inputBufferDMA[j] - lastnumber) < 50000)) { // blank space
      if ((inputBufferDMA[j+7] - inputBufferDMA[j]) < 10000) {

        for (int i = 0; i < 4; i++) {
          propulse[i] = (((inputBufferDMA[j + i*2 + 1] - inputBufferDMA[j + i*2])) - 23)/3;
        }

        calcCRC = ((propulse[0]^propulse[1]^propulse[2]) << 3
                   | (propulse[0]^propulse[1]^propulse[2]) << 2
                   | (propulse[0]^propulse[1]^propulse[2]) << 1
                   | (propulse[0]^propulse[1]^propulse[2]) << 0);
        checkCRC = (propulse[3] << 3 | propulse[3] << 2 | propulse[3] << 1 | propulse[3] << 0);
      }

      if (calcCRC == checkCRC) {
        //debug
        //LED_ON(BLUE);

        inputTimeoutCounter = 0;

        int tocheck = ((propulse[0] << 7 | propulse[1] << 3 | propulse[2] >> 1));
        if (tocheck > 2047 || tocheck < 0) {
          break;
        } else {
          if(tocheck > 47) {
            inputDataNew = tocheck;
            imputCommandDshot = 0;
          }

          if ((tocheck <= 47)&& (tocheck > 0)) {
            inputDataNew = 0;
            imputCommandDshot = tocheck;  //  todo
          }

          if (tocheck == 0) {
            inputDataNew = 0;
            imputCommandDshot = 0;
          }
        }
      }
      break;
    }

    lastnumber = inputBufferDMA[j];
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
    return;
  } else {
    inputDataValid = false;
    return;
  }
}
