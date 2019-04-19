#include "input.h"

uint32_t inputPulseWidthMin = 20000;
uint32_t timeTest0,timeTest1,timeTest2,timeTest3,timeTest4,timeTest5,timeTest6,timeTest7;

uint8_t inputProtocol;
uint32_t inputDataNew;
uint8_t imputCommandDshot;

bool inputArmed;
uint32_t inputArmCounter;
uint32_t inputTimeoutCounter;

uint32_t propulse[4], dpulse[16];
uint32_t inputBufferDMA[64];
uint32_t inputBufferSize = 64;

extern TIM_HandleTypeDef htim15;
extern bool motorDirection;


void inputArmCheck(void) {
  if (!inputArmed) {
    if ((inputProtocol != AUTODETECT) && (inputDataNew == 0)) {
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

  //inputDataNew = 0;
  inputTimeoutCounter = 0;

  inputProtocol = AUTODETECT;
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 16) != HAL_OK);
}

void inputDisarmCheck(void) {
  //if (inputArmed) {
    inputTimeoutCounter++;
    if (inputTimeoutCounter > INPUT_TIMEOUT_COUNTER_THRESHOLD ) {
      inputDisarm();
      //debug
      LED_OFF(RED);
    }

  //}

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
      //debug
      LED_ON(BLUE);
      inputDetectProtocol();
      LED_OFF(BLUE);
      break;
    case PROSHOT:
      //debug
      LED_ON(GREEN);
      inputProshot();
      LED_OFF(GREEN);
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 16) != HAL_OK);
      break;
    case SERVOPWM:
      inputServoPwm();
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 2) != HAL_OK);
      break;
    default:
      while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA,16) != HAL_OK);
      break;
  }

}


void inputDetectProtocol() {
  inputPulseWidthMin = 20000;

  //ToDo
  inputBufferSize = 8;
  uint32_t inputPulseWidthBuff;
  //uint32_t inputPulseWidthMin = 20000;

  timeTest0 = inputBufferDMA[1] - inputBufferDMA[0];
  timeTest1 = inputBufferDMA[2] - inputBufferDMA[1];
  timeTest2 = inputBufferDMA[3] - inputBufferDMA[2];
  timeTest3 = inputBufferDMA[4] - inputBufferDMA[3];
  timeTest4 = inputBufferDMA[5] - inputBufferDMA[4];
  timeTest5 = inputBufferDMA[6] - inputBufferDMA[5];
  timeTest6 = inputBufferDMA[7] - inputBufferDMA[6];

  for (int i = 0; i < inputBufferSize; i++) {
    inputPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];
    if(inputPulseWidthBuff < inputPulseWidthMin) {
      inputPulseWidthMin = inputPulseWidthBuff;
    }
  }

  memset(inputBufferDMA, 0, sizeof(inputBufferDMA));

  if ((inputPulseWidthMin > INPUT_PROSHOT_WIDTH_MIN_SYSTICKS ) && (inputPulseWidthMin < INPUT_PROSHOT_WIDTH_MAX_SYSTICKS)) {
    //inputProtocol = PROSHOT;
    TIM15->PSC = 1;
    //TIM15->CNT = 0xffff;
    TIM15->CNT = 0x0;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 8) != HAL_OK);
    return;
  }

  if (inputPulseWidthMin > 2000) {
    //inputProtocol = SERVOPWM;
    //TIM15->PSC = 1;
    TIM15->PSC = 47;
    //TIM15->CNT = 0xffff;
    TIM15->CNT = 0x0;
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 2) != HAL_OK);
    return;
  }

  // default
  if (inputProtocol == AUTODETECT) {
    while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 8) != HAL_OK);
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

void inputMultishot() {
  int lastnumber = inputBufferDMA[0];

  for (int j = 1; j < 2; j++) {
    // blank space
    if(((inputBufferDMA[j] - lastnumber) < 1500) && ((inputBufferDMA[j] - lastnumber) > 0)) {
      inputDataNew = map((inputBufferDMA[j] - lastnumber),243,1200, 0, 2000);
      break;
    }
    lastnumber = inputBufferDMA[j];
  }
}

void inputOneshot125() {
  int lastnumber = inputBufferDMA[0];

  for (int j = 1; j < 2; j++) {
    // blank space
    if(((inputBufferDMA[j] - lastnumber) < 12300) && ((inputBufferDMA[j] - lastnumber) > 0)) {
      inputDataNew = map((inputBufferDMA[j] - lastnumber),6500,12000, 0, 2000);
      break;
    }
    lastnumber = inputBufferDMA[j];
  }
}

void inputOneshot42() {
  int lastnumber = inputBufferDMA[0];
  for (int j = 1; j < 2; j++) {
    // blank space
    if(((inputBufferDMA[j] - lastnumber) < 4500) && ((inputBufferDMA[j] - lastnumber) > 0)) {
      inputDataNew = map((inputBufferDMA[j] - lastnumber),2020, 4032, 0, 2000);
      break;
    }
    lastnumber = inputBufferDMA[j];
  }
}

void inputServoPwm() {
  int lastnumber = inputBufferDMA[0];
  for (int j = 1; j < 3; j++) {
    // blank space
    if(((inputBufferDMA[j] - lastnumber) >1000 ) && ((inputBufferDMA[j] - lastnumber) < 2010)) {
      inputDataNew = map((inputBufferDMA[j] - lastnumber), 1090, 2000, 0, 2000);
      inputTimeoutCounter = 0;
      //break;
      return;
    }
    lastnumber = inputBufferDMA[j];
  }
}


void inputDshot() {
  uint8_t calcCRC, checkCRC;
  int lastnumber = inputBufferDMA[0];

  for (int j = 1; j < inputBufferSize; j++) {
    // blank space
    if (((inputBufferDMA[j] - lastnumber) > 50) && ((inputBufferDMA[j] - lastnumber) < 65000)) {
      for (int i = 0; i < 16; i++) {
        dpulse[i] = ((inputBufferDMA[j + i*2 +1] - inputBufferDMA[j + i*2]) / 13) - 1;
      }

      calcCRC = ( (dpulse[0]^dpulse[4]^dpulse[8]) << 3
                  |(dpulse[1]^dpulse[5]^dpulse[9]) << 2
                  |(dpulse[2]^dpulse[6]^dpulse[10]) << 1
                  |(dpulse[3]^dpulse[7]^dpulse[11])
                  );
      checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);

      int tocheck = (
        dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7
          | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3
          | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);

      if(calcCRC == checkCRC) {
        if (tocheck > 47) {
          inputDataNew = tocheck;
          imputCommandDshot = 0;
        }
      }
      if ((tocheck <= 47) && (tocheck > 0)) {
        inputDataNew = 0;
        imputCommandDshot = tocheck;    // todo
      }
      if (tocheck == 0) {
        inputDataNew = 0;
        imputCommandDshot = 0;
      }

      break;
    }
    lastnumber = inputBufferDMA[j];
  }
}
