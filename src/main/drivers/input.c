#include "input.h"

extern uint8_t dshotcommand;

uint8_t inputProtocol;
uint32_t inputTimeout;
uint32_t inputTimeoutThreshold = 10000;
uint32_t inputDataNew;
uint32_t propulse[4], dpulse[16];

uint32_t inputBufferDMA[64];
uint32_t inputBufferSize = 64;


extern TIM_HandleTypeDef htim15;


void inputCallbackDMA() {
  inputTimeout = 0;

  //debug
  LED_ON(LED1);

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);

  switch (inputProtocol) {
    case DSHOT:
      inputDshot();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 64);
      break;
    case PROSHOT:
      inputProshot();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 16);
      //debug
      //HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 8);
      break;
    case SERVOPWM:
      inputServoPwm();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 3);
      break;
    case MULTISHOT:
      inputMultishot();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 3);
      break;
    case ONESHOT125:
      inputOneshot125();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 3);
      break;
    case ONESHOT42:
      inputOneshot42();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 3);
      break;
  }

  //debug
  LED_OFF(LED1);
  LED_OFF(LED2);
}


void inputDetectProtocol() {
  uint32_t smallestnumber = 20000;

  int lastnumber = inputBufferDMA[0];

  for ( int j = 1; j < inputBufferSize; j++) {
    if((inputBufferDMA[j] - lastnumber) < smallestnumber) { // blank space
      smallestnumber = inputBufferDMA[j] - lastnumber;
    }
    lastnumber = inputBufferDMA[j];
  }

  if ((smallestnumber > 3)&&(smallestnumber < 22)) {
    inputProtocol = DSHOT;
  }

  if ((smallestnumber > 40 )&&(smallestnumber < 80)) {
    inputProtocol = PROSHOT;
    TIM15->PSC = 1;
    TIM15->CNT = 0xffff;
  }

  if ((smallestnumber > 100 )&&(smallestnumber < 400)) {
    inputProtocol = MULTISHOT;
  }
  //if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
  //  inputProtocol = ONESHOT42;
  //}
  //if ((smallestnumber > 3000 )&&(smallestnumber < 7000)){
  //  inputProtocol = ONESHOT125;
  //}
  if (smallestnumber > 2000) {
    inputProtocol = SERVOPWM;
    TIM15->PSC = 47;
    TIM15->CNT = 0xffff;
  }

  if (smallestnumber == 0) {
    inputProtocol = AUTODETECT;
  } else {
    HAL_Delay(50);
  }
  HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 64);
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
        LED_ON(LED2);

        int tocheck = ((propulse[0] << 7 | propulse[1] << 3 | propulse[2] >> 1));
        if (tocheck > 2047 || tocheck < 0) {
          break;
        } else {
          if(tocheck > 47) {
            inputDataNew = tocheck;
            dshotcommand = 0;
          }

          if ((tocheck <= 47)&& (tocheck > 0)) {
            inputDataNew = 0;
            dshotcommand = tocheck;  //  todo
          }

          if (tocheck == 0) {
            inputDataNew = 0;
            dshotcommand = 0;
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
      break;
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
          dshotcommand = 0;
        }
      }
      if ((tocheck <= 47) && (tocheck > 0)) {
        inputDataNew = 0;
        dshotcommand = tocheck;    // todo
      }
      if (tocheck == 0) {
        inputDataNew = 0;
        dshotcommand = 0;
      }

      break;
    }
    lastnumber = inputBufferDMA[j];
  }
}
