#include "input.h"

extern uint8_t dshotcommand, inputSet;
extern uint8_t calcCRC, checkCRC;
extern uint8_t dshot, proshot, multishot, oneshot42, oneshot125, servoPwm;

extern uint32_t input_buffer_size;

extern uint32_t dma_buffer[64];
extern uint32_t propulse[4];
extern uint32_t dpulse[16];

extern uint32_t input, newinput;

extern uint32_t signaltimeout;


extern TIM_HandleTypeDef htim15;


void detectInput() {
  uint32_t smallestnumber = 20000;
  dshot = 0;
  proshot = 0;
  multishot = 0;
  oneshot42 = 0;
  oneshot125 = 0;
  servoPwm = 0;
  int lastnumber = dma_buffer[0];

  for ( int j = 1; j < input_buffer_size; j++) {
    if((dma_buffer[j] - lastnumber) < smallestnumber) { // blank space
      smallestnumber = dma_buffer[j] - lastnumber;
    }
    lastnumber = dma_buffer[j];
  }

  if ((smallestnumber > 3)&&(smallestnumber < 22)) {
    dshot = 1;
  }

  if ((smallestnumber > 40 )&&(smallestnumber < 80)) {
    proshot = 1;
    TIM15->PSC = 1;
    TIM15->CNT = 0xffff;
  }

  if ((smallestnumber > 100 )&&(smallestnumber < 400)) {
    multishot = 1;
  }
//	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
//		oneshot42 = 1;
//	}
//	if ((smallestnumber > 3000 )&&(smallestnumber < 7000)){
//		oneshot125 = 1;
//	}
  if (smallestnumber > 2000) {
    servoPwm = 1;
    TIM15->PSC = 47;
    TIM15->CNT = 0xffff;
  }

  if (smallestnumber == 0) {
    inputSet = 0;
  } else {
    inputSet = 1;
    HAL_Delay(50);
    // playInputTune();
  }
  HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64);
}

void computeProshotDMA() {
  uint32_t lastnumber = dma_buffer[0];

  for (int j = 1; j < 9; j++) {
    if (((dma_buffer[j] - lastnumber) > 1500) && ((dma_buffer[j] - lastnumber) < 50000)) { // blank space
      if ((dma_buffer[j+7] - dma_buffer[j]) < 10000) {

        for (int i = 0; i < 4; i++) {
          propulse[i] = (((dma_buffer[j + i*2 + 1] - dma_buffer[j + i*2])) - 23)/3;
        }

        calcCRC = ((propulse[0]^propulse[1]^propulse[2]) << 3
                   | (propulse[0]^propulse[1]^propulse[2]) << 2
                   | (propulse[0]^propulse[1]^propulse[2]) << 1
                   | (propulse[0]^propulse[1]^propulse[2]) << 0);
        checkCRC = (propulse[3] << 3 | propulse[3] << 2 | propulse[3] << 1 | propulse[3] << 0);
      }

      if (calcCRC == checkCRC) {
        int tocheck = ((propulse[0] << 7 | propulse[1] << 3 | propulse[2] >> 1));
        if (tocheck > 2047 || tocheck < 0) {
          break;
        } else {
          if(tocheck > 47) {
            newinput = tocheck;
            dshotcommand = 0;
          }

          if ((tocheck <= 47)&& (tocheck > 0)) {
            newinput = 0;
            dshotcommand = tocheck;  //  todo
          }

          if (tocheck == 0) {
            newinput = 0;
            dshotcommand = 0;
          }
        }
      }
      break;
    }

    lastnumber = dma_buffer[j];
  }
}

void computeMSInput() {
  int lastnumber = dma_buffer[0];

  for (int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),243,1200, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeOS125Input() {
  int lastnumber = dma_buffer[0];

  for (int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 12300) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),6500,12000, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeOS42Input() {
  int lastnumber = dma_buffer[0];
  for (int j = 1; j < 2; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) < 4500) && ((dma_buffer[j] - lastnumber) > 0)) {
      newinput = map((dma_buffer[j] - lastnumber),2020, 4032, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}

void computeServoInput() {
  int lastnumber = dma_buffer[0];
  for (int j = 1; j < 3; j++) {
    // blank space
    if(((dma_buffer[j] - lastnumber) >1000 ) && ((dma_buffer[j] - lastnumber) < 2010)) {
      newinput = map((dma_buffer[j] - lastnumber), 1090, 2000, 0, 2000);
      break;
    }
    lastnumber = dma_buffer[j];
  }
}


void computeDshotDMA() {
  int lastnumber = dma_buffer[0];

  for (int j = 1; j < input_buffer_size; j++) {
    // blank space
    if (((dma_buffer[j] - lastnumber) > 50) && ((dma_buffer[j] - lastnumber) < 65000)) {
      for (int i = 0; i < 16; i++) {
        dpulse[i] = ((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2]) / 13) - 1;
      }

      uint8_t calcCRC = ( (dpulse[0]^dpulse[4]^dpulse[8]) << 3
                          |(dpulse[1]^dpulse[5]^dpulse[9]) << 2
                          |(dpulse[2]^dpulse[6]^dpulse[10]) << 1
                          |(dpulse[3]^dpulse[7]^dpulse[11])
                          );
      uint8_t checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);

      int tocheck = (
        dpulse[0] << 10 | dpulse[1] << 9 | dpulse[2] << 8 | dpulse[3] << 7
          | dpulse[4] << 6 | dpulse[5] << 5 | dpulse[6] << 4 | dpulse[7] << 3
          | dpulse[8] << 2 | dpulse[9] << 1 | dpulse[10]);

      if(calcCRC == checkCRC) {
        if (tocheck > 47) {
          newinput = tocheck;
          dshotcommand = 0;
        }
      }
      if ((tocheck <= 47) && (tocheck > 0)) {
        newinput = 0;
        dshotcommand = tocheck;    // todo
      }
      if (tocheck == 0) {
        newinput = 0;
        dshotcommand = 0;
      }

      break;
    }
    lastnumber = dma_buffer[j];
  }
}


void transferComplete() {
  //debug
  LED_ON(LED1);

  signaltimeout = 0;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);

  if (inputSet == 1) {
    if (dshot == 1) {
      computeDshotDMA();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64);
    }

    if (proshot == 1) {
      computeProshotDMA();
      //debug
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 16);
      //HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 8);
    }

    if (servoPwm == 1) {
      computeServoInput();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }

    if (multishot) {
      computeMSInput();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }

    if (oneshot125) {
      computeOS125Input();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }

    if (oneshot42) {
      computeOS42Input();
      HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 3);
    }
  }

  //debug
  LED_OFF(LED1);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
