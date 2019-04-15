#include "main.h"

extern uint32_t compit;

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
COMP_HandleTypeDef hcomp1;
TIM_HandleTypeDef htim1, htim2, htim3, htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;


// apply full motor motorBrakeFull on stop
extern uint16_t motorBrakeFull;
uint16_t motorStartPower = 150;
uint16_t motorBrakeProportional, motorBrakeProportionalActive;
uint16_t motorBrakeProportionalStrength = 300;

uint32_t sensorless, commutation_interval;


uint32_t filter_level = 1;
uint32_t filter_delay = 2;

uint32_t zctimeout = 0;
// depends on speed of main loop
uint32_t zc_timeout_threshold = 2000;

uint32_t duty_cycle = 100;

uint32_t bemf_counts;

uint8_t motorDirection = 1;
uint8_t rising = 1;
uint8_t running;
uint8_t started;
uint8_t inputArmed;
uint32_t inputArmedCounter;

uint32_t voltageraw;
uint32_t currentraw;
uint32_t ADC1ConvertedValues[2];

//ToDo input
uint8_t dshotCommand;
uint32_t input;
uint32_t inputAdjusted;

extern uint32_t inputDataNew;
extern uint32_t inputTimeout;
extern uint8_t inputProtocol;
extern uint32_t inputTimeoutThreshold;
extern uint32_t inputBufferDMA[64];
extern uint32_t inputBufferSize;


//ToDo
void getADCs(){
  voltageraw = ADC1ConvertedValues[0];
  currentraw = ADC1ConvertedValues[1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  getADCs();
}





int main(void) {
  HAL_Init();
  systemClockConfig();

  configValidateOrReset();
  configRead();

  ledInit();
  systemDmaInit();
  systemAdcInit();
  systemComparator1Init();
  systemTimer1Init();
  systemTimer2Init();
  systemTimer3Init();
  systemTimer15Init();
  watchdogInit(2000);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);

  // HAL_Delay(500);
  playStartupTune();

  while (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK);
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, 64) != HAL_OK);
  //while (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK);
  while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);



  //ToDo
  // 3D motorDirection and normalSpin  opposed
  // what is normal ?
  motorDirection = escConfig()->motorDirection;

  if(escConfig()->motor3Dmode) {
    inputDataNew = 1001;
    //	motorStartPower = 175;
  }

  // set duty cycle to 50 out of 768 to start.
  TIM1->CCR1 = 1;
  TIM1->CCR2 = 1;
  TIM1->CCR3 = 1;
  TIM1->CCR4 = 800;

  // main loop
  while (true) {
    watchdogFeed();

    //debug
    //LED_OFF(LED0);

    compit = 0;

    //Todo lock for mottor running
    switch (dshotCommand) {
    case DSHOT_CMD_MOTOR_STOP:
      //ToDo
      //input = 0;
      break;
    case DSHOT_CMD_BEACON1:
      playStartupTune();
      break;
    case DSHOT_CMD_BEACON2:
      playInputTune();
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

    if ((escConfig()->motor3Dmode == 1) && ((inputProtocol != PROSHOT) && (inputProtocol != DSHOT))) {
      //char oldbrake = motorBrakeFull;
      if ( inputDataNew > 1100 ) {
        if (motorDirection == escConfig()->motorDirection) {
          inputAdjusted = 0;
          motorBrakeProportionalActive = 1;
          motorDirection = !escConfig()->motorDirection;
          //	HAL_Delay(1);
        }

        if (motorBrakeProportionalActive == 0) {
          inputAdjusted = (inputDataNew - 1050)*3;
        }
      }

      if (inputDataNew < 800) {
        if (motorDirection == (!escConfig()->motorDirection)) {
          motorBrakeProportionalActive = 1;
          inputAdjusted = 0;
          motorDirection = escConfig()->motorDirection;
          //	HAL_Delay(1);
        }

        if (motorBrakeProportionalActive == 0) {
          inputAdjusted = (800 - inputDataNew) * 3;
        }
      }

      if (zctimeout >= zc_timeout_threshold) {
        motorBrakeProportionalActive = 0;
        bemf_counts = 0;
      }

      if (inputDataNew > 800 && inputDataNew < 1100) {
        inputAdjusted = 0;
        motorBrakeProportionalActive = 0;
      }
    } else if(((inputProtocol == PROSHOT) || (inputProtocol == DSHOT) ) && escConfig()->motor3Dmode) {
      if ( inputDataNew > 1097 ) {

        if (motorDirection == escConfig()->motorDirection) {
          motorDirection = !escConfig()->motorDirection;
          bemf_counts =0;
        }
        inputAdjusted = (inputDataNew - 1100) * 2 + 100;
      } if ( inputDataNew <= 1047 &&  inputDataNew > 0) {
        if(motorDirection == (!escConfig()->motorDirection)) {
          bemf_counts =0;
          motorDirection = escConfig()->motorDirection;
        }
        inputAdjusted = (inputDataNew - 90) * 2;
      }
      if ((inputDataNew > 1047 && inputDataNew < 1098 ) || inputDataNew <= 120) {
        inputAdjusted = 0;
      }
    } else {
      inputAdjusted = inputDataNew;
    }

    if (inputAdjusted > 2000) {
      inputAdjusted = 2000;
    }

    if (inputAdjusted - input > 25) {
      input = input + 5;
    } else {
      input = inputAdjusted;
    }

    if (inputAdjusted <= input) {
      input = inputAdjusted;
    }

    advanceDivisor();

    if (inputProtocol == AUTODETECT) {
      HAL_Delay(10);
      inputDetectProtocol();
    }

    if (!inputArmed) {
      if ((inputProtocol != AUTODETECT) && (input == 0)) {
        inputArmedCounter++;
        HAL_Delay(1);
        if (inputArmedCounter > 1000) {
          inputArmed = true;
          //debug
          LED_ON(LED0);
          playInputTune();
        }
      }

      if (input > 1) {
        inputArmedCounter = 0;
      }
    }

    if ((input > 47) && (inputArmed)) {
      motorBrakeProportionalActive = 0;
      started = 1;

      duty_cycle = input / 2 - 10;

      if (bemf_counts < 15) {
        if(duty_cycle < 70) {
          duty_cycle=70;
        }
        if (duty_cycle > 400) {
          duty_cycle=400;
        }
      }

      if (running) {
        if (duty_cycle > 998 ) {                                          // safety!!!
          duty_cycle = 998;
        }
        if (duty_cycle < 44) {
          duty_cycle = 44;
        }

        // set duty cycle to 50 out of 768 to start.
        TIM1->CCR1 = duty_cycle;
        TIM1->CCR2 = duty_cycle;
        TIM1->CCR3 = duty_cycle;
        //	TIM1->CCR4 = duty_cycle;
      }
    }

    inputTimeout++;
    if (inputTimeout > inputTimeoutThreshold ) {
      input = 0;
      inputArmed = false;
      inputArmedCounter = 0;
      //debug
      LED_OFF(LED0);
    }

    if (input <= 47) {
      //	sensorless = 0;
      started = 0;
      if ( !motorBrakeFull && !motorBrakeProportionalActive) {
        allOff();
      }
      duty_cycle = 0;
      if(motorBrakeFull) {
        fullBrake();
        duty_cycle = 0;
        //HAL_COMP_Stop_IT(&hcomp1);
      }

      if(motorBrakeProportional && motorBrakeProportionalActive) {
        //motorBrakeProportionalActive = 1;
        duty_cycle = motorBrakeProportionalStrength;
        proBrake();
      }

      // set duty cycle to 50 out of 768 to start.
      TIM1->CCR1 = duty_cycle;
      TIM1->CCR2 = duty_cycle;
      TIM1->CCR3 = duty_cycle;

      if (commutation_interval > 30000) {
        HAL_COMP_Stop_IT(&hcomp1);
        //motorBrakeProportionalActive = 0;
      }

    }

    if (bemf_counts < 100 || commutation_interval > 10000) {
      filter_delay = 15;
      filter_level = 10;
    } else {
      filter_level = 3;
      filter_delay = 3;
    }

    if(commutation_interval < 200 && duty_cycle > 500) {
      filter_delay = 1;
      filter_level = 0;
    }

    if (started == 1) {
      if (running == 0) {
        //allOff();
        zctimeout = 0;
        // safety on for input testing
        startMotor();
      }
    }

    if (duty_cycle < 300) {
      zc_timeout_threshold = 4000;
    }else{
      zc_timeout_threshold = 2000;
    }

    zctimeout++;                                            // move to started if
    if (zctimeout > zc_timeout_threshold) {
      //motorBrakeProportionalActive = 0;
      sensorless = 0;
      HAL_COMP_Stop_IT(&hcomp1);

      running = 0;
      //		commutation_interval = 0;
      zctimeout = zc_timeout_threshold + 1;
      duty_cycle = 0;
    }
  }
}
