#include "main.h"

extern uint32_t compit;

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
COMP_HandleTypeDef hcomp1;
TIM_HandleTypeDef htim1, htim2, htim3, htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

// 1 for complementary pwm , 0 for diode freewheeling
uint16_t slow_decay = 1;
// apply full motor brake on stop
uint16_t brake = 1;
uint16_t start_power = 150;
uint16_t prop_brake, prop_brake_active;
uint16_t prop_brake_strength = 300;

uint32_t sensorless, commutation_interval;


uint32_t filter_level = 1;
uint32_t filter_delay = 2;

uint32_t zctimeout = 0;
// depends on speed of main loop
uint32_t zc_timeout_threshold = 2000;

uint32_t duty_cycle = 100;

uint32_t bemf_counts;

uint8_t forward = 1;
uint8_t rising = 1;
uint8_t running;
uint8_t started;
uint8_t inputArmed;
uint32_t armedcount;

uint32_t voltageraw;
uint32_t currentraw;
uint32_t ADC1ConvertedValues[2];

//ToDo input
uint8_t dshotcommand;
uint32_t input;
uint32_t adjusted_input;

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
  forward = escConfig()->motorDirection;

  if(escConfig()->motor3Dmode) {
    inputDataNew = 1001;
    //	start_power = 175;
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
    switch (dshotcommand) {
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
      forward = escConfig()->motorDirection;
      break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
      forward = !escConfig()->motorDirection;
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
      //char oldbrake = brake;
      if ( inputDataNew > 1100 ) {
        if (forward == escConfig()->motorDirection) {
          adjusted_input = 0;
          prop_brake_active = 1;
          forward = !escConfig()->motorDirection;
          //	HAL_Delay(1);
        }

        if (prop_brake_active == 0) {
          adjusted_input = (inputDataNew - 1050)*3;
        }
      }

      if (inputDataNew < 800) {
        if (forward == (!escConfig()->motorDirection)) {
          prop_brake_active = 1;
          adjusted_input = 0;
          forward = escConfig()->motorDirection;
          //	HAL_Delay(1);
        }

        if (prop_brake_active == 0) {
          adjusted_input = (800 - inputDataNew) * 3;
        }
      }

      if (zctimeout >= zc_timeout_threshold) {
        prop_brake_active = 0;
        bemf_counts = 0;
      }

      if (inputDataNew > 800 && inputDataNew < 1100) {
        adjusted_input = 0;
        prop_brake_active = 0;
      }
    } else if(((inputProtocol == PROSHOT) || (inputProtocol == DSHOT) ) && escConfig()->motor3Dmode) {
      if ( inputDataNew > 1097 ) {

        if (forward == escConfig()->motorDirection) {
          forward = !escConfig()->motorDirection;
          bemf_counts =0;
        }
        adjusted_input = (inputDataNew - 1100) * 2 + 100;
      } if ( inputDataNew <= 1047 &&  inputDataNew > 0) {
        if(forward == (!escConfig()->motorDirection)) {
          bemf_counts =0;
          forward = escConfig()->motorDirection;
        }
        adjusted_input = (inputDataNew - 90) * 2;
      }
      if ((inputDataNew > 1047 && inputDataNew < 1098 ) || inputDataNew <= 120) {
        adjusted_input = 0;
      }
    } else {
      adjusted_input = inputDataNew;
    }

    if (adjusted_input > 2000) {
      adjusted_input = 2000;
    }

    if (adjusted_input - input > 25) {
      input = input + 5;
    } else {
      input = adjusted_input;
    }

    if (adjusted_input <= input) {
      input = adjusted_input;
    }

    advanceDivisor();

    if (inputProtocol == AUTODETECT) {
      HAL_Delay(10);
      inputDetectProtocol();
    }

    if (!inputArmed) {
      if ((inputProtocol != AUTODETECT) && (input == 0)) {
        armedcount++;
        HAL_Delay(1);
        if (armedcount > 1000) {
          inputArmed = true;
          //debug
          LED_ON(LED0);
          playInputTune();
        }
      }

      if (input > 1) {
        armedcount = 0;
      }
    }

    if ((input > 47) && (inputArmed)) {
      prop_brake_active = 0;
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
      armedcount = 0;
      //debug
      LED_OFF(LED0);
    }

    if (input <= 47) {
      //	sensorless = 0;
      started = 0;
      if ( !brake && !prop_brake_active) {
        allOff();
      }
      duty_cycle = 0;
      if(brake) {
        fullBrake();
        duty_cycle = 0;
        //HAL_COMP_Stop_IT(&hcomp1);
      }

      if(prop_brake && prop_brake_active) {
        //prop_brake_active = 1;
        duty_cycle = prop_brake_strength;
        proBrake();
      }

      // set duty cycle to 50 out of 768 to start.
      TIM1->CCR1 = duty_cycle;
      TIM1->CCR2 = duty_cycle;
      TIM1->CCR3 = duty_cycle;

      if (commutation_interval > 30000) {
        HAL_COMP_Stop_IT(&hcomp1);
        //prop_brake_active = 0;
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
      //prop_brake_active = 0;
      sensorless = 0;
      HAL_COMP_Stop_IT(&hcomp1);

      running = 0;
      //		commutation_interval = 0;
      zctimeout = zc_timeout_threshold + 1;
      duty_cycle = 0;
    }
  }
}
