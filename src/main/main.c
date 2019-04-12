#include "main.h"

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

// increase divisor to decrease advance
uint16_t advancedivisor = 8;
//char advancedivisorup = 3;
//char advancedivisordown = 3;

uint32_t thiszctime, lastzctime, sensorless, commutation_interval;

uint32_t blanktime, waitTime, compit;
uint32_t filter_level = 1;
uint32_t filter_delay = 2;

uint32_t zctimeout = 0;
// depends on speed of main loop
uint32_t zc_timeout_threshold = 2000;
uint32_t tim2_start_arr = 9000;
uint32_t duty_cycle = 100;

//ToDo enum
uint8_t pwm = 1;
uint8_t floating = 2;
uint8_t lowside = 3;

uint32_t bemf_counts;

uint8_t forward = 1;
uint8_t rising = 1;
uint8_t running;
uint8_t started;
uint8_t armed;
uint32_t armedcount;

uint32_t voltageraw;
uint32_t currentraw;
uint32_t ADC1ConvertedValues[2];

//ToDo input
uint8_t dshotcommand, inputSet;
uint8_t calcCRC, checkCRC;
uint8_t dshot, proshot, multishot, oneshot42, oneshot125, servoPwm;

uint32_t input_buffer_size = 64;

uint32_t dma_buffer[64];
uint32_t propulse[4];
uint32_t dpulse[16];

uint32_t input, newinput;

uint32_t signaltimeout;
uint32_t signal_timeout_threshold = 10000;

uint32_t adjusted_input;


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
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer, 64) != HAL_OK);
  //while (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK);
  while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);



  //ToDo
  // 3D spinDirection and normalSpin  opposed
  // what is normal ?
  forward = escConfig()->spinDirection;

  if(escConfig()->mode3D) {
    newinput = 1001;
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
    LED_OFF(LED0);

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
      escConfig()->spinDirection = 0;
      armed = 0;
      break;
    case DSHOT_CMD_SETTING_SPIN_DIRECTION_REVERSED:
      escConfig()->spinDirection = 1;
      armed = 0;
      break;
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
      forward = escConfig()->spinDirection;
      break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
      forward = !escConfig()->spinDirection;
      break;
    case DSHOT_CMD_SETTING_3D_MODE_OFF:
      escConfig()->mode3D = 0;
      armed = 0;
      break;
    case DSHOT_CMD_SETTING_3D_MODE_ON:
      escConfig()->mode3D = 1;
      armed = 0;
      break;
    case DSHOT_CMD_SETTING_SAVE:
      configWrite();
      // reset esc, iwdg timeout
      while(true);
    default:
      break;
    }





    if (escConfig()->mode3D == 1 && (proshot == 0 && dshot == 0)) {
      //char oldbrake = brake;
      if ( newinput > 1100 ) {
        if (forward == escConfig()->spinDirection) {
          adjusted_input = 0;
          prop_brake_active = 1;
          forward = 1 - escConfig()->spinDirection;
          //	HAL_Delay(1);
        }

        if (prop_brake_active == 0) {
          adjusted_input = (newinput - 1050)*3;
        }
      }

      if (newinput < 800) {
        if (forward == (1 - escConfig()->spinDirection)) {
          prop_brake_active = 1;
          adjusted_input = 0;
          forward = escConfig()->spinDirection;
          //	HAL_Delay(1);
        }

        if (prop_brake_active == 0) {
          adjusted_input = (800 - newinput) * 3;
        }
      }

      if (zctimeout >= zc_timeout_threshold) {
        prop_brake_active = 0;
        bemf_counts = 0;
      }

      if (newinput > 800 && newinput < 1100) {
        adjusted_input = 0;
        prop_brake_active = 0;
      }

    } else if((proshot || dshot ) && escConfig()->mode3D) {
      if ( newinput > 1097 ) {

        if (forward == escConfig()->spinDirection) {
          forward = 1 - escConfig()->spinDirection;
          bemf_counts =0;
        }
        adjusted_input = (newinput - 1100) * 2 + 100;
      } if ( newinput <= 1047 &&  newinput > 0) {
        if(forward == (1 - escConfig()->spinDirection)) {
          bemf_counts =0;
          forward = escConfig()->spinDirection;
        }
        adjusted_input = (newinput - 90) * 2;
      }
      if ((newinput > 1047 && newinput < 1098 ) || newinput <= 120) {
        adjusted_input = 0;
      }
    } else {
      adjusted_input = newinput;
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


    advancedivisor = map((commutation_interval),100,5000, 2, 20);

    if (inputSet == 0) {
      HAL_Delay(10);
      detectInput();
    }

    if (!armed) {
      if ((inputSet == 1) && (input == 0)) {
        armedcount++;
        HAL_Delay(1);
        if (armedcount > 1000) {
          armed = 1;
          playInputTune();
        }
      }

      if (input > 1) {
        armedcount = 0;
      }
    }

    if ((input > 47) && (armed == 1)) {
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

    signaltimeout++;
    if (signaltimeout > signal_timeout_threshold ) {
      input = 0;
      armed = 0;
      armedcount = 0;
      //	  duty_cycle = 0;          //mid point
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
