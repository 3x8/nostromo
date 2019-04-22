#include "main.h"

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
COMP_HandleTypeDef hcomp1;
TIM_HandleTypeDef htim1, htim2, htim3, htim15;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;


//ToDo rest
extern uint32_t compit;
uint32_t sensorless, commutation_interval;
uint32_t filter_level = 1;
uint32_t filter_delay = 2;
uint32_t zctimeout = 0;
// depends on speed of main loop
uint32_t zc_timeout_threshold = 2000;
uint32_t dutyCycle = 100;
uint32_t bemf_counts;

//ToDo motor
extern bool motorBrakeActiveProportional;
extern bool motorDirection;
extern bool motorRunning;
bool motorStartup;

//ToDo input
uint32_t input;
uint32_t inputAdjusted;
extern bool inputArmed;
extern uint32_t inputData;
extern uint8_t  inputProtocol;
extern uint32_t inputBufferDMA[16];


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
  motorStartupTune();

  while (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK);
  while (HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
  adcInit();
  while (HAL_COMP_Start_IT(&hcomp1) != HAL_OK);


  //ToDo
  // 3D motorDirection and normalSpin  opposed
  // what is normal ?
  motorDirection = escConfig()->motorDirection;


  // set duty cycle to 50 out of 768 to start.
  TIM1->CCR1 = 1;
  TIM1->CCR2 = 1;
  TIM1->CCR3 = 1;
  TIM1->CCR4 = 800;

  // main loop
  while (true) {
    watchdogFeed();

    //debug
    //LED_OFF(RED);

    if (inputProtocol == AUTODETECT) {
      HAL_Delay(20);
    } else {
      inputArmCheck();
      if (inputArmed) {
        inputDisarmCheck();

        advanceDivisor();
        compit = 0;


        if (inputData <= DSHOT_CMD_MAX) {
          motorStartup = false;

          if (!motorRunning) {
            inputDshotCommandRun();
          }

          switch(escConfig()->motorBrakeState) {
            case BRAKE_FULL:
              motorBrakeFull();
              dutyCycle = 0;
              break;
            case BRAKE_PROPORTIONAL:
              if(motorBrakeActiveProportional) {
                dutyCycle = escConfig()->motorBrakeProportionalStrength;
                motorBrakeProportional();
              }
              break;
            case BRAKE_OFF:
                motorBrakeOff();
                dutyCycle = 0;
              break;
          }

          TIM1->CCR1 = dutyCycle;
          TIM1->CCR2 = dutyCycle;
          TIM1->CCR3 = dutyCycle;

          if (commutation_interval > 30000) {
            //HAL_COMP_Stop_IT(&hcomp1);
          }
        }




         else {

          if (escConfig()->motor3Dmode) {
            /*
            if(escConfig()->motor3Dmode) {
              inputData = 1001;
            }*/

            if ((inputProtocol != PROSHOT) && (inputProtocol != DSHOT)) {
              if ( inputData > 1100 ) {
                if (motorDirection == escConfig()->motorDirection) {
                  inputAdjusted = 0;
                  motorBrakeActiveProportional = true;
                  motorDirection = !escConfig()->motorDirection;
                }

                if (!motorBrakeActiveProportional) {
                  inputAdjusted = (inputData - 1050)*3;
                }
              }

              if (inputData < 800) {
                if (motorDirection == (!escConfig()->motorDirection)) {
                  motorBrakeActiveProportional = true;
                  inputAdjusted = 0;
                  motorDirection = escConfig()->motorDirection;
                  //	HAL_Delay(1);
                }

                if (!motorBrakeActiveProportional) {
                  inputAdjusted = (800 - inputData) * 3;
                }
              }

              if (zctimeout >= zc_timeout_threshold) {
                motorBrakeActiveProportional = false;
                bemf_counts = 0;
              }

              if ((inputData > 800) && (inputData < 1100)) {
                inputAdjusted = 0;
                motorBrakeActiveProportional = false;
              }
            }

            if ((inputProtocol == PROSHOT) || (inputProtocol == DSHOT)) {
              if (inputData > 1097) {
                if (motorDirection == escConfig()->motorDirection) {
                  motorDirection = !escConfig()->motorDirection;
                  bemf_counts =0;
               }
                inputAdjusted = (inputData - 1100) * 2 + 100;
              }

              if ((inputData <= 1047) &&  (inputData > 0)) {
               if(motorDirection == (!escConfig()->motorDirection)) {
                 bemf_counts =0;
                 motorDirection = escConfig()->motorDirection;
               }
               inputAdjusted = (inputData - 90) * 2;
              }
              if (((inputData > 1047) && (inputData < 1098)) || (inputData <= 120)) {
                inputAdjusted = 0;
              }
            }
          } else {
            inputAdjusted = inputData;
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



            motorBrakeActiveProportional = false;
            motorStartup = true;

            dutyCycle = input / 2 - 10;

            if (bemf_counts < 15) {
              if(dutyCycle < 70) {
                dutyCycle = 70;
              }
              if (dutyCycle > 400) {
                dutyCycle = 400;
              }
            }

            if (motorRunning) {
              if (dutyCycle > 998 ) {                                          // safety!!!
                dutyCycle = 998;
              }
              if (dutyCycle < 44) {
                dutyCycle = 44;
              }

              // set duty cycle to 50 out of 768 to start.
              TIM1->CCR1 = dutyCycle;
              TIM1->CCR2 = dutyCycle;
              TIM1->CCR3 = dutyCycle;
              //TIM1->CCR4 = dutyCycle;
            }



        }






        if (bemf_counts < 100 || commutation_interval > 10000) {
          filter_delay = 15;
          filter_level = 10;
        } else {
          filter_level = 3;
          filter_delay = 3;
        }

        if(commutation_interval < 200 && dutyCycle > 500) {
          filter_delay = 1;
          filter_level = 0;
        }

        if (motorStartup) {
          if (!motorRunning) {
            zctimeout = 0;
            // safety on for input testing
            motorStart();
          }
        }

        if (dutyCycle < 300) {
          zc_timeout_threshold = 4000;
        }else{
          zc_timeout_threshold = 2000;
        }

        zctimeout++;                                            // move to motorStartup if
        if (zctimeout > zc_timeout_threshold) {
          sensorless = 0;
          //HAL_COMP_Stop_IT(&hcomp1);

          motorRunning = false;
          //		commutation_interval = 0;
          zctimeout = zc_timeout_threshold + 1;
          dutyCycle = 0;
        }



      }
    }
  }
}
