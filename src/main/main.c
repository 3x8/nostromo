#include "main.h"


extern COMP_HandleTypeDef comparator1Handle;

TIM_HandleTypeDef timer1Handle, timer2Handle, timer3Handle, timer15Handle;
DMA_HandleTypeDef timer15Channel1DmaHandle;


//ToDo rest
extern uint32_t compit;
uint32_t sensorless, commutation_interval;
uint32_t filterLevel = 1;
uint32_t filter_delay = 2;
uint32_t zctimeout = 0;
// depends on speed of main loop
uint32_t zc_timeout_threshold = 2000;
uint32_t dutyCycle = 100;
uint32_t bemfCounter;

//ToDo motor
extern bool motorBrakeActiveProportional;
extern bool motorDirection;
extern bool motorRunning;
bool motorStartup;

//ToDo input
uint32_t input;
uint32_t inputAdjusted;
extern uint8_t inputArmed;
extern uint32_t inputData;
extern uint8_t inputDataValid;
extern uint8_t  inputProtocol;
extern uint32_t inputBufferDMA[INPUT_BUFFER_DMA_SIZE];


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

  HAL_TIM_PWM_Start(&timer1Handle, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&timer1Handle, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&timer1Handle, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&timer1Handle, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&timer1Handle, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&timer1Handle, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&timer2Handle);
  HAL_TIM_Base_Start(&timer3Handle);

  // HAL_Delay(500);
  motorStartupTune();

  while (HAL_TIM_OC_Start_IT(&timer1Handle, TIM_CHANNEL_4) != HAL_OK);
  while (HAL_TIM_IC_Start_DMA(&timer15Handle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
  adcInit();
  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);


  //ToDo
  // 3D motorDirection and normalSpin  opposed
  // what is normal ?
  motorDirection = escConfig()->motorDirection;


  // set duty cycle to 50 out of 768 to start.
  TIM1->CCR1 = 1;
  TIM1->CCR2 = 1;
  TIM1->CCR3 = 1;
  TIM1->CCR4 = 800;

  // start with break
  inputDataValid = true;
  inputData = 0;

  // main loop
  while (true) {
    watchdogFeed();

    //debug
    //LED_OFF(RED);

    if ((inputData <= DSHOT_CMD_MAX) && inputDataValid) {
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

      //ToDo where ???
      if (commutation_interval > 30000) {
        //HAL_COMP_Stop_IT(&comparator1Handle);
      }
    }



    if (inputProtocol == AUTODETECT) {
      //HAL_Delay(3);
    } else {
      inputArmCheck();
      //ToDo
      inputDisarmCheck();
      if (inputArmed) {

        advanceDivisor();
        compit = 0;

        if (inputData <= DSHOT_CMD_MAX) {
          motorStartup = false;

          if (!motorRunning) {
            inputDshotCommandRun();
          }
        }




        else {
          if (escConfig()->motor3Dmode) {

            //if(escConfig()->motor3Dmode) {
            //  inputData = 1001;
            //}

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
                bemfCounter = 0;
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
                  bemfCounter =0;
               }
                inputAdjusted = (inputData - 1100) * 2 + 100;
              }

              if ((inputData <= 1047) &&  (inputData > 0)) {
               if(motorDirection == (!escConfig()->motorDirection)) {
                 bemfCounter =0;
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

            if (bemfCounter < 15) {
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




        if (bemfCounter < 100 || commutation_interval > 10000) {
          filter_delay = 15;
          filterLevel = 10;
        } else {
          filterLevel = 3;
          filter_delay = 3;
        }

        if(commutation_interval < 200 && dutyCycle > 500) {
          filter_delay = 1;
          filterLevel = 0;
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
          //HAL_COMP_Stop_IT(&comparator1Handle);

          motorRunning = false;
          //commutation_interval = 0;
          zctimeout = zc_timeout_threshold + 1;
          dutyCycle = 0;
        }

      }
    }
  }
}
