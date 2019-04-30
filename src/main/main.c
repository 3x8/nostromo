#include "main.h"

extern COMP_HandleTypeDef comparator1Handle;

TIM_HandleTypeDef timer1Handle, timer2Handle, timer3Handle, timer15Handle;
DMA_HandleTypeDef timer15Channel1DmaHandle;

//ToDo motor
extern bool motorStartup, motorRunning, motorSensorless;
extern bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;
extern uint16_t motorStep, motorAdvanceDivisor;
extern uint32_t motorTimer2StartArr;
extern uint32_t motorZeroCounterTimeout, motorZeroCounterTimeoutThreshold;
extern uint32_t motorDutyCycle, motorBemfCounter ;
extern uint32_t motorCommutationInterval;
extern uint32_t motorFilterLevel, motorFilterDelay;

//ToDo input
uint32_t input;
uint32_t inputAdjusted;
extern bool inputArmed, inputDataValid;
extern uint8_t  inputProtocol;
extern uint32_t inputData;
extern uint32_t inputBufferDMA[INPUT_BUFFER_DMA_SIZE];


int main(void) {
  HAL_Init();
  systemClockConfig();

  configValidateOrReset();
  configRead();

  ledInit();

  systemDmaInit();
  //systemAdcInit();
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
  //adcInit();
  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);


  //ToDo
  // init phase
  // 3D motorDirection and normalSpin  opposed
  // what is normal ?
  motorDirection = escConfig()->motorDirection;
  motorSlowDecay = escConfig()->motorSlowDecay;

  motorFilterLevel = 1;
  motorFilterDelay = 2;
  motorDutyCycle = 100;
  //motorTimer2StartArr = 6000;
  motorTimer2StartArr = 100;
  motorZeroCounterTimeoutThreshold  = 2000; // depends on speed of main loop
  motorAdvanceDivisor = 3; // increase divisor to decrease motorAdvance
  motorStep = 1;

  // start with break
  inputDataValid = true;
  inputData = 0;

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

    if ((inputData <= DSHOT_CMD_MAX) && inputDataValid) {
      switch(escConfig()->motorBrakeState) {
        case BRAKE_FULL:
          motorBrakeFull();
          motorDutyCycle = 0;
          break;
        case BRAKE_PROPORTIONAL:
          if(motorBrakeActiveProportional) {
            motorDutyCycle = escConfig()->motorBrakeProportionalStrength;
            motorBrakeProportional();
          }
          break;
        case BRAKE_OFF:
            motorBrakeOff();
            motorDutyCycle = 0;
          break;
      }

      TIM1->CCR1 = motorDutyCycle;
      TIM1->CCR2 = motorDutyCycle;
      TIM1->CCR3 = motorDutyCycle;

      //ToDo where ???
      if (motorCommutationInterval > 30000) {
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
        motorAdvanceDivisorCalculate();

        if (inputData <= DSHOT_CMD_MAX) {
          motorStartup = false;
          if (!motorRunning) {
            inputDshotCommandRun();
          }
        } else {
          motorStartup = true;
          motorBrakeActiveProportional = false;

          if (escConfig()->motor3Dmode) {
            //if(escConfig()->motor3Dmode) {
            //  inputData = 1001;
            //}

            if (inputProtocol == SERVOPWM) {
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

              if (motorZeroCounterTimeout >= motorZeroCounterTimeoutThreshold) {
                motorBrakeActiveProportional = false;
                motorBemfCounter = 0;
              }

              if ((inputData > 800) && (inputData < 1100)) {
                inputAdjusted = 0;
                motorBrakeActiveProportional = false;
              }
            }

            if (inputProtocol == PROSHOT) {
              if (inputData > 1097) {
                if (motorDirection == escConfig()->motorDirection) {
                  motorDirection = !escConfig()->motorDirection;
                  motorBemfCounter =0;
               }
                inputAdjusted = (inputData - 1100) * 2 + 100;
              }

              if ((inputData <= 1047) &&  (inputData > 0)) {
               if(motorDirection == (!escConfig()->motorDirection)) {
                 motorBemfCounter =0;
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

          constrain(inputAdjusted, INPUT_VALUE_MIN, INPUT_VALUE_MAX);


          // filter ???
          if ((inputAdjusted - input) > 25) {
            input = input + 5;
          } else {
            input = inputAdjusted;
          }

          if (inputAdjusted <= input) {
            input = inputAdjusted;
          }


          motorDutyCycle = input >> 1;

          if (motorBemfCounter < 15) {
            constrain(motorDutyCycle, 50, 300);
            //constrain(motorDutyCycle, 40, 400);
          }

          if (motorRunning) {
            constrain(motorDutyCycle, 44, 998);
          }

          TIM1->CCR1 = motorDutyCycle;
          TIM1->CCR2 = motorDutyCycle;
          TIM1->CCR3 = motorDutyCycle;
          //TIM1->CCR4 = motorDutyCycle;

        } //input is setpoint value



        if ((motorBemfCounter < 100) || (motorCommutationInterval > 10000)) {
          motorFilterDelay = 15;
          motorFilterLevel = 10;
        } else {
          motorFilterLevel = 3;
          motorFilterDelay = 3;
        }

        if ((motorCommutationInterval < 200) && (motorDutyCycle > 500)) {
          motorFilterDelay = 1;
          motorFilterLevel = 0;
        }

        if ((motorStartup) && (!motorRunning)) {
          motorZeroCounterTimeout = 0;
          motorStart();
        }

        if (motorDutyCycle < 300) {
          motorZeroCounterTimeoutThreshold = 4000;
        } else {
          motorZeroCounterTimeoutThreshold = 2000;
        }

        // move to motorStartup if
        if (++motorZeroCounterTimeout > motorZeroCounterTimeoutThreshold) {
          motorSensorless = false;
          //HAL_COMP_Stop_IT(&comparator1Handle);

          motorRunning = false;
          //motorCommutationInterval = 0;
          motorZeroCounterTimeout = motorZeroCounterTimeoutThreshold + 1;
          motorDutyCycle = 0;
        }

      } // inputArmed
    } // inputProtocol detected
  } // main loop

}
