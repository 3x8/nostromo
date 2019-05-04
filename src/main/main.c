#include "main.h"

extern COMP_HandleTypeDef comparator1Handle;

TIM_HandleTypeDef timer1Handle, timer2Handle, timer3Handle, timer15Handle;
DMA_HandleTypeDef timer15Channel1DmaHandle;

//ToDo motor
extern bool motorStartup, motorRunning, motorSensorless;
extern bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;

extern uint16_t motorStep, motorAdvanceDivisor;
extern uint32_t motorCommutationInterval;
extern uint32_t motorFilterLevel, motorFilterDelay;
extern uint32_t motorDutyCycle, motorBemfCounter;
extern uint32_t motorZeroCounterTimeout, motorZeroCounterTimeoutThreshold;


//ToDo input
extern uint32_t inputNormed, outputPwm;
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

  motorStartupTune();

  while (HAL_TIM_OC_Start_IT(&timer1Handle, TIM_CHANNEL_4) != HAL_OK);
  while (HAL_TIM_IC_Start_DMA(&timer15Handle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT) != HAL_OK);
  //adcInit();
  while (HAL_COMP_Start_IT(&comparator1Handle) != HAL_OK);


  //ToDo init
  motorDirection = escConfig()->motorDirection;
  motorSlowDecay = escConfig()->motorSlowDecay;
  // start with brake on
  inputDataValid = true;
  inputData = 0;
  outputPwm = 0;

  // main loop
  while (true) {
    watchdogFeed();

    // brake
    if (!outputPwm) {
      switch(escConfig()->motorBrake) {
        case BRAKE_FULL:
          motorBrakeFull();
          motorDutyCycle = 0;
          break;
        case BRAKE_PROPORTIONAL:
          if(motorBrakeActiveProportional) {
            motorDutyCycle = escConfig()->motorBrakeStrength;
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
    }

    if (inputProtocol == AUTODETECT) {
      // noop
    } else {
      inputArmCheck();
      inputDisarmCheck();
      if (inputArmed) {
        motorAdvanceDivisorCalculate();

        if ((inputProtocol == PROSHOT) && (inputDataValid)) {
          if (inputData <= DSHOT_CMD_MAX) {
            motorStartup = false;
            outputPwm = 0;
            if (!motorRunning) {
              inputDshotCommandRun();
            }
          } else {
            motorStartup = true;
            motorBrakeActiveProportional = false;
            inputNormed = constrain((inputData - DSHOT_CMD_MAX), INPUT_NORMED_MIN, INPUT_NORMED_MAX);

            if (escConfig()->motor3Dmode) {
              // up
              if (inputNormed >= escConfig()->input3DdeadbandHigh) {
                if (motorDirection == !escConfig()->motorDirection) {
                  motorDirection = escConfig()->motorDirection;
                  motorBemfCounter = 0;
                }
                outputPwm = (inputNormed - escConfig()->input3Dneutral) + escConfig()->motorStartThreshold;
              }
              // down
              if (inputNormed <= escConfig()->input3DdeadbandLow) {
                if(motorDirection == escConfig()->motorDirection) {
                  motorDirection = !escConfig()->motorDirection;
                  motorBemfCounter = 0;
                }
                outputPwm = inputNormed + escConfig()->motorStartThreshold;
              }
              // deadband
              if ((inputNormed > escConfig()->input3DdeadbandLow) && (inputNormed < escConfig()->input3DdeadbandHigh)) {
                outputPwm = 0;
              }
            } else {
              outputPwm = (inputNormed >> 1) + (escConfig()->motorStartThreshold);
            }

            outputPwm = constrain(outputPwm, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
          }
        } //PROSHOT

        //ToDo PWM input for thrust tests
        if ((inputProtocol == SERVOPWM)  && (inputDataValid)) {
          if (inputData  < DSHOT_CMD_MAX) {
            motorStartup = false;
            outputPwm = 0;
          } else {
            motorStartup = true;
            motorBrakeActiveProportional = false;
            outputPwm = constrain(inputData, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
          }
        } // SERVOPWM

        //ToDo filter too quick changes (motor desync ?)
        if (ABS(outputPwm - motorDutyCycle) > 10) {
          if (outputPwm > motorDutyCycle) {
            motorDutyCycle = motorDutyCycle + 10;
          } else {
            motorDutyCycle = motorDutyCycle - 10;
          }
        } else {
          motorDutyCycle = outputPwm;
        }

        motorDutyCycle = constrain(motorDutyCycle, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
        TIM1->CCR1 = motorDutyCycle;
        TIM1->CCR2 = motorDutyCycle;
        TIM1->CCR3 = motorDutyCycle;

        // comparator filtering params
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

        // timeouts
        if (motorDutyCycle < 300) {
          motorZeroCounterTimeoutThreshold = 4000;
        } else {
          motorZeroCounterTimeoutThreshold = 2000;
        }

        if (++motorZeroCounterTimeout > motorZeroCounterTimeoutThreshold) {
          motorSensorless = false;
          motorRunning = false;
          motorDutyCycle = 0;
          motorZeroCounterTimeout = motorZeroCounterTimeoutThreshold + 1;
        }

        // motor start
        if ((motorStartup) && (!motorRunning)) {
          motorZeroCounterTimeout = 0;
          motorStart();
        }

      } //inputArmed
    } //inputProtocol detected
  } //main loop

} //main
