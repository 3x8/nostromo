#include "main.h"

extern COMP_HandleTypeDef comparator1Handle;

TIM_HandleTypeDef timer1Handle, timer3Handle, timer15Handle;
DMA_HandleTypeDef timer15Channel1DmaHandle;

// ADC kalman filter
kalman_t adcCurrentFilterState;
extern uint32_t adcVoltageRaw, adcCurrentRaw, adcTemperatureRaw;
extern uint32_t adcVoltage, adcCurrent, adcTemperature;

// motor
extern bool motorStartup, motorRunning;
extern bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;
extern uint32_t motorCommutationInterval;
extern uint32_t motorFilterLevel, motorFilterDelay;
extern uint32_t motorDutyCycle, motorBemfCounter;
extern uint32_t motorZeroCounterTimeout, motorZeroCounterTimeoutThreshold;

// input
extern bool inputArmed, inputDataValid;
extern uint8_t  inputProtocol;
extern uint32_t inputData;
extern uint32_t inputNormed, outputPwm;

int main(void) {
  HAL_Init();
  systemClockConfig();

  configValidateOrReset();
  configRead();

  ledInit();

  systemDmaInit();
  systemComparator1Init();
  systemAdcInit();
  systemTimer1Init();
  systemTimer3Init();
  systemTimer15Init();

  // init
  ledOff();
  kalmanInit(&adcCurrentFilterState, 1500.0f, 31);

  motorDirection = escConfig()->motorDirection;
  motorSlowDecay = escConfig()->motorSlowDecay;
  // start with motor off
  inputData = 0;
  outputPwm = 0;

  watchdogInit(2000);
  motorStartupTune();

  // main loop
  while (true) {

    #if (defined(_DEBUG_) && defined(CYCLETIME_MAINLOOP))
    LED_OFF(BLUE);
    #endif

    watchdogFeed();

    // brake
    if (!outputPwm) {
      switch(escConfig()->motorBrake) {
        case BRAKE_FULL:
          motorBrakeActiveProportional = false;
          motorBrakeFull();
          motorDutyCycle = 0;
          break;
        case BRAKE_PROPORTIONAL:
          motorBrakeActiveProportional = true;
          motorDutyCycle = escConfig()->motorBrakeStrength;
          motorBrakeProportional();
          break;
        case BRAKE_OFF:
          motorBrakeActiveProportional = false;
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
        // PROSHOT
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
              //outputPwm = scaleInputToOutput(inputNormed, INPUT_NORMED_MIN, INPUT_NORMED_MAX, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX) + (escConfig()->motorStartThreshold);
            }

            outputPwm = constrain(outputPwm, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
          }
        } // PROSHOT

        // SERVOPWM (use only for thrust tests ...)
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

        // input
        motorDutyCycle = constrain(outputPwm, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
        TIM1->CCR1 = motorDutyCycle;
        TIM1->CCR2 = motorDutyCycle;
        TIM1->CCR3 = motorDutyCycle;

        // motor BEMF filter
        if ((motorCommutationInterval < 200) && (motorDutyCycle > 500)) {
          motorFilterDelay = 1;
          motorFilterLevel = 0;
        } else {
          motorFilterLevel = 3;
          motorFilterDelay = 3;
        }

        // timeouts
        if (motorDutyCycle < 300) {
          motorZeroCounterTimeoutThreshold = 4000;
        } else {
          motorZeroCounterTimeoutThreshold = 2000;
        }

        if (++motorZeroCounterTimeout > motorZeroCounterTimeoutThreshold) {
          motorRunning = false;
          motorDutyCycle = 0;
          motorZeroCounterTimeout = motorZeroCounterTimeoutThreshold + 1;
        }

        // motor start
        if ((motorStartup) && (!motorRunning)) {
          motorZeroCounterTimeout = 0;
          motorStart();
        }

      } // inputArmed
    } // inputProtocol detected

    // ESC hardware limits
    #if !(defined(FURLING45MINI) || defined(DYS35ARIA))
    adcCurrent = kalmanUpdate(&adcCurrentFilterState, (float)adcCurrentRaw);
    if ((escConfig()->limitCurrent > 0) && (adcCurrent > escConfig()->limitCurrent)) {
      inputDisarm();
      #if (!defined(_DEBUG_))
      LED_ON(RED);
      #endif
    }
    #endif

    #if (defined(_DEBUG_) && defined(CYCLETIME_MAINLOOP))
    LED_ON(BLUE);
    #endif
  } // main loop

} // main
