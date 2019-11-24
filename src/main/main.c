#include "main.h"

uint32_t msIndex;

// filter
kalman_t motorCommutationIntervalFilterState;
#if defined(USE_ADC)
  kalman_t adcVoltageFilterState, adcCurrentFilterState;
#endif

int main(void) {
  // init
  HAL_Init();
  systemClockConfig();
  configValidateOrReset();
  configRead();
  ledInit();
  systemDmaInit();
  systemBemfComparatorInit();
  systemAdcInit();
  systemMotorPwmTimerInit();
  systemMotorCommutationTimerInit();
  systemInputTimerInit();
  systemMsTimerInit();
  ledOff();

  kalmanInit(&motorCommutationIntervalFilterState, 2500.0f, 13);
  #if defined(USE_ADC)
    kalmanInit(&adcVoltageFilterState, 1500.0f, 13);
    kalmanInit(&adcCurrentFilterState, 1500.0f, 13);
  #endif

  // start with motor off
  motor.Step = 1;
  motor.BemfZeroCounterTimeoutThreshold = 27;
  motor.Direction = escConfig()->motorDirection;
  motor.ComplementaryPWM = escConfig()->motorComplementaryPWM;
  input.Data = 0;
  input.PwmValue = 0;
  serialPort.InitDone = false;

  watchdogInit(2000);
  motorTuneStartup();

  // main loop
  while (true) {
    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_OFF(LED_GREEN);
    #endif

    watchdogFeed();

    // brake
    if ((!motor.Startup) && (!motor.Running)) {
      switch(escConfig()->motorBrake) {
        case BRAKE_FULL:
          motor.BrakeActiveProportional = false;
          motorBrakeFull();
          break;
        case BRAKE_PROPORTIONAL:
          motor.BrakeActiveProportional = true;
          motorPwmTimerHandle.Instance->CCR1 = escConfig()->motorBrakeStrength;
          motorPwmTimerHandle.Instance->CCR2 = escConfig()->motorBrakeStrength;
          motorPwmTimerHandle.Instance->CCR3 = escConfig()->motorBrakeStrength;
          motorBrakeProportional();
          break;
        case BRAKE_OFF:
          motor.BrakeActiveProportional = false;
          motorBrakeOff();
          break;
      }
    }

    if (input.Protocol == AUTODETECT) {
      // noop
    } else {
      inputArmCheck();
      inputDisarmCheck();
      if (input.Armed) {
        #if defined(USE_ADC)
          // adcCurrent, auto offset at first arm after firmware write
          if (escConfig()->adcCurrentOffset == 0) {
            if (adcScaled.current != 0) {
              escConfig()->adcCurrentOffset = -adcScaled.current;
            } else {
              escConfig()->adcCurrentOffset = 1;
            }
            configWrite();
            // reset esc, iwdg timeout
            while(true);
          }
        #endif

        // uart init (USART2 swd programming locked after arming)
        if (!serialPort.InitDone){
          uartInit();
          serialPort.InitDone = true;
        }

        // motor BEMF filter
        if ((motor.CommutationInterval < 401) && (input.PwmValue > 503)) {
          motor.BemfFilterDelay = 5;
          motor.BemfFilterLevel = 2;
        } else {
          motor.BemfFilterDelay = 7;
          motor.BemfFilterLevel = 3;
        }

        // motor not running
        if (++motor.BemfZeroCounterTimeout > motor.BemfZeroCounterTimeoutThreshold) {
          motor.BemfZeroCrossTimestamp = 0;
          motor.BemfCounter = 0;
          motor.Running = false;
          kalmanInit(&motorCommutationIntervalFilterState, 2500.0f, 13);
        }

        // motor start
        if ((motor.Startup) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorStart();
        }

        // ToDo
        motor.CommutationInterval = kalmanUpdate(&motorCommutationIntervalFilterState, (float)motor.BemfZeroCrossTimestamp);
        motor.CommutationDelay = 0; //timing 30°
        //motor.CommutationDelay = constrain((motor.CommutationInterval >> 3), 41, 401); //timing 15°
        //motor.CommutationDelay = constrain((motor.CommutationInterval >> 2), 41, 401); //timing 0°
      } // input.Armed
    } // input.Protocol detected

    // adc limits
    adcScaled.temperature = ((adcRaw.temperature  * ADC_TEMPERATURE_FACTOR) + ADC_TEMPERATURE_OFFSET);
    if ((adcScaled.temperature > 0) && (escConfig()->limitTemperature > 0) && (adcScaled.temperature > escConfig()->limitTemperature)) {
      inputDisarm();
      #if (!defined(_DEBUG_))
        LED_ON(LED_GREEN);
      #endif
    }

    #if defined(USE_ADC)
      adcScaled.current = ((kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset));
      adcScaled.voltage = ((kalmanUpdate(&adcVoltageFilterState, (float)adcRaw.voltage) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET));
      if ((escConfig()->limitCurrent > 0) && (adcScaled.current > 0) && (ABS(adcScaled.current) > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }
    #endif

    if (msTimerHandle.Instance->CNT > 100) {
      msTimerHandle.Instance->CNT = 0;
      msIndex++;

      #if defined(USE_ADC)
        consumptionMah += adcScaled.current * ADC_CONSUMPTION_FACTOR;
      #endif

      #if (defined(_DEBUG_) && defined(DEBUG_MS_TIMER))
        LED_TOGGLE(LED_GREEN);
      #endif
    }

    if (input.TelemetryRequest) {
      telemetry();
      input.TelemetryRequest = false;
    }

    #if (!defined(_DEBUG_))
      if (input.Armed) {
        if ((msIndex % 10) == 0){
          if ((msTimerHandle.Instance->CNT > 11) && msTimerHandle.Instance->CNT < 21) {
            LED_ON(LED_BLUE);
          } else {
            LED_OFF(LED_BLUE);
          }
        }
      } else {
        LED_OFF(LED_BLUE);
      }
    #endif

    #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
      if ((msTimerHandle.Instance->CNT % 101) == 0) {
        /*
        uartPrint("ARM[");
        uartPrintInteger(input.Armed, 10, 1);
        uartPrint("] ");
        uartPrint("IN[");
        uartPrintInteger(input.Data, 10, 1);
        uartPrint("] ");
        uartPrint("INN[");
        uartPrintInteger(input.DataNormed, 10, 1);
        uartPrint("] ");
        uartPrint("PWM[");
        uartPrintInteger(input.PwmValue, 10, 1);
        uartPrint("] ");
        uartPrint("STA[");
        uartPrintInteger(input.DataValidCounter, 10, 1);
        uartPrint("->");
        uartPrintInteger(input.DataErrorCounter, 10, 1);
        uartPrint("->");
        if (input.DataValidCounter > 0) {
          uartPrintInteger((input.DataErrorCounter * 100)/ input.DataValidCounter, 10, 1);
        }
        uartPrint("] ");*/
        uartPrint("Ufs[");
        uartPrintInteger(adcScaled.voltage, 10, 1);
        uartPrint("] ");
        uartPrint("Ifs[");
        uartPrintInteger(ABS(adcScaled.current), 10, 1);
        uartPrint("] ");
        uartPrint("Ts[");
        uartPrintInteger(adcScaled.temperature, 10, 1);
        uartPrint("] ");

        /*
        uartPrint("Ur[");
        uartPrintInteger(adcRaw.voltage, 10, 1);
        uartPrint("] ");
        uartPrint("Ir[");
        uartPrintInteger(adcRaw.current, 10, 1);
        uartPrint("] ");

        uartPrint("mAh[");
        uartPrintInteger(ABS((int)consumptionMah), 10, 1);
        uartPrint("] ");
        uartPrint("telemAh[");
        uartPrintInteger(telemetryData.consumption, 10, 1);
        uartPrint("] ");*/

        /*
        uartPrint("MCI[");
        uartPrintInteger(motor.CommutationInterval, 10, 1);
        uartPrint("] ");
        uartPrint("MCD[");
        uartPrintInteger(motor.CommutationDelay, 10, 1);
        uartPrint("] ");
        uartPrint("ZCT[");
        uartPrintInteger(motor.BemfZeroCrossTimestamp, 10, 1);
        uartPrint("] ");*/
        uartPrint("RPM[");
        if (motor.CommutationInterval > 0) {
          uartPrintInteger(7744820/motor.CommutationInterval, 10, 1); // RCBenchmark calibrated
          //uartPrintInteger(9276437/motor.CommutationInterval, 10, 1); //calculated
        }
        uartPrint("] ");
        uartPrint("\r\n");
      }
    #endif

    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_ON(LED_GREEN);
    #endif
  } // main loop

} // main
