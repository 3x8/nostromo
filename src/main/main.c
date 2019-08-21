#include "main.h"

// filter
kalman_t motorCommutationIntervalFilterState;
#if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2) || defined(FURLING45MINI))
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

  if (!serialPort.InitDone){
    uartInit();
    serialPort.InitDone = true;
  }

  kalmanInit(&motorCommutationIntervalFilterState, 1500.0f, 13);
  #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2) || defined(FURLING45MINI))
    kalmanInit(&adcVoltageFilterState, 1500.0f, 13);
    kalmanInit(&adcCurrentFilterState, 1500.0f, 13);
  #endif

  // start with motor off
  motor.Step = 1;
  motor.Direction = escConfig()->motorDirection;
  motor.SlowDecay = escConfig()->motorSlowDecay;
  input.Data = 0;
  input.PwmValue = 0;
  serialPort.InitDone = false;

  // ToDo 3D double
  if(escConfig()->motor3Dmode) {
    escConfig()->limitCurrent = HBRIDGE_MAX_CURRENT << 1;
    escConfig()->motorStartThreshold = MOTOR_START_THRESHOLD << 1;
  } else {
    escConfig()->limitCurrent = HBRIDGE_MAX_CURRENT;
    escConfig()->motorStartThreshold = MOTOR_START_THRESHOLD;
  }

  watchdogInit(2000);
  motorStartupTune();

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

        #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2) || defined(FURLING45MINI))
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

        // uart init
        if (!serialPort.InitDone){
          uartInit();
          serialPort.InitDone = true;
        }

        // motor BEMF filter
        if (input.PwmValue < 67) {
          motor.BemfFilterLevel = 5;
          motor.BemfFilterDelay = 5;
        } else {
          if ((motor.CommutationInterval < 401) && (input.PwmValue > 503)) {
            motor.BemfFilterDelay = 1;
            motor.BemfFilterLevel = 1;
          } else {
            motor.BemfFilterLevel = 3;
            motor.BemfFilterDelay = 3;
          }
        }

        // motor BEMF timeouts
        if (input.PwmValue < 300) {
          motor.BemfZeroCounterTimeoutThreshold = 401;
        } else {
          motor.BemfZeroCounterTimeoutThreshold = 199;
        }

        // motor not running
        if (++motor.BemfZeroCounterTimeout > motor.BemfZeroCounterTimeoutThreshold) {
          motor.BemfZeroCrossTimestamp = 0;
            kalmanInit(&motorCommutationIntervalFilterState, 1500.0f, 13);
          motor.Running = false;
        }

        // motor start
        if ((motor.Startup) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorStart();
        }

        // ToDo
        motor.CommutationInterval = kalmanUpdate(&motorCommutationIntervalFilterState, (float)motor.BemfZeroCrossTimestamp);
        motor.CommutationDelay = 0; //timing 30°
        //motor.CommutationDelay = motor.CommutationInterval >> 3; //timing 15°
        //motor.CommutationDelay = motor.CommutationInterval >> 2; //timing 0°
        //motor.CommutationDelay = constrain(motor.CommutationDelay, 17, 413);
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

    #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2) || defined(FURLING45MINI))
      adcScaled.current = ((kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset));
      adcScaled.voltage = ((kalmanUpdate(&adcVoltageFilterState, (float)adcRaw.voltage) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET));
      if ((escConfig()->limitCurrent > 0) && (adcScaled.current > 0) && (ABS(adcScaled.current) > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }

      if (msTimerHandle.Instance->CNT > 100) {
        #if (defined(_DEBUG_) && defined(DEBUG_MS_TIMER))
          LED_TOGGLE(LED_GREEN);
        #endif
        msTimerHandle.Instance->CNT = 0;
        consumptionMah += adcScaled.current * ADC_CONSUMPTION_FACTOR;
      }
    #endif

    if (input.TelemetryRequest) {
      telemetry();
      input.TelemetryRequest = false;
    }

    #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
      static uint8_t  printIndex = 0;

      if ((msTimerHandle.Instance->CNT % 101) == 0) {

        uartPrint("IN[");
        uartPrintInteger(input.Data, 10, 1);
        uartPrint("] ");
        uartPrint("INv[");
        uartPrintInteger(input.DataValid, 10, 1);
        uartPrint("] ");
        uartPrint("INp[");
        uartPrintInteger(input.Protocol, 10, 1);
        uartPrint("] ");
        uartPrint("INN[");
        uartPrintInteger(input.DataNormed, 10, 1);
        uartPrint("] ");
        uartPrint("PWM[");
        uartPrintInteger(input.PwmValue, 10, 1);
        uartPrint("] ");

        uartPrint("Ufs[");
        uartPrintInteger(adcScaled.voltage, 10, 1);
        uartPrint("] ");
        uartPrint("Ifs[");
        uartPrintInteger(ABS(adcScaled.current), 10, 1);
        uartPrint("] ");
        /*uartPrint("Ur[");
        uartPrintInteger(adcRaw.voltage, 10, 1);
        uartPrint("] ");
        uartPrint("Ir[");
        uartPrintInteger(adcRaw.current, 10, 1);
        uartPrint("] ");*/
        uartPrint("Ts[");
        uartPrintInteger(adcScaled.temperature, 10, 1);
        uartPrint("] ");
        /*
        uartPrint("mAh[");
        uartPrintInteger(ABS((int)consumptionMah), 10, 1);
        uartPrint("] ");
        uartPrint("telemAh[");
        uartPrintInteger(telemetryData.consumption, 10, 1);
        uartPrint("] ");*/

        uartPrint("RPM[");
        uartPrintInteger(7744820/motor.CommutationInterval, 10, 1); // RCBenchmark calibrated
        //uartPrintInteger(9276437/motor.CommutationInterval, 10, 1); //calculated
        uartPrint("] ");

        uartPrint("\r\n");
        printIndex = 0;
      } else {
        printIndex++;
      }
    #endif

    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_ON(LED_GREEN);
    #endif
  } // main loop

} // main
