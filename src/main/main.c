#include "main.h"

// debug
uint8_t  printIndex = 0;

// filter
#if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2))
  kalman_t adcVoltageFilterState, adcCurrentFilterState;
#endif

kalman_t motorCommutationIntervalFilterState;


int main(void) {
  HAL_Init();
  systemClockConfig();

  uartInit();

  configValidateOrReset();
  configRead();

  ledInit();

  systemDmaInit();
  systemBemfComparatorInit();
  systemAdcInit();
  systemMotorPwmTimerInit();
  systemMotorCommutationTimerInit();
  systemInputTimerInit();

  // init
  ledOff();
  kalmanInit(&motorCommutationIntervalFilterState, 2500.0f, 13);

  #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2))
    kalmanInit(&adcVoltageFilterState, 2500.0f, 5);
    kalmanInit(&adcCurrentFilterState, 2500.0f, 5);
  #endif

  // start with motor off
  motor.Step = 1;
  motor.Direction = escConfig()->motorDirection;
  motor.SlowDecay = escConfig()->motorSlowDecay;
  input.Data = 0;
  input.PwmValue = 0;

  watchdogInit(2000);
  motorStartupTune();

  // main loop
  while (true) {

    #if (defined(_DEBUG_) && defined(CYCLETIME_MAINLOOP))
      LED_OFF(LED_BLUE);
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

        // motor BEMF filter
        if ((motor.CommutationInterval < 400) && (input.PwmValue > 500)) {
          motor.BemfFilterDelay = 1;
          motor.BemfFilterLevel = 1;
        } else {
          motor.BemfFilterLevel = 3;
          motor.BemfFilterDelay = 3;
        }

        // timeouts
        if (input.PwmValue < 300) {
          motor.BemfZeroCounterTimeoutThreshold = 400;
        } else {
          motor.BemfZeroCounterTimeoutThreshold = 200;
        }

        // motor not turning
        if (++motor.BemfZeroCounterTimeout > motor.BemfZeroCounterTimeoutThreshold) {
          motor.BemfZeroCrossTimestamp = 0;
          kalmanInit(&motorCommutationIntervalFilterState, 2500.0f, 7);
          motor.Running = false;
          //input.PwmValue = 0;
        }

        // motor start
        if ((motor.Startup) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorStart();
        }

        // ToDo
        //motor.CommutationInterval = motor.BemfZeroCrossTimestamp;
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

    #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2))
      // ToDo
      //adcScaled.current = ((adcRaw.current - ADC_CURRENT_OFFSET) * ADC_CURRENT_FACTOR);
      //adcScaled.voltage = ((adcRaw.voltage - ADC_VOLTAGE_OFFSET) * ADC_VOLTAGE_FACTOR);
      adcFiltered.current = kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current);
      adcFiltered.voltage = kalmanUpdate(&adcVoltageFilterState, (float)adcRaw.voltage);
      adcScaled.current = ((adcFiltered.current - ADC_CURRENT_OFFSET) * ADC_CURRENT_FACTOR);
      adcScaled.voltage = ((adcFiltered.voltage - ADC_VOLTAGE_OFFSET) * ADC_VOLTAGE_FACTOR);
      if ((escConfig()->limitCurrent > 0) && (ABS(adcScaled.current) > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }
    #endif

    /*
    #if (!defined(_DEBUG_))
      telemetry();
    #endif*/

    //#if (defined(_DEBUG_))
      if ( ((input.PwmValue > 500) && (printIndex > 7)) || ((input.PwmValue < 500) && (printIndex > 100)) ){

        /*
        uartPrint("[");
        uartPrint(byte_to_binary(telegramPulseValue[0]));
        uartPrint("]");
        uartPrint("[");
        uartPrint(byte_to_binary(telegramPulseValue[1]));
        uartPrint("]");
        uartPrint("[");
        uartPrint(byte_to_binary(telegramPulseValue[2]));
        uartPrint("]");
        uartPrint("[");
        uartPrint(byte_to_binary(telegramPulseValue[3]));
        uartPrint("] ");*/
        /*
        uartPrint("TR[");
        uartPrintInteger(input.TelemetryRequest, 10, 1);
        uartPrint("] ");*/

        /*
        uartPrint("ARM[");
        uartPrintInteger(input.Armed, 10, 1);
        uartPrint("] ");

        uartPrint("VALID[");
        uartPrintInteger(input.DataValid, 10, 1);
        uartPrint("] ");*/

        uartPrint("IN[");
        uartPrintInteger(input.Data, 10, 1);
        uartPrint("] ");

        /*
        uartPrint("INN[");
        uartPrintInteger(input.DataNormed, 10, 1);
        uartPrint("] ");*/

        uartPrint("PWM[");
        uartPrintInteger(input.PwmValue, 10, 1);
        uartPrint("] ");
/*
        uartPrint("Ur[");
        uartPrintInteger(adcRaw.voltage, 10, 1);
        uartPrint("] ");*/

        uartPrint("If[");
        uartPrintInteger(adcRaw.current, 10, 1);
        uartPrint("] ");

/*
        uartPrint("Ufs[");
        uartPrintInteger(adcScaled.voltage, 10, 1);
        uartPrint("] ");

        uartPrint("Ifs[");
        uartPrintInteger(adcScaled.current, 10, 1);
        uartPrint("] ");*/

        uartPrint("Ts[");
        uartPrintInteger(adcScaled.temperature, 10, 1);
        uartPrint("] ");

        uartPrint("RPM[");
        uartPrintInteger(7744820/motor.CommutationInterval, 10, 1); // RCBenchmark calibrated
        //uartPrintInteger(9276437/motor.CommutationInterval, 10, 1); //calculated
        uartPrint("] ");

        /*
        uartPrint("BEMF[");
        uartPrintInteger(motor.CommutationInterval, 10, 1);
        uartPrint("] ");

        uartPrint("BEMFr[");
        uartPrintInteger(motor.BemfZeroCrossTimestamp, 10, 1);
        uartPrint("] ");*/

        uartPrint("\r\n");
        printIndex = 0;
      } else {
        printIndex++;
      }
    //#endif

    #if (defined(_DEBUG_) && defined(CYCLETIME_MAINLOOP))
      LED_ON(LED_BLUE);
    #endif
  } // main loop

} // main
