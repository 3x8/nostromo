#include "main.h"

medianStructure motorCommutationIntervalFilterState;

#if (defined(USE_ADC))
  #if (defined(USE_ADC_MEDIAN))
    medianStructure adcVoltageFilterState, adcCurrentFilterState;
  #else
    kalmanStructure adcVoltageFilterState, adcCurrentFilterState;
  #endif
#endif

int main(void) {
  #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
    uint32_t mainBegin, mainTime;
  #endif

  #if (defined(USE_BOOTLOADER))
    systemInitAfterBootloaderJump();
  #endif

  // basic initialization
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

  medianInit(&motorCommutationIntervalFilterState, MOTOR_BLDC_MEDIAN);

  #if (defined(USE_ADC))
    #if (defined(USE_ADC_MEDIAN))
      medianInit(&adcVoltageFilterState, 113);
      medianInit(&adcCurrentFilterState, 113);
    #else
      kalmanInit(&adcVoltageFilterState, 25000.0f, 7);
      kalmanInit(&adcCurrentFilterState, 25000.0f, 7);
    #endif
  #endif

  // start with motor off
  motor.Step = 1;
  motor.Direction = escConfig()->motorDirection;
  motor.ComplementaryPWM = escConfig()->motorComplementaryPWM;
  #if (defined(USE_ADC_MEDIAN))
    motor.BemfZeroCounterTimeoutThreshold = 71;
  #else
    motor.BemfZeroCounterTimeoutThreshold = 13;
  #endif
  input.Data = 0;
  input.PwmValue = 0;
  serialPort.InitDone = false;

  watchdogInit(2000);
  motorTuneStartup();

  // main loop
  while (true) {
    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_OFF(LED_GREEN);
      mainBegin = motorCommutationTimerHandle.Instance->CNT;
    #endif

    watchdogFeed();

    // brake
    if ((!motor.Start) && (!motor.Running)) {
      switch (escConfig()->motorBrake) {
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
        #if (defined(USE_ADC))
          // adcCurrent, auto offset at first arm after firmware write
          if (escConfig()->adcCurrentOffset == 0) {
            if (adcScaled.current != 0) {
              escConfig()->adcCurrentOffset = -adcScaled.current;
            } else {
              escConfig()->adcCurrentOffset = 1;
            }
            configWrite();
            // reset esc, iwdg timeout
            while (true);
          }
        #endif

        // uart init (USART2 swd programming locked after arming)
        if (!serialPort.InitDone){
          uartInit();
          serialPort.InitDone = true;
        }

        // motor BEMF filter (1 tick 0.167 us)
        if ((motor.OneErpmTime < 2411) && (input.DataNormed > 500)) {
          // ERpm > 140K
          motor.BemfFilterDelay = 3;
          motor.BemfFilterLevel = 1;
        } else {
          motor.BemfFilterDelay = 7;
          motor.BemfFilterLevel = 3;
        }

        // motor stopped
        if (++motor.BemfZeroCounterTimeout > motor.BemfZeroCounterTimeoutThreshold) {
          motor.BemfZeroCrossTimestamp = 0;
          motor.BemfCounter = 0;
          motor.Running = false;
          medianInit(&motorCommutationIntervalFilterState, MOTOR_BLDC_MEDIAN);
        }

        // motor start
        if ((motor.Start) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorStart();
        }

        motor.OneErpmTime = medianSumm(&motorCommutationIntervalFilterState) >> 3;

        // ToDo
        //motor.CommutationDelay = 0; //timing 30°
        motor.CommutationDelay = constrain((motor.OneErpmTime >> 3), 41, 401); //timing 15°
        //motor.CommutationDelay = constrain((motor.OneErpmTime >> 2), 41, 401); //timing 0°
      } // input.Armed
    } // input.Protocol detected

    // adc limits
    adcScaled.temperature = ((adcRaw.temperature  * ADC_TEMPERATURE_FACTOR) + ADC_TEMPERATURE_OFFSET);
    if ((escConfig()->limitTemperature > 0) && (ABS(adcScaled.temperature) > escConfig()->limitTemperature)) {
      inputDisarm();
      #if (!defined(_DEBUG_))
        LED_ON(LED_GREEN);
      #endif
    }

    #if (defined(USE_ADC))
      #if (defined(USE_ADC_MEDIAN))
        medianPush(&adcCurrentFilterState, adcRaw.current);
        adcScaled.current = medianCalculate(&adcCurrentFilterState) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset;
        medianPush(&adcVoltageFilterState, adcRaw.voltage);
        adcScaled.voltage = medianCalculate(&adcVoltageFilterState) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET;
      #else
        adcScaled.current = ((kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset));
        adcScaled.voltage = ((kalmanUpdate(&adcVoltageFilterState, (float)adcRaw.voltage) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET));
      #endif

      if ((escConfig()->limitCurrent > 0) && (ABS(adcScaled.current) > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }
    #endif

    if (msTimerHandle.Instance->CNT > 200) {
      msTimerHandle.Instance->CNT = 0;
      #if (defined(USE_ADC))
        consumptionMah += adcScaled.current * ADC_CONSUMPTION_FACTOR;
      #endif
      #if (defined(_DEBUG_) && defined(DEBUG_MS_TIMER))
        LED_TOGGLE(LED_GREEN);
      #endif
    }

    if (input.TelemetryRequest) {
      telemetry();
      input.TelemetryRequest = false;
      #if (!defined(_DEBUG_))
        if (input.Armed) {
          LED_ON(LED_BLUE);
        }
      #endif
    } else {
      #if (!defined(_DEBUG_))
        LED_OFF(LED_BLUE);
      #endif
    }

    #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
      extern uint32_t motorDebugTime;
      if (((msTimerHandle.Instance->CNT % 2) == 0) && (input.DataNormed > 0)) {
        // each 1ms
        uartPrintInteger(msTimerHandle.Instance->CNT, 10, 1);
        uartPrint(",");
        uartPrintInteger(input.DataNormed, 10, 1);
        uartPrint(",");
        if (motor.OneErpmTime > 0) {
          #if (defined(DEBUG_CYCLETIME_MAINLOOP))
            uartPrintInteger(mainTime * 0.17, 10, 1);
          #else
            //uartPrintInteger(motor.OneErpmTime, 10, 1);
            uartPrintInteger(motorGetRpm(), 10, 1);
          #endif
        } else {
          uartPrintInteger(0, 10, 1);
        }
        uartPrint(",");
        uartPrintInteger(motorDebugTime, 10, 1);
        uartPrint(",");
        uartPrintInteger(adcScaled.voltage, 10, 1);
        uartPrint(",");
        uartPrintInteger(ABS(adcScaled.current), 10, 1);
        uartPrint("\r\n");
      }
    #endif

    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_ON(LED_GREEN);
      mainTime = motorCommutationTimerHandle.Instance->CNT - mainBegin;
      if (mainTime > 100000) {
        mainTime = -mainTime;
      } 
    #endif
  } // main loop

} // main
