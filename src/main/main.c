#include "main.h"

medianStructure motorCommutationIntervalFilter;

#if (defined(USE_ADC))
  kalmanStructure adcVoltageFilter, adcCurrentFilter;
#endif

int main(void) {
  #if (defined(USE_BOOTLOADER))
    systemInitAfterBootloaderJump();
  #endif

  // basic initialization
  HAL_Init();
  systemClockConfig();
  systemDmaInit();
  systemBemfComparatorInit();
  systemAdcInit();
  systemMotorPwmTimerInit();
  systemMotorCommutationTimerInit();
  systemMotorAutotimingTimerInit();
  systemInputTimerInit();
  systemMsTimerInit();
  configValidateOrReset();
  configRead();
  ledInit();

  medianInit(&motorCommutationIntervalFilter, MOTOR_BLDC_MEDIAN);

  #if (defined(USE_ADC))
    kalmanInit(&adcVoltageFilter, 25000.0f, 7);
    kalmanInit(&adcCurrentFilter, 25000.0f, 7);
  #endif

  // start with motor off
  motor.Step = 1;
  motor.Direction = escConfig()->motorDirection;
  motor.ComplementaryPWM = escConfig()->motorComplementaryPWM;
  motor.BemfZeroCounterTimeoutThreshold = 13;
  #if defined(SUCCEXMINI40A)
    motor.BemfZeroCounterTimeoutThreshold = 313;
  #endif
  input.Data = 0;
  input.PwmValue = 0;
  serialPort.InitDone = false;

  watchdogInit(2000);
  motorTuneReady();

  // main loop
  while (true) {
    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_OFF(LED_GREEN);
      uint32_t mainBegin = motorCommutationTimerHandle.Instance->CNT;
    #endif

    watchdogFeed();

    // brake
    if ((!motor.Start) && (!motor.Running)) {
      switch (escConfig()->motorBrake) {
        case BRAKE_FULL:
          motorBrakeFull();
          break;
        case BRAKE_OFF:
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
          medianInit(&motorCommutationIntervalFilter, MOTOR_BLDC_MEDIAN);
        }

        // motor start
        if ((motor.Start) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorStart();
        }

        // motor timing (one Erpm -> 360°)
        motor.OneErpmTime = medianSumm(&motorCommutationIntervalFilter) >> 3;
        motor.OneDegree = motor.OneErpmTime / 360;
        if ((escConfig()->motorCommutationDelay > 0) &&  (escConfig()->motorCommutationDelay <= 30)) {
          motor.CommutationDelay = constrain((escConfig()->motorCommutationDelay * motor.OneDegree), MOTOR_AUTOTIMING_DELAY_LIMIT_MIN, MOTOR_AUTOTIMING_DELAY_LIMIT_MAX);
        } else {
          motor.CommutationDelay = constrain(((30 - scaleLinear(input.PwmValue, INPUT_PWM_MIN, INPUT_PWM_MAX, MOTOR_AUTOTIMING_DELAY_DEGREES_MIN, MOTOR_AUTOTIMING_DELAY_DEGREES_MAX)) * motor.OneDegree), MOTOR_AUTOTIMING_DELAY_LIMIT_MIN, MOTOR_AUTOTIMING_DELAY_LIMIT_MAX);
        }
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

    // adc filtering
    #if (defined(USE_ADC))
      adcScaled.current = (kalmanUpdate(&adcCurrentFilter, (float)adcRaw.current) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset);
      adcScaled.voltage = (kalmanUpdate(&adcVoltageFilter, (float)adcRaw.voltage) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET);

      if ((escConfig()->limitCurrent > 0) && (ABS(adcScaled.current) > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }
    #endif

    // integrate consumption
    if (msTimerHandle.Instance->CNT > 200) {
      msTimerHandle.Instance->CNT = 0;
      #if (defined(USE_ADC))
        consumptionMah += adcScaled.current * ADC_CONSUMPTION_FACTOR;
      #endif
      #if (defined(_DEBUG_) && defined(DEBUG_MS_TIMER))
        LED_TOGGLE(LED_GREEN);
      #endif
    }

    // telemetry
    #if (!defined(DEBUG_DATA_UART))
      if (input.RequestTelemetry) {
        telemetry();
        input.RequestTelemetry = false;
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
    #endif

    // debug (each 1ms)
    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      uint32_t mainTime;
    #endif
    #if (defined(_DEBUG_) && defined(DEBUG_DATA_UART))
      if (((msTimerHandle.Instance->CNT % 2) == 0) && (input.DataNormed > 0)) {
        // csv tests
        /*
        uartPrintInteger(msTimerHandle.Instance->CNT, 10, 1);
        uartPrint(","); */

        uartPrintInteger(input.PwmValue, 10, 1);
        uartPrint(",");

        // debug cycle (us)
        if (motor.OneErpmTime > 0) {
          uartPrintInteger(motorGetRpm(), 10, 1);
        } else {
          uartPrintInteger(0, 10, 1);
        }
        uartPrint(",");
        #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
          uartPrintInteger(mainTime * 0.167, 10, 1);
          uartPrint(",");
        #endif

        // debug motor timing (us)
        uartPrintInteger(motor.OneErpmTime * 0.028, 10, 1); // commutationTime
        uartPrint(",");
        uartPrintInteger(motor.CommutationDelay * 0.167, 10, 1);
        uartPrint(",");

        // debug adc
        /*
        uartPrintInteger(adcRaw.voltage, 10, 1);
        uartPrint(",");
        uartPrintInteger(adcRaw.current, 10, 1);
        uartPrint(","); */
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
