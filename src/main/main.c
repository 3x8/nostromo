#include "main.h"

// filter
medianStructure motorCommutationIntervalFilterState;
uint32_t counter;

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

  // init
  #if (defined(USE_BOOTLOADER))
    systemInitAfterBootloaderJump();
  #endif
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

  // ToDo
  systemMotorSinTimerInit();

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

  //ToDo
  motor.BemfZeroCounterTimeoutThreshold = 22;

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
          motor.BemfFilterDelay = 1;
          motor.BemfFilterLevel = 1;
        } else {
          motor.BemfFilterDelay = 1;
          motor.BemfFilterLevel = 1;
        }

        // motor stopped
        if (++motor.BemfZeroCounterTimeout > motor.BemfZeroCounterTimeoutThreshold) {
          motor.BemfZeroCrossTimestamp = 0;
          motor.BemfCounter = 0;
          motor.Running = false;
          medianInit(&motorCommutationIntervalFilterState, MOTOR_BLDC_MEDIAN);
        }

        // motor start
        /*
        if ((motor.Start) && (!motor.Running)) {
          motor.BemfZeroCounterTimeout = 0;
          motorBrakeOff();
          motorStart();
        }*/

        // motor start
        if (motor.Start) {
          motorBrakeOff();
          HAL_TIM_Base_Start_IT(&motorSinTimerHandle);
        } else {
          HAL_TIM_Base_Stop_IT(&motorSinTimerHandle);
          motorBrakeFull();
        }

        motor.OneErpmTime = medianGetSumm(&motorCommutationIntervalFilterState) >> 3;

        // ToDo
        //motor.CommutationDelay = 0; //timing 30°
        //motor.CommutationDelay = constrain((motor.OneErpmTime >> 3), 41, 401); //timing 15°
        //motor.CommutationDelay = constrain((motor.OneErpmTime >> 2), 41, 1001); //timing 0°
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
        adcScaled.current = medianGetMean(&adcCurrentFilterState) * ADC_CURRENT_FACTOR + escConfig()->adcCurrentOffset;
        medianPush(&adcVoltageFilterState, adcRaw.voltage);
        adcScaled.voltage = medianGetMean(&adcVoltageFilterState) * ADC_VOLTAGE_FACTOR + ADC_VOLTAGE_OFFSET;
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
          //LED_ON(LED_BLUE);
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
      //if ((msTimerHandle.Instance->CNT % 2) == 0) {
        uartPrint("in[");
        uartPrintInteger(input.PwmValue, 10, 1);
        uartPrint("]");
        uartPrint("step[");
        uartPrintInteger(motor.Step, 10, 1);
        uartPrint("]");
        uartPrint(" zct[");
        uartPrintInteger(motor.BemfZeroCrossTimestamp, 10, 1);
        uartPrint("]");
        uartPrint(" run[");
        uartPrintInteger(motor.Running, 10, 1);
        uartPrint("]");
        uartPrint(" to[");
        uartPrintInteger(motor.BemfZeroCounterTimeout, 10, 1);
        uartPrint("]");
        uartPrint(" bemf[");
        uartPrintInteger(motor.BemfCounter, 10, 1);
        uartPrint("]");

        uartPrint("\r\n");
        // each 1ms
        /*
        uartPrintInteger(msTimerHandle.Instance->CNT, 10, 1);
        uartPrint(",");
        uartPrintInteger(input.DataNormed, 10, 1);
        uartPrint(",");
        if (motor.OneErpmTime > 0) {
          #if (defined(DEBUG_CYCLETIME_MAINLOOP))
            uartPrintInteger(mainTime * 0.17, 10, 1);
          #else
          uartPrintInteger(motor.BemfZeroCrossTimestamp, 10, 1);
          uartPrint(",");
          uartPrintInteger(motor.OneErpmTime, 10, 1);
            //uartPrintInteger(motorGetRpm(), 10, 1);
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
        uartPrint("\r\n");*/
      }
    #endif

    #if (defined(_DEBUG_) && defined(DEBUG_CYCLETIME_MAINLOOP))
      LED_ON(LED_GREEN);
      if (mainTime < 100000) {
        mainTime = motorCommutationTimerHandle.Instance->CNT - mainBegin;
      } else {
        mainTime = mainBegin - motorCommutationTimerHandle.Instance->CNT;
      }
    #endif
  } // main loop

} // main
