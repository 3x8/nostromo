#include "main.h"

// debug
uint8_t  printIndex = 0;

// adc
kalman_t adcVoltageFilterState, adcCurrentFilterState, adcTemperatureFilterState;

// motor
kalman_t motorCommutationIntervalFilterState;
extern TIM_HandleTypeDef motorPwmTimerHandle;
extern bool motorStartup, motorRunning;
extern bool motorDirection, motorSlowDecay, motorBrakeActiveProportional;
extern uint32_t  motorBemfCounter, motorBemfZeroCrossTimestamp;
extern uint32_t motorBemfFilterLevel, motorBemfFilterDelay;
extern uint32_t motorBemfZeroCounterTimeout, motorBemfZeroCounterTimeoutThreshold;
extern uint32_t motorCommutationInterval, motorCommutationDelay;

// input
extern TIM_HandleTypeDef  inputTimerHandle;
extern DMA_HandleTypeDef inputTimerDmaHandle;
extern bool inputArmed, inputDataValid;
extern uint8_t  inputProtocol;
extern uint32_t inputData, outputPwm;

int main(void) {
  HAL_Init();
  systemClockConfig();

  serialInit();

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
  kalmanInit(&adcVoltageFilterState, 2500.0f, 31);
  kalmanInit(&adcCurrentFilterState, 2500.0f, 31);
  kalmanInit(&adcTemperatureFilterState, 2500.0f, 31);

  kalmanInit(&motorCommutationIntervalFilterState, 1500.0f, 31);

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
      LED_OFF(LED_BLUE);
    #endif

    watchdogFeed();

    // brake
    if ((!motorStartup) && (!motorRunning)) {
      switch(escConfig()->motorBrake) {
        case BRAKE_FULL:
          motorBrakeActiveProportional = false;
          motorBrakeFull();
          break;
        case BRAKE_PROPORTIONAL:
          motorBrakeActiveProportional = true;
          motorPwmTimerHandle.Instance->CCR1 = escConfig()->motorBrakeStrength;
          motorPwmTimerHandle.Instance->CCR2 = escConfig()->motorBrakeStrength;
          motorPwmTimerHandle.Instance->CCR3 = escConfig()->motorBrakeStrength;
          motorBrakeProportional();
          break;
        case BRAKE_OFF:
          motorBrakeActiveProportional = false;
          motorBrakeOff();
          break;
      }
    }

    if (inputProtocol == AUTODETECT) {
      // noop
    } else {
      inputArmCheck();
      inputDisarmCheck();
      if (inputArmed) {

        // motor BEMF filter
        if ((motorCommutationInterval < 400) && (outputPwm > 500)) {
          motorBemfFilterDelay = 1;
          motorBemfFilterLevel = 1;
        } else {
          motorBemfFilterLevel = 3;
          motorBemfFilterDelay = 3;
        }

        // timeouts
        if (outputPwm < 300) {
          motorBemfZeroCounterTimeoutThreshold = 400;
        } else {
          motorBemfZeroCounterTimeoutThreshold = 200;
        }

        // motor not turning
        if (++motorBemfZeroCounterTimeout > motorBemfZeroCounterTimeoutThreshold) {
          motorBemfZeroCrossTimestamp = 0;
          kalmanInit(&motorCommutationIntervalFilterState, 1500.0f, 31);
          motorRunning = false;
          outputPwm = 0;
        }

        // motor start
        if ((motorStartup) && (!motorRunning)) {
          motorBemfZeroCounterTimeout = 0;
          motorStart();
        }

        // ToDo
        motorCommutationInterval = kalmanUpdate(&motorCommutationIntervalFilterState, (float)motorBemfZeroCrossTimestamp);
        motorCommutationDelay = 0; //timing 30°
        //motorCommutationDelay = motorCommutationInterval >> 3; //timing 15°
        //motorCommutationDelay = motorCommutationInterval >> 2; //timing 0°
        //motorCommutationDelay = constrain(motorCommutationDelay, 17, 413);
      } // inputArmed
    } // inputProtocol detected

    // adc limits
    adcScaled.temperature = ((adcRaw.temperature  * ADC_TEMPERATURE_FACTOR) + ADC_TEMPERATURE_OFFSET);
    if ((adcScaled.temperature > 0) && (escConfig()->limitTemperature > 0) && (adcScaled.temperature > escConfig()->limitTemperature)) {
      inputDisarm();
      #if (!defined(_DEBUG_))
        LED_ON(LED_GREEN);
      #endif
    }

    #if (defined(WRAITH32) || defined(WRAITH32V2) || defined(TYPHOON32V2))
      adcFiltered.current = kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current);
      adcFiltered.voltage = kalmanUpdate(&adcVoltageFilterState, (float)adcRaw.voltage);
      adcScaled.current = ((adcFiltered.current - ADC_CURRENT_OFFSET) * ADC_CURRENT_FACTOR);
      adcScaled.voltage = ((adcFiltered.voltage - ADC_VOLTAGE_OFFSET) * ADC_VOLTAGE_FACTOR);
      if ((adcScaled.current > 0) && (escConfig()->limitCurrent > 0) && (adcScaled.current > escConfig()->limitCurrent)) {
        inputDisarm();
        #if (!defined(_DEBUG_))
          LED_ON(LED_RED);
        #endif
      }
    #endif



    #if (defined(DYS35ARIA))
      adcFiltered.current = kalmanUpdate(&adcCurrentFilterState, (float)adcRaw.current);
      if ((escConfig()->limitCurrent > 0) && (adcFiltered.current > escConfig()->limitCurrent) && (outputPwm > 200)) {
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
      if ( ((outputPwm > 500) && (printIndex > 7)) || ((outputPwm < 500) && (printIndex > 100)) ){
        serialPrint("IN[");
        serialPrintInteger(inputData, 10, 1);
        serialPrint("] ");
        serialPrint("PWM[");
        serialPrintInteger(outputPwm, 10, 1);
        serialPrint("] ");

        /*
        serialPrint("RPM[");
        serialPrintInteger(7037000/motorCommutationInterval, 10, 1);
        //serialPrintInteger(9276437/motorCommutationInterval, 10, 1); //calculated
        serialPrint("] ");

        serialPrint("BEMF[");
        serialPrintInteger(motorCommutationInterval, 10, 1);
        serialPrint("] ");

        serialPrint("BEMFr[");
        serialPrintInteger(motorBemfZeroCrossTimestamp, 10, 1);
        serialPrint("] ");
        */


        serialPrint("Ufs[");
        serialPrintInteger(adcScaled.voltage, 10, 1);
        serialPrint("] ");

        serialPrint("Ifs[");
        serialPrintInteger(adcScaled.current, 10, 1);
        serialPrint("] ");


        serialPrint("Tr[");
        serialPrintInteger(adcRaw.temperature, 10, 1);
        serialPrint("] ");

        serialPrint("Ts[");
        serialPrintInteger(adcScaled.temperature, 10, 1);
        serialPrint("] ");



        serialPrint("\r\n");
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
