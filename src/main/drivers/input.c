#include "input.h"

TIM_HandleTypeDef inputTimerHandle;
DMA_HandleTypeDef inputTimerDmaHandle;
input_t input;
uint32_t inputBufferDMA[INPUT_BUFFER_DMA_SIZE];

void inputArmCheck(void) {
  if (!input.Armed) {
    if ((input.Protocol != AUTODETECT) && (input.Data < DSHOT_CMD_MAX) && input.DataValid) {
      input.ArmingCounter++;
      HAL_Delay(1);
      if (input.ArmingCounter > INPUT_ARM_COUNTER_THRESHOLD) {
        input.Armed = true;
        #if (!defined(_DEBUG_))
          LED_ON(LED_BLUE);
        #endif
        motorInputTune(1);
      }
    }
  }
}

void inputDisarm(void) {
  input.DataValid = false;
  input.Data = 0;
  input.DataNormed = 0;
  input.PwmValue = 0;
  input.Armed = false;
  input.ArmingCounter = 0;
  input.TimeoutCounter = 0;

  // output
  motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
  motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
  motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;

  #if (!defined(_DEBUG_))
    LED_OFF(LED_BLUE);
  #endif

  HAL_TIM_IC_Stop_DMA(&inputTimerHandle, TIM_CHANNEL_1);
  input.Protocol = AUTODETECT;
  inputTimerHandle.Instance->PSC = INPUT_AUTODETECT_PRESCALER;
  inputTimerHandle.Instance->CNT = 0xffff;
  HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT);
}

void inputDisarmCheck(void) {
  input.TimeoutCounter++;
  if (input.TimeoutCounter > INPUT_TIMEOUT_COUNTER_THRESHOLD ) {
    inputDisarm();
  }
}

void inputDshotCommandRun(void) {
  if (input.Protocol == PROSHOT) {
    switch (input.Data) {
    case DSHOT_CMD_MOTOR_STOP:
      break;
    case DSHOT_CMD_BEACON1:
      motorInputTune(1);
      break;
    case DSHOT_CMD_BEACON2:
      motorInputTune(2);
      break;
    case DSHOT_CMD_BEACON3:
      motorInputTune(3);
      break;
    case DSHOT_CMD_BEACON4:
      motorInputTune(4);
      break;
    case DSHOT_CMD_BEACON5:
      motorInputTune(5);
      break;
    case DSHOT_CMD_ESC_INFO:
      uartPrint("# ");
      uartPrint(FW_FIRMWARE_NAME);
      uartPrint("_");
      uartPrintInteger(FW_VERSION_MAJOR, 10, 1);
      uartPrint(".");
      uartPrintInteger(FW_VERSION_MINOR, 10, 1);
      uartPrint(".");
      uartPrintInteger(FW_VERSION_PATCH_LEVEL, 10, 1);
      uartPrint(" ");
      uartPrint(__DATE__);
      uartPrint(" / ");
      uartPrint(__TIME__);
      uartPrint(" (");
      uartPrint(__REVISION__);
      uartPrint(")");
      uartPrint("\r\n");
      uartPrint("# MCU UID ");
      uartPrintInteger(U_ID_2, 16, 1);
      uartPrintInteger(U_ID_1, 16, 1);
      uartPrintInteger(U_ID_0, 16, 1);
      uartPrint("\r\n");
      if (escConfig()->motorDirection == SPIN_CW) {
        uartPrint("# CW ");
      } else {
        uartPrint("# CCW ");
      }
      if (escConfig()->motor3Dmode == true) {
        uartPrint("3D ");
      }
      uartPrint("[");
      if (escConfig()->adcCurrentOffset < 0) {
        uartPrint("-");
      }
      uartPrintInteger(ABS(escConfig()->adcCurrentOffset), 16, 1);
      uartPrint("]");
      uartPrint("\r\n");
      break;
    case DSHOT_CMD_SETTING_LED0_ON:
      motorInputTune(6);
      break;
    case DSHOT_CMD_SETTING_SPIN_DIRECTION_NORMAL:
      escConfig()->motorDirection = SPIN_CW;
      uartPrint("# CW");
      uartPrint("\r\n");
      inputDisarm();
      break;
    case DSHOT_CMD_SETTING_SPIN_DIRECTION_REVERSED:
      escConfig()->motorDirection = SPIN_CCW;
      uartPrint("# CCW");
      uartPrint("\r\n");
      inputDisarm();
      break;
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
      motor.Direction = escConfig()->motorDirection;
      break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
      motor.Direction = !escConfig()->motorDirection;
      break;
    case DSHOT_CMD_SETTING_3D_MODE_OFF:
      escConfig()->motor3Dmode = 0;
      uartPrint("# 3D=OFF");
      uartPrint("\r\n");
      inputDisarm();
      break;
    case DSHOT_CMD_SETTING_3D_MODE_ON:
      escConfig()->motor3Dmode = 1;
      uartPrint("# 3D=ON");
      uartPrint("\r\n");
      inputDisarm();
      break;
    case DSHOT_CMD_SETTING_SAVE:
      uartPrint("# SAVE");
      uartPrint("\r\n");
      HAL_Delay(11);
      configWrite();
      // reset esc, iwdg timeout
      while(true);
    default:
      break;
    }
  }
}

void inputCallbackDMA() {
  switch (input.Protocol) {
     case AUTODETECT:
      inputDetectProtocol();
      break;
    case PROSHOT:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, TIM_CHANNEL_1);
      inputProshot();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT);
      break;
    case SERVOPWM:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, TIM_CHANNEL_1);
      inputServoPwm();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM);
      break;
  }
}

void inputDetectProtocol() {
  uint32_t telegramPulseWidthBuff;
  uint32_t telegramPulseWidthMin = 20000;

  #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
    LED_OFF(LED_GREEN);
  #endif

  HAL_TIM_IC_Stop_DMA(&inputTimerHandle, TIM_CHANNEL_1);

  for (int i = 0; i < (INPUT_BUFFER_DMA_SIZE_AUTODETECT - 1); i++) {
    telegramPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];
    if(telegramPulseWidthBuff < telegramPulseWidthMin) {
      telegramPulseWidthMin = telegramPulseWidthBuff;
    }
  }

  if ((telegramPulseWidthMin > INPUT_PROSHOT_WIDTH_MIN_SYSTICKS ) && (telegramPulseWidthMin < INPUT_PROSHOT_WIDTH_MAX_SYSTICKS)) {
    input.Protocol = PROSHOT;
    inputTimerHandle.Instance->PSC = INPUT_PROSHOT_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PROSHOT);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
        LED_ON(LED_GREEN);
    #endif

    return;
  }

  if (telegramPulseWidthMin > 900) {
    input.Protocol = SERVOPWM;
    inputTimerHandle.Instance->PSC = INPUT_PWM_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_PWM);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
      LED_ON(LED_GREEN);
    #endif

    return;
  }

  // default
  if (input.Protocol == AUTODETECT) {
    inputTimerHandle.Instance->PSC = INPUT_AUTODETECT_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, TIM_CHANNEL_1, inputBufferDMA, INPUT_BUFFER_DMA_SIZE_AUTODETECT);
  }
}

void inputProshot() {
  uint8_t telegramCalculatedCRC = 0, telegramReceivedCRC = 0;
  uint16_t telegramData = 0;
  uint8_t telegramPulseValue[4] = {0, 0, 0, 0};

  #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT))
    LED_ON(LED_GREEN);
  #endif

  for (int i = 0; i < 4; i++) {
    telegramPulseValue[i] = ( (inputBufferDMA[i*2 + 1] - inputBufferDMA[i*2]) - 45) / 6;
  }

  for (int i = 0; i < 4; i++) {
    telegramCalculatedCRC = telegramCalculatedCRC | ((telegramPulseValue[0]^telegramPulseValue[1]^telegramPulseValue[2]) << i);
    telegramReceivedCRC = telegramReceivedCRC | (telegramPulseValue[3] << i);
  }

  telegramData = ((telegramPulseValue[0] << 7 | telegramPulseValue[1] << 3 | telegramPulseValue[2] >> 1));

  if ((telegramCalculatedCRC == telegramReceivedCRC) && (telegramData >= INPUT_VALUE_MIN) && (telegramData <= INPUT_VALUE_MAX)) {
    input.DataValid = true;
    input.TimeoutCounter = 0;
    input.Data = telegramData;

    // only update if not active
    if (!input.TelemetryRequest) {
      input.TelemetryRequest = (telegramPulseValue[2] & BIT(0));
    }

    if (input.Armed) {
      if (input.Data <= DSHOT_CMD_MAX) {
        motor.Startup = false;
        input.PwmValue = 0;
        if (!motor.Running) {
          inputDshotCommandRun();
        }
      } else {
        motor.Startup = true;
        motor.BrakeActiveProportional = false;
        input.DataNormed = constrain((input.Data - DSHOT_CMD_MAX), INPUT_NORMED_MIN, INPUT_NORMED_MAX);

        if (escConfig()->motor3Dmode) {
          // up
          if (input.DataNormed >= escConfig()->input3DdeadbandHigh) {
            if (motor.Direction == !escConfig()->motorDirection) {
              motor.Direction = escConfig()->motorDirection;
              motor.BemfCounter = 0;
            }
            input.PwmValue = (input.DataNormed - escConfig()->input3Dneutral) + escConfig()->motorStartThreshold;
          }
          // down
          if ((input.DataNormed < escConfig()->input3Dneutral) && (input.DataNormed >= escConfig()->input3DdeadbandLow)) {
            if(motor.Direction == escConfig()->motorDirection) {
              motor.Direction = !escConfig()->motorDirection;
              motor.BemfCounter = 0;
            }
            input.PwmValue = input.DataNormed + escConfig()->motorStartThreshold;
          }
          // deadband
          if ((input.DataNormed < escConfig()->input3DdeadbandLow) || ((input.DataNormed < escConfig()->input3DdeadbandHigh) && ((input.DataNormed > escConfig()->input3Dneutral)))) {
            input.PwmValue = 0;
          }
        } else {
          input.PwmValue = (input.DataNormed >> 1) + (escConfig()->motorStartThreshold);
          // introduces non linearity.
          //input.PwmValue = scaleInputToOutput(input.DataNormed, INPUT_NORMED_MIN, INPUT_NORMED_MAX, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX) + escConfig()->motorStartThreshold;
        }

        // output
        input.PwmValue = constrain(input.PwmValue, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
        motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
        motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
        motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
      }
    }

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT))
      LED_OFF(LED_GREEN);
    #endif

    return;
  } else {
    input.DataValid = false;

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT))
      LED_OFF(LED_GREEN);
    #endif

    return;
  }
}

// SERVOPWM (use only for thrust tests ...)
void inputServoPwm() {
  uint32_t telegramPulseWidthBuff = 0;

  for (int i = 0; i < (INPUT_BUFFER_DMA_SIZE_PWM - 1); i++) {
    telegramPulseWidthBuff = inputBufferDMA[i + 1] - inputBufferDMA[i];

    if ((telegramPulseWidthBuff >= INPUT_PWM_WIDTH_MIN_US) && (telegramPulseWidthBuff <= INPUT_PWM_WIDTH_MAX_US)) {
      input.DataValid = true;
      input.TimeoutCounter = 0;
      input.Data = scaleInputToOutput(telegramPulseWidthBuff, INPUT_PWM_WIDTH_MIN_US, INPUT_PWM_WIDTH_MAX_US, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);

      if (input.Armed) {
        if (input.Data  < DSHOT_CMD_MAX) {
          motor.Startup = false;
          input.PwmValue = 0;
        } else {
          motor.Startup = true;
          motor.BrakeActiveProportional = false;
          input.PwmValue = constrain(input.Data, OUTPUT_PWM_MIN, OUTPUT_PWM_MAX);
        }

        // output
        motorPwmTimerHandle.Instance->CCR1 = input.PwmValue;
        motorPwmTimerHandle.Instance->CCR2 = input.PwmValue;
        motorPwmTimerHandle.Instance->CCR3 = input.PwmValue;
      }

      return;
    } else {
      input.DataValid = false;
    }
  }

}
