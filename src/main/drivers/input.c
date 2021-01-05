#include "input.h"

TIM_HandleTypeDef inputTimerHandle;
DMA_HandleTypeDef inputTimerDmaHandle;
inputStructure input;
uint32_t inputDmaBuffer[INPUT_DMA_BUFFER_SIZE_MAX];

void inputArmCheck(void) {
  if (!input.Armed) {
    if ((input.Protocol != AUTODETECT) && (input.Data < DSHOT_CMD_MAX) && input.DataValid) {
      input.ArmingCounter++;
      HAL_Delay(1);
      if (input.ArmingCounter > INPUT_ARM_COUNTER_THRESHOLD) {
        input.Armed = true;
        motorTuneInput(1);
      }
    }
  }
}

void inputDisarm(void) {
  __disable_irq();

  // output
  motorPwmTimerHandle.Instance->CCR1 = 0;
  motorPwmTimerHandle.Instance->CCR2 = 0;
  motorPwmTimerHandle.Instance->CCR3 = 0;

  // input
  input.DataValid = false;
  input.Data = 0;
  input.DataNormed = 0;
  input.PwmValue = 0;
  input.Armed = false;
  input.ArmingCounter = 0;
  input.TimeoutCounter = 0;
  input.Protocol = AUTODETECT;

  HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
  inputTimerHandle.Instance->PSC = INPUT_AUTODETECT_PRESCALER;
  inputTimerHandle.Instance->CNT = 0xffff;
  HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_AUTODETECT);

  __enable_irq();
}

void inputDisarmCheck(void) {
  input.TimeoutCounter++;
  if (input.TimeoutCounter > INPUT_TIMEOUT_COUNTER_THRESHOLD ) {
    inputDisarm();
  }
}

void inputDshotCommandRun(void) {
  if ((input.Protocol == PROSHOT1000) || (input.Protocol == DSHOT300) || (input.Protocol == DSHOT600)) {
    switch (input.Data) {
    case DSHOT_CMD_MOTOR_STOP:
      motor.Start = false;
      input.DataNormed = 0;
      input.PwmValue = 0;
      input.PwmValueLast = 0;
      break;
    case DSHOT_CMD_BEACON1:
      if (!motor.Running) {
        motorTuneInput(1);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_BEACON2:
      if (!motor.Running) {
        motorTuneInput(2);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_BEACON3:
      if (!motor.Running) {
        motorTuneInput(3);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_BEACON4:
      if (!motor.Running) {
        motorTuneInput(4);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_BEACON5:
      if (!motor.Running) {
        motorTuneInput(5);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_ESC_INFO:
      uartPrint("# ");
      uartPrint(__TARGET__);
      uartPrint(" ");
      uartPrint(FW_FIRMWARE_NAME);
      uartPrint("_");
      uartPrintInteger(FW_VERSION_MAJOR, 10, 1);
      uartPrint(".");
      uartPrintInteger(FW_VERSION_MINOR, 10, 1);
      uartPrint(".");
      uartPrintInteger(FW_VERSION_PATCH, 10, 1);
      uartPrint(" (");
      uartPrint(__REVISION__);
      uartPrint(")");
      uartPrint("\r\n");
      uartPrint("# UID ");
      uartPrintInteger(U_ID_2, 16, 1);
      uartPrintInteger(U_ID_1, 16, 1);
      uartPrintInteger(U_ID_0, 16, 1);
      uartPrint("\r\n");
      #if (defined(USE_BOOTLOADER))
        uartPrint("# BL ");
      #else
        uartPrint("# ");
      #endif
      #if (!defined(USE_PWM_FREQUENCY_48kHz))
        uartPrint("24kHz ");
      #else
        uartPrint("48kHz ");
      #endif
      if (motor.Direction == SPIN_CW) {
        uartPrint("CW ");
      } else {
        uartPrint("CCW ");
      }
      if (escConfig()->motor3Dmode == true) {
        uartPrint("3D ");
      }
      if ((escConfig()->motorCommutationDelay > 0) &&  (escConfig()->motorCommutationDelay <= 30)) {
        uartPrint("[");
        uartPrintInteger(escConfig()->motorCommutationDelay, 10, 1);
        uartPrint(" DEG] ");
      } else {
        uartPrint("AUTOTIMING ");
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
      if (!motor.Running) {
        motorTuneInput(6);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SETTING_SPIN_DIRECTION_NORMAL:
      if (!motor.Running) {
        escConfig()->motorDirection = SPIN_CW;
        uartPrint("# CW");
        uartPrint("\r\n");
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SETTING_SPIN_DIRECTION_REVERSED:
      if (!motor.Running) {
        escConfig()->motorDirection = SPIN_CCW;
        uartPrint("# CCW");
        uartPrint("\r\n");
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
      motor.Direction = escConfig()->motorDirection;
      break;
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
      motor.Direction = !escConfig()->motorDirection;
      break;
    case DSHOT_CMD_SETTING_3D_MODE_OFF:
      if (!motor.Running) {
        escConfig()->motor3Dmode = false;
        uartPrint("# 3D OFF");
        uartPrint("\r\n");
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SETTING_3D_MODE_ON:
      if (!motor.Running) {
        escConfig()->motor3Dmode = true;
        uartPrint("# 3D ON");
        uartPrint("\r\n");
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SETTING_SAVE:
      if (!motor.Running) {
        uartPrint("# SAVE");
        uartPrint("\r\n");
        HAL_Delay(11);
        configWrite();
        // reset esc, iwdg timeout
        while(true);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    case DSHOT_CMD_SETTING_EEPROM_RESET:
      if (!motor.Running) {
        uartPrint("# EEPROM RESET");
        uartPrint("\r\n");
        HAL_Delay(11);
        configReset();
        // reset esc, iwdg timeout
        while(true);
      } else {
        uartPrint("# Ko->motor.Running");
        uartPrint("\r\n");
      }
      break;
    }
  }
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void inputCallbackDMA() {
  switch (input.Protocol) {
    case PROSHOT1000:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
      inputProshot();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_PROSHOT1000);
      break;
    case DSHOT600:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
      inputDshot();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_DSHOT);
      break;
    case DSHOT300:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
      inputDshot();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_DSHOT);
      break;
    case SERVOPWM:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
      inputServoPwm();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_PWM);
      break;
    case AUTODETECT:
    default:
      HAL_TIM_IC_Stop_DMA(&inputTimerHandle, INPUT_TIMER_CH);
      inputAutoDetect();
      HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_AUTODETECT);
      break;
  }
}
#pragma GCC pop_options

void inputAutoDetect() {
  #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
    LED_OFF(LED_GREEN);
  #endif

  uint32_t  pulseWidthHi = inputDmaBuffer[1] - inputDmaBuffer[0];
  uint32_t  pulseWidthLo = inputDmaBuffer[2] - inputDmaBuffer[1];

  if (((pulseWidthHi >= (INPUT_PROSHOT1000_WIDTH_HI_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthHi <= (INPUT_PROSHOT1000_WIDTH_HI_MAX / (INPUT_AUTODETECT_PRESCALER + 1)))) &&
      ((pulseWidthLo >= (INPUT_PROSHOT1000_WIDTH_LO_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthLo <= (INPUT_PROSHOT1000_WIDTH_LO_MAX / (INPUT_AUTODETECT_PRESCALER + 1))))) {

    input.Protocol = PROSHOT1000;
    inputTimerHandle.Instance->PSC = INPUT_PROSHOT1000_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_PROSHOT1000);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
      LED_ON(LED_GREEN);
    #endif

    return;
  }

  if (((pulseWidthHi >= (INPUT_DSHOT600_WIDTH_HI_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthHi <= (INPUT_DSHOT600_WIDTH_HI_MAX / (INPUT_AUTODETECT_PRESCALER + 1)))) &&
      ((pulseWidthLo >= (INPUT_DSHOT600_WIDTH_LO_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthLo <= (INPUT_DSHOT600_WIDTH_LO_MAX / (INPUT_AUTODETECT_PRESCALER + 1))))) {

    input.Protocol = DSHOT600;
    inputTimerHandle.Instance->PSC = INPUT_DSHOT600_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_DSHOT);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
      LED_ON(LED_GREEN);
    #endif

    return;
  }

  if (((pulseWidthHi >= (INPUT_DSHOT300_WIDTH_HI_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthHi <= (INPUT_DSHOT300_WIDTH_HI_MAX / (INPUT_AUTODETECT_PRESCALER + 1)))) &&
      ((pulseWidthLo >= (INPUT_DSHOT300_WIDTH_LO_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthLo <= (INPUT_DSHOT300_WIDTH_LO_MAX / (INPUT_AUTODETECT_PRESCALER + 1))))) {

    input.Protocol = DSHOT300;
    inputTimerHandle.Instance->PSC = INPUT_DSHOT300_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_DSHOT);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
      LED_ON(LED_GREEN);
    #endif

    return;
  }

  if (((pulseWidthHi >= (INPUT_SERVOPWM_WIDTH_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthHi <= (INPUT_SERVOPWM_WIDTH_MAX / (INPUT_AUTODETECT_PRESCALER + 1)))) ||
      ((pulseWidthLo >= (INPUT_SERVOPWM_WIDTH_MIN / (INPUT_AUTODETECT_PRESCALER + 1))) && (pulseWidthLo <= (INPUT_SERVOPWM_WIDTH_MAX / (INPUT_AUTODETECT_PRESCALER + 1))))) {

    input.Protocol = SERVOPWM;
    inputTimerHandle.Instance->PSC = INPUT_PWM_PRESCALER;
    inputTimerHandle.Instance->CNT = 0xffff;
    HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_PWM);

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_AUTODETECT))
      LED_ON(LED_GREEN);
    #endif

    return;
  }

  // default
  input.Protocol = AUTODETECT;
  inputTimerHandle.Instance->PSC = INPUT_AUTODETECT_PRESCALER;
  inputTimerHandle.Instance->CNT = 0xffff;
  HAL_TIM_IC_Start_DMA(&inputTimerHandle, INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_AUTODETECT);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void inputProshot() {
  __disable_irq();
  uint8_t pulseValue[4] = {0, 0, 0, 0};
  uint8_t crcComputed = 0, crcReceived = 0;
  uint16_t data = 0;

  #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT1000))
    LED_ON(LED_GREEN);
  #endif

  for (int i = 0; i < 4; i++) {
    pulseValue[i] = ((inputDmaBuffer[i*2 + 1] - inputDmaBuffer[i*2]) - 45) / 6;
  }

  for (int i = 0; i < 4; i++) {
    crcComputed = crcComputed | ((pulseValue[0]^pulseValue[1]^pulseValue[2]) << i);
    crcReceived = crcReceived | (pulseValue[3] << i);
  }

  data = ((pulseValue[0] << 7 | pulseValue[1] << 3 | pulseValue[2] >> 1));

  if ((crcComputed == crcReceived) && (data >= INPUT_VALUE_MIN) && (data <= INPUT_VALUE_MAX)) {
    input.DataValid = true;
    input.DataValidCounter++;
    input.TimeoutCounter = 0;
    input.Data = data;
    __enable_irq();
    motorInputUpdate();

    // only update if not active
    if (!input.RequestTelemetry) {
      input.RequestTelemetry = (pulseValue[2] & BIT(0));
    }

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT1000))
      LED_OFF(LED_GREEN);
    #endif

    return;
  } else {
    input.DataValid = false;
    input.DataErrorCounter++;
    __enable_irq();

    #if (defined(_DEBUG_) && defined(DEBUG_INPUT_PROSHOT1000))
      LED_OFF(LED_GREEN);
    #endif

    return;
  }
}

void inputDshot() {
  __disable_irq();
  uint8_t pulseValue[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t crcComputed = 0, crcReceived = 0;
  uint16_t data = 0;

  for (int i = 0; i < 32; i+=2) {
    uint32_t tmp = (inputDmaBuffer[i + 1] - inputDmaBuffer[i]);
    if (( tmp > INPUT_DSHOT_WIDTH_HI_MIN) && (tmp < INPUT_DSHOT_WIDTH_HI_MAX)) {
      pulseValue[i >> 1] = 1;
    }
  }

  crcComputed = ( (pulseValue[0] ^ pulseValue[4] ^ pulseValue[8]) << 3 | (pulseValue[1] ^ pulseValue[5] ^ pulseValue[9]) << 2 |
                    (pulseValue[2] ^ pulseValue[6] ^ pulseValue[10]) << 1  | (pulseValue[3] ^ pulseValue[7] ^ pulseValue[11]));

  crcReceived = (pulseValue[12] << 3 | pulseValue[13] << 2 | pulseValue[14] << 1 | pulseValue[15]);

  if((crcComputed == crcReceived)  && (data >= INPUT_VALUE_MIN) && (data <= INPUT_VALUE_MAX)) {
    data = (pulseValue[0] << 10 | pulseValue[1] << 9 | pulseValue[2] << 8 | pulseValue[3] << 7 | pulseValue[4] << 6 |
            pulseValue[5] << 5 | pulseValue[6] << 4 | pulseValue[7] << 3 | pulseValue[8] << 2 | pulseValue[9] << 1 | pulseValue[10]);

    input.DataValid = true;
    input.DataValidCounter++;
    input.TimeoutCounter = 0;
    input.Data = data;
    __enable_irq();
    motorInputUpdate();

    // only update if not active
    if (!input.RequestTelemetry) {
      input.RequestTelemetry = pulseValue[11];
    }

    return;
  } else {
    input.DataValid = false;
    input.DataErrorCounter++;
    __enable_irq();

    return;
  }
}

// SERVOPWM (used only for thrust tests ...)
void inputServoPwm() {
  #if (defined(_DEBUG_))
    __disable_irq();
    uint32_t pulseWidthBuffer = 0;

    for (int i = 0; i < (INPUT_DMA_BUFFER_SIZE_PWM - 1); i++) {
      pulseWidthBuffer = inputDmaBuffer[i + 1] - inputDmaBuffer[i];
      if ((pulseWidthBuffer >= (INPUT_SERVOPWM_WIDTH_MIN_US - 50 )) && (pulseWidthBuffer <= (INPUT_SERVOPWM_WIDTH_MAX_US + 100))) {
        input.DataValid = true;
        input.DataValidCounter++;
        input.TimeoutCounter = 0;
        input.Data = (pulseWidthBuffer - INPUT_SERVOPWM_WIDTH_MIN_US) << 2;
        __enable_irq();
        motorInputUpdate();
        return;
      } else {
        input.DataValid = false;
        input.DataErrorCounter++;
        __enable_irq();
      }
    }
  #endif
}
#pragma GCC pop_options
