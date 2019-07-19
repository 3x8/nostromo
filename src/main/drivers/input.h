#pragma once

#include "main.h"

#define INPUT_ARM_COUNTER_THRESHOLD  1000
#define INPUT_TIMEOUT_COUNTER_THRESHOLD  1000

#define INPUT_AUTODETECT_PRESCALER 1
#define INPUT_PROSHOT_PRESCALER 0
#define INPUT_PROSHOT_WIDTH_MIN_SYSTICKS 22
#define INPUT_PROSHOT_WIDTH_MAX_SYSTICKS 80

#define INPUT_PWM_PRESCALER 95
#define INPUT_PWM_WIDTH_MIN_US 490
#define INPUT_PWM_WIDTH_MAX_US 983 //1020

#define INPUT_VALUE_MIN 0
#define INPUT_VALUE_MAX 2047

#define INPUT_NORMED_MIN 0
#define INPUT_NORMED_MAX 2000

#define OUTPUT_PWM_MIN 0
#define OUTPUT_PWM_MAX TIMER1_INIT_PERIOD

#define INPUT_BUFFER_DMA_SIZE 9
#define INPUT_BUFFER_DMA_SIZE_PWM 3
#define INPUT_BUFFER_DMA_SIZE_PROSHOT 8
#define INPUT_BUFFER_DMA_SIZE_AUTODETECT 7

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SETTING_SPIN_DIRECTION_NORMAL,
    DSHOT_CMD_SETTING_SPIN_DIRECTION_REVERSED,
    DSHOT_CMD_SETTING_3D_MODE_OFF,
    DSHOT_CMD_SETTING_3D_MODE_ON,
    DSHOT_CMD_SETTING_REQUEST,
    DSHOT_CMD_SETTING_SAVE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_SETTING_LED0_ON,
    DSHOT_CMD_SETTING_LED1_ON,
    DSHOT_CMD_SETTING_LED2_ON,
    DSHOT_CMD_SETTING_LED3_ON,
    DSHOT_CMD_SETTING_LED0_OFF,
    DSHOT_CMD_SETTING_LED1_OFF,
    DSHOT_CMD_SETTING_LED2_OFF,
    DSHOT_CMD_SETTING_LED3_OFF,
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} imputCommandDshotsEnum;

typedef enum {
    AUTODETECT = 0,
    SERVOPWM,
    ONESHOT42,
    ONESHOT125,
    MULTISHOT,
    DSHOT,
    PROSHOT
} inputProtocolEnum;

void inputArmCheck(void);
void inputDisarm(void);
void inputDisarmCheck(void);

void inputDshotCommandRun(void);

void inputCallbackDMA();
void inputDetectProtocol();

void inputProshot();
void inputServoPwm();
