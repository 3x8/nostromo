#pragma once

// debug
//#define DEBUG_CYCLETIME_MAINLOOP
//#define DEBUG_ZERO_CROSS
//#define DEBUG_INPUT_PROSHOT
//#define DEBUG_INPUT_AUTODETECT

//ToDo
#define HBRIDGE_DEAD_TIME 7 // in 22ns cycles
#define HBRIDGE_MAX_CURRENT 2503 // ~66A  ... 30steps/1A  (at 35A no offset compensation)
#define HBRIDGE_MAX_TEMPERATURE 1234 // not used
#define TIMER1_INIT_PERIOD  1103
#define MOTOR_START_THRESHOLD 11

// PA15 --- LED RED ,Ok
#define RED_GPIO       GPIOA
#define RED_PIN        GPIO_PIN_15
#define RED_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOA

// PB3  --- LED GREEN ,Ok
#define GREEN_GPIO       GPIOB
#define GREEN_PIN        GPIO_PIN_3
#define GREEN_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PB4  --- LED BLUE ,Ok
#define BLUE_GPIO       GPIOB
#define BLUE_PIN        GPIO_PIN_4
#define BLUE_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PA10 --- Phase_A_HI ,Ok
#define A_FET_HI_GPIO   GPIOA
#define A_FET_HI_PIN    GPIO_PIN_10

// PA9 --- Phase_B_HI ,Ok
#define B_FET_HI_GPIO   GPIOA
#define B_FET_HI_PIN    GPIO_PIN_9

// PA8 --- Phase_C_HI ,Ok
#define C_FET_HI_GPIO   GPIOA
#define C_FET_HI_PIN    GPIO_PIN_8

// PB1 --- Phase_A_LO ,Ok
#define A_FET_LO_GPIO   GPIOB
#define A_FET_LO_PIN    GPIO_PIN_1

// PB0 --- Phase_B_LO ,Ok
#define B_FET_LO_GPIO   GPIOB
#define B_FET_LO_PIN    GPIO_PIN_0

// PA7 --- Phase_C_LO ,Ok
#define C_FET_LO_GPIO   GPIOA
#define C_FET_LO_PIN    GPIO_PIN_7

// comparator ,Ok
#define COMPARATOR_PHASE_A       COMP_INVERTINGINPUT_IO1
#define COMPARATOR_PHASE_B       COMP_INVERTINGINPUT_DAC1
#define COMPARATOR_PHASE_C       COMP_INVERTINGINPUT_DAC2

// ADC ,Ok
#define ADC_VOLTAGE              ADC_CHANNEL_3
#define GPIO_PIN_VOLTAGE         GPIO_PIN_3
#define ADC_CURRENT              ADC_CHANNEL_6
#define GPIO_PIN_CURRENT         GPIO_PIN_6
#define ADC_TEMPERATURE          ADC_CHANNEL_TEMPSENSOR
