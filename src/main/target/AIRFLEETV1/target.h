#pragma once

// debug
//#define _DEBUG_
//#define DEBUG_CYCLETIME_MAINLOOP
//#define DEBUG_MOTOR_TIMING
//#define DEBUG_INPUT_PROSHOT
//#define DEBUG_INPUT_AUTODETECT
//#define DEBUG_DATA_UART
//#define DEBUG_MS_TIMER
//#define DEBUG_DATA_QUALITY

// ToDo new
// CYCLETIME_MAINLOOP kalman->400us (+-10%), median->150us (+-10%)
// PWM_FREQUENCY 48kHz resolution 500 steps, 24kHz resolution 1000 steps
//#define USE_PWM_FREQUENCY_48kHz
#define USE_RPM_MEDIAN
#define USE_ADC_MEDIAN

// hw constants
#define HBRIDGE_DEAD_TIME       11     // (in 20.833ns cycles at 48MHz) (FD6288 has a builtin 200ns deadtime)
#define HBRIDGE_MAX_CURRENT     1111  // (in 10mA steps)
#define HBRIDGE_MAX_TEMPERATURE 77    // °C
#if (!defined(USE_PWM_FREQUENCY_48kHz))
  #define TIMER1_INIT_PERIOD    1001
  #define MOTOR_START_THRESHOLD 21
#else
  #define TIMER1_INIT_PERIOD    501
  #define MOTOR_START_THRESHOLD 13
#endif
#if (defined(USE_RPM_MEDIAN))
  #if (!defined(USE_PWM_FREQUENCY_48kHz))
    #define RPM_CONSTANT     7616032
  #else
    #define RPM_CONSTANT     7235231
  #endif
#else
  #define RPM_CONSTANT       7744820
#endif
#define MOTOR_POLES             14

// input ,begin
#define INPUT_GPIO      GPIOA
#define INPUT_PIN       GPIO_PIN_3
#define INPUT_TIMER     TIM2
#define INPUT_TIMER_CH  TIM_CHANNEL_4

// LEDs ,ko
#define LED_RED_GPIO    GPIOA
#define LED_RED_PIN     GPIO_PIN_15
#define LED_GREEN_GPIO  GPIOA
#define LED_GREEN_PIN   GPIO_PIN_15
#define LED_BLUE_GPIO   GPIOA
#define LED_BLUE_PIN    GPIO_PIN_15
#define LED_MASK        LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB

// H_BRIDGE ,done
#define STSPIN32F0   // FD6288
#define A_FET_HI_GPIO   GPIOA
#define A_FET_HI_PIN    GPIO_PIN_8
#define B_FET_HI_GPIO   GPIOA
#define B_FET_HI_PIN    GPIO_PIN_9
#define C_FET_HI_GPIO   GPIOA
#define C_FET_HI_PIN    GPIO_PIN_10
#define A_FET_LO_GPIO   GPIOB
#define A_FET_LO_PIN    GPIO_PIN_13
#define B_FET_LO_GPIO   GPIOB
#define B_FET_LO_PIN    GPIO_PIN_14
#define C_FET_LO_GPIO   GPIOB
#define C_FET_LO_PIN    GPIO_PIN_15

//OpAmpEXTI ,done
#define OPAMP_EXTI_GPIO GPIOA
#define OPAMP_EXTI_PINA GPIO_PIN_0
#define OPAMP_EXTI_PINB GPIO_PIN_1
#define OPAMP_EXTI_PINC GPIO_PIN_2

// adc
//#define USE_ADC
#define ADC_VOLTAGE             ADC_CHANNEL_3
#define ADC_CURRENT             ADC_CHANNEL_6
#define ADC_TEMPERATURE         ADC_CHANNEL_TEMPSENSOR
#define ADC_MASK                GPIO_PIN_3 | GPIO_PIN_6

// adc calibration
#define ADC_VOLTAGE_OFFSET      40.5
#define ADC_VOLTAGE_FACTOR      0.595
#define ADC_CURRENT_OFFSET      0  // 0 -> auto offset
#define ADC_CURRENT_FACTOR      1.87
#define ADC_CONSUMPTION_FACTOR  0.00028
#define ADC_TEMPERATURE_OFFSET  95.05
#define ADC_TEMPERATURE_FACTOR  -0.04

// telemetry
#define USART                   USART1
#define USART_IRQn              USART1_IRQn
#define USART_IRQHandler        USART1_IRQHandler
#define USART_TX_PIN            LL_GPIO_PIN_6
#define USART_TX_GPIO_PORT      GPIOB
#define USART_TX_GPIO_CLK       LL_AHB1_GRP1_PERIPH_GPIOB
#define USART_TX_AF             LL_GPIO_AF_0
#define USART_TDR_ADDRESS       0x40013828
#define USART_TX_DMA_CHANNEL    LL_DMA_CHANNEL_2
