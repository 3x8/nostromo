#pragma once

// changed MCU to stm32f051
// bridge PB4 to PA2 input (SERVOPWM not working)

// debug
//#define _DEBUG_
//#define DEBUG_CYCLETIME_MAINLOOP // ~500us
//#define DEBUG_MOTOR_TIMING
//#define DEBUG_INPUT_PROSHOT
//#define DEBUG_INPUT_AUTODETECT
//#define DEBUG_DATA_UART
//#define DEBUG_MS_TIMER

// ToDo
#define HBRIDGE_DEAD_TIME       0     // (in 21.26ns cycles ~47.017MHz) (FD6288 has a builtin 200ns deadtime)
#define HBRIDGE_MAX_CURRENT     0  // (in 10mA steps)
#define HBRIDGE_MAX_TEMPERATURE 0    // °C
#define TIMER1_INIT_PERIOD      977   // max linearity //797~30.03kHz //911~26.31kHz //977~24.4kHz //997~24.1kHz
#define MOTOR_START_THRESHOLD   13

// input ,Ok
#define INPUT_GPIO      GPIOB
#define INPUT_PIN       GPIO_PIN_4
#define INPUT_TIMER     TIM3

// LEDs ,no
#define LED_RED_GPIO    GPIOA
#define LED_RED_PIN     GPIO_PIN_15
#define LED_GREEN_GPIO  GPIOA
#define LED_GREEN_PIN   GPIO_PIN_15
#define LED_BLUE_GPIO   GPIOA
#define LED_BLUE_PIN    GPIO_PIN_15
#define LED_MASK        LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB

// H_BRIDGE ,Ok
#define A_FET_HI_GPIO   GPIOA
#define A_FET_HI_PIN    GPIO_PIN_10
#define B_FET_HI_GPIO   GPIOA
#define B_FET_HI_PIN    GPIO_PIN_9
#define C_FET_HI_GPIO   GPIOA
#define C_FET_HI_PIN    GPIO_PIN_8
#define A_FET_LO_GPIO   GPIOB
#define A_FET_LO_PIN    GPIO_PIN_1
#define B_FET_LO_GPIO   GPIOB
#define B_FET_LO_PIN    GPIO_PIN_0
#define C_FET_LO_GPIO   GPIOA
#define C_FET_LO_PIN    GPIO_PIN_7

// comparator ,Ok
#define COMPARATOR_PHASE_A      COMP_INVERTINGINPUT_IO1
#define COMPARATOR_PHASE_B      COMP_INVERTINGINPUT_DAC1
#define COMPARATOR_PHASE_C      COMP_INVERTINGINPUT_DAC2
#define COMPARATOR_COMMON       COMP_NONINVERTINGINPUT_IO1
#define COMPARATOR_MASK         GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5

// adc ,Ok  I->Ko
#define ADC_VOLTAGE             ADC_CHANNEL_3
#define ADC_CURRENT             ADC_CHANNEL_6
#define ADC_TEMPERATURE         ADC_CHANNEL_TEMPSENSOR
#define ADC_MASK                GPIO_PIN_3 | GPIO_PIN_6

// adc calibration ,Ok  I->Ko
#define ADC_VOLTAGE_OFFSET      59.5
#define ADC_VOLTAGE_FACTOR      0.888
#define ADC_CURRENT_OFFSET      1.0  // 0 -> auto offset
#define ADC_CURRENT_FACTOR      1.0
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
