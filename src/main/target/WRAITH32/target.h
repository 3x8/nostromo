#pragma once

//ToDo
#define DEAD_TIME 60

//ToDo
#define MP6531
// #define FD6288

//ToDo
#define MOTOR_NUM_PHASES            3
#define MOTOR_NUM_COMMUTATION_STEPS 6

// PA15 --- LED RED ,Ok
#define RED_GPIO       GPIOA
#define RED_PIN        LL_GPIO_PIN_15
#define RED_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOA

// PB3  --- LED GREEN ,Ok
#define GREEN_GPIO       GPIOB
#define GREEN_PIN        LL_GPIO_PIN_3
#define GREEN_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PB4  --- LED BLUE ,Ok
#define BLUE_GPIO       GPIOB
#define BLUE_PIN        LL_GPIO_PIN_4
#define BLUE_PERIPHERAL LL_AHB1_GRP1_PERIPH_GPIOB

// PA10 --- Phase_A_HI ,Ok
#define A_FET_HI_GPIO   GPIOA
#define A_FET_HI_PIN    LL_GPIO_PIN_10

// PA9 --- Phase_B_HI
#define B_FET_HI_GPIO   GPIOA
#define B_FET_HI_PIN    LL_GPIO_PIN_9

// PA8 --- Phase_C_HI ,Ok
#define C_FET_HI_GPIO   GPIOA
#define C_FET_HI_PIN    LL_GPIO_PIN_8

// PB1 --- Phase_A_LO ,Ok
#define A_FET_LO_GPIO   GPIOB
#define A_FET_LO_PIN    LL_GPIO_PIN_1

// PB0 --- Phase_B_LO ,Ok
#define B_FET_LO_GPIO   GPIOB
#define B_FET_LO_PIN    LL_GPIO_PIN_0

// PA7 --- Phase_C_LO ,Ok
#define C_FET_LO_GPIO   GPIOA
#define C_FET_LO_PIN    LL_GPIO_PIN_7

//ToDo
// Sequence number of ADC each particular channel (the order in which they are scanned)
#define ADC_SEQ_TEMPERATURE       0
#define ADC_SEQ_PHASE_A           1
#define ADC_SEQ_VOLTAGE           3
#define ADC_SEQ_PHASE_B           4
#define ADC_SEQ_PHASE_C           5
#define ADC_SEQ_CURRENT           6

// ADC channel assignments
#define ADC_CHAN_TEMPERATURE      LL_ADC_CHANNEL_TEMPSENSOR // CH16
#define ADC_CHAN_PHASE_A          LL_ADC_CHANNEL_0
#define ADC_CHAN_VOLTAGE          LL_ADC_CHANNEL_3
#define ADC_CHAN_PHASE_B          LL_ADC_CHANNEL_4
#define ADC_CHAN_PHASE_C          LL_ADC_CHANNEL_5
#define ADC_CHAN_CURRENT          LL_ADC_CHANNEL_6

#define TARGET_ADC_CHANNEL_MASK ( ADC_CHAN_PHASE_A | \
                                  ADC_CHAN_PHASE_B | \
                                  ADC_CHAN_PHASE_C | \
                                  ADC_CHAN_TEMPERATURE | \
                                  ADC_CHAN_VOLTAGE | \
                                  ADC_CHAN_CURRENT )

#define GPIO_PIN_PHASE_A          LL_GPIO_PIN_0
#define GPIO_PIN_PHASE_B          LL_GPIO_PIN_4
#define GPIO_PIN_PHASE_C          LL_GPIO_PIN_5
#define GPIO_PIN_VOLTAGE          LL_GPIO_PIN_3
#define GPIO_PIN_CURRENT          LL_GPIO_PIN_6

#define TARGET_ADC_PIN_MASK     ( GPIO_PIN_PHASE_A | \
                                  GPIO_PIN_PHASE_B | \
                                  GPIO_PIN_PHASE_C | \
                                  GPIO_PIN_VOLTAGE | \
                                  GPIO_PIN_CURRENT )
