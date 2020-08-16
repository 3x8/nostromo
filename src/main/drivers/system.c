#include "system.h"

TIM_HandleTypeDef msTimerHandle;
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timerHandle);

void systemClockConfig(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  // Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  while (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK);

  // Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  while (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK);

  // Configure the Systick
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void systemDmaInit(void) {
  // DMA controller clock enable
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

void systemAdcInit(void) {
  ADC_ChannelConfTypeDef sConfig;

  // configure global ADC features
  adcHandle.Instance = ADC1;
  adcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  adcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  adcHandle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  adcHandle.Init.LowPowerAutoWait = DISABLE;
  adcHandle.Init.LowPowerAutoPowerOff = DISABLE;
  adcHandle.Init.ContinuousConvMode = ENABLE;
  adcHandle.Init.DiscontinuousConvMode = DISABLE;
  adcHandle.Init.DMAContinuousRequests = ENABLE;
  adcHandle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  while (HAL_ADC_Init(&adcHandle) != HAL_OK);

  // ADC channels to be converted.
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

  #if (defined(USE_ADC))
    sConfig.Channel = ADC_CURRENT;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);
    sConfig.Channel = ADC_VOLTAGE;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);
    sConfig.Channel = ADC_TEMPERATURE;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);

    while (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcDmaBuffer, 3) != HAL_OK);
  #endif

  #if (defined(WRAITH32MINI))
    sConfig.Channel = ADC_TEMPERATURE;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);

    while (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcDmaBuffer, 1) != HAL_OK);
  #endif

  #if (defined(DYS35ARIA))
    sConfig.Channel = ADC_CURRENT;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);
    sConfig.Channel = ADC_TEMPERATURE;
    while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);

    while (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcDmaBuffer, 2) != HAL_OK);
  #endif
}

void systemBemfComparatorInit(void) {
  motorBemfComparatorHandle.Instance = COMPARATOR;
  motorBemfComparatorHandle.Init.NonInvertingInput = COMPARATOR_COMMON;
  motorBemfComparatorHandle.Init.InvertingInput = COMPARATOR_PHASE_A;
  motorBemfComparatorHandle.Init.Output = COMP_OUTPUT_NONE;
  motorBemfComparatorHandle.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  motorBemfComparatorHandle.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  motorBemfComparatorHandle.Init.Mode = COMP_MODE_HIGHSPEED;
  motorBemfComparatorHandle.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  motorBemfComparatorHandle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  while (HAL_COMP_Init(&motorBemfComparatorHandle) != HAL_OK);

  while (HAL_COMP_Start_IT(&motorBemfComparatorHandle) != HAL_OK);
}

void systemMotorPwmTimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  motorPwmTimerHandle.Instance = TIM1;
  motorPwmTimerHandle.Init.Prescaler = 0;
  #if (defined(USE_PWM_FREQUENCY_48kHz))
    motorPwmTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  #else
    motorPwmTimerHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  #endif
  motorPwmTimerHandle.Init.Period = MOTOR_PWM_RESOLUTION;
  motorPwmTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  motorPwmTimerHandle.Init.RepetitionCounter = 0;
  motorPwmTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&motorPwmTimerHandle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&motorPwmTimerHandle, &sClockSourceConfig) != HAL_OK);

  while (HAL_TIM_PWM_Init(&motorPwmTimerHandle) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&motorPwmTimerHandle, &sMasterConfig) != HAL_OK);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  while (HAL_TIM_PWM_ConfigChannel(&motorPwmTimerHandle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(&motorPwmTimerHandle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(&motorPwmTimerHandle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = HBRIDGE_DEAD_TIME;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  while (HAL_TIMEx_ConfigBreakDeadTime(&motorPwmTimerHandle, &sBreakDeadTimeConfig) != HAL_OK);

  HAL_TIM_MspPostInit(&motorPwmTimerHandle);

  while (HAL_TIM_PWM_Start(&motorPwmTimerHandle, TIM_CHANNEL_1) != HAL_OK);
  while (HAL_TIMEx_PWMN_Start(&motorPwmTimerHandle, TIM_CHANNEL_1) != HAL_OK);
  while (HAL_TIM_PWM_Start(&motorPwmTimerHandle, TIM_CHANNEL_2) != HAL_OK);
  while (HAL_TIMEx_PWMN_Start(&motorPwmTimerHandle, TIM_CHANNEL_2) != HAL_OK);
  while (HAL_TIM_PWM_Start(&motorPwmTimerHandle, TIM_CHANNEL_3) != HAL_OK);
  while (HAL_TIMEx_PWMN_Start(&motorPwmTimerHandle, TIM_CHANNEL_3) != HAL_OK);
}

void systemMotorCommutationTimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  motorCommutationTimerHandle.Instance = TIM14;
  motorCommutationTimerHandle.Init.Prescaler = 7;
  motorCommutationTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  motorCommutationTimerHandle.Init.Period = 0xffff;
  motorCommutationTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  motorCommutationTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&motorCommutationTimerHandle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&motorCommutationTimerHandle, &sClockSourceConfig) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&motorCommutationTimerHandle, &sMasterConfig) != HAL_OK);

  while (HAL_TIM_Base_Start(&motorCommutationTimerHandle) != HAL_OK);
}

void systemInputTimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  inputTimerHandle.Instance = INPUT_TIMER;
  inputTimerHandle.Init.Prescaler = 0;
  inputTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  inputTimerHandle.Init.Period = 0xffff;
  inputTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  inputTimerHandle.Init.RepetitionCounter = 0;
  inputTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&inputTimerHandle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&inputTimerHandle, &sClockSourceConfig) != HAL_OK);
  while (HAL_TIM_IC_Init(&inputTimerHandle) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&inputTimerHandle, &sMasterConfig) != HAL_OK);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  while (HAL_TIM_IC_ConfigChannel(&inputTimerHandle, &sConfigIC, INPUT_TIMER_CH) != HAL_OK);

  while (HAL_TIM_IC_Start_DMA(&inputTimerHandle,INPUT_TIMER_CH, inputDmaBuffer, INPUT_DMA_BUFFER_SIZE_AUTODETECT) != HAL_OK);
}

void systemMsTimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  msTimerHandle.Instance = TIM16;
  msTimerHandle.Init.Prescaler = 24001;   // 500us
  msTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  msTimerHandle.Init.Period = 0xffffffff;
  msTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  msTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&msTimerHandle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&msTimerHandle, &sClockSourceConfig) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&msTimerHandle, &sMasterConfig) != HAL_OK);

  while (HAL_TIM_Base_Start(&msTimerHandle) != HAL_OK);
}

// ToDo
void systemMotorSinTimerInit(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  motorSinTimerHandle.Instance = TIM6;
  motorSinTimerHandle.Init.Prescaler = 10;
  motorSinTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  motorSinTimerHandle.Init.Period = 4000;
  motorSinTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  motorSinTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  while (HAL_TIM_Base_Init(&motorSinTimerHandle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&motorSinTimerHandle, &sClockSourceConfig) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&motorSinTimerHandle, &sMasterConfig) != HAL_OK);

  while (HAL_TIM_Base_Start_IT(&motorSinTimerHandle) != HAL_OK);

}

#if (defined(USE_BOOTLOADER))
  void systemInitAfterBootloaderJump() {
    volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;

    for(uint8_t i = 0; i < 48; i++) {
      VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i << 2));
    }
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
    __enable_irq();
  }
#endif
