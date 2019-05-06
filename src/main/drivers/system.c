#include "system.h"

extern ADC_HandleTypeDef adcHandle;
extern COMP_HandleTypeDef comparator1Handle;
extern TIM_HandleTypeDef timer1Handle, timer2Handle, timer3Handle, timer15Handle;

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


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

  // Configure the Systick interrupt time
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  // Configure the Systick
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void systemDmaInit(void) {
  // DMA controller clock enable
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA interrupt init
  // DMA1_Channel1_IRQn interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  // DMA1_Channel4_5_IRQn interrupt configuration
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
  adcHandle.Init.ContinuousConvMode = DISABLE;
  adcHandle.Init.DiscontinuousConvMode = ENABLE;
  adcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;
  adcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  adcHandle.Init.DMAContinuousRequests = ENABLE;
  adcHandle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  while (HAL_ADC_Init(&adcHandle) != HAL_OK);

  // ADC channels to be converted.
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

  sConfig.Channel = ADC_VOLTAGE;
  while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);

  sConfig.Channel = ADC_CURRENT;
  while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);

  sConfig.Channel = ADC_TEMPERATURE;
  while (HAL_ADC_ConfigChannel(&adcHandle, &sConfig) != HAL_OK);
}

void systemComparator1Init(void) {
  comparator1Handle.Instance = COMP1;
  comparator1Handle.Init.InvertingInput = COMPARATOR_PHASE_A;
  comparator1Handle.Init.Output = COMP_OUTPUT_NONE;
  comparator1Handle.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  comparator1Handle.Init.Hysteresis = COMP_HYSTERESIS_LOW;
  comparator1Handle.Init.Mode = COMP_MODE_HIGHSPEED;
  comparator1Handle.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  comparator1Handle.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  while (HAL_COMP_Init(&comparator1Handle) != HAL_OK);
}

void systemTimer1Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  timer1Handle.Instance = TIM1;
  timer1Handle.Init.Prescaler = 0;
  timer1Handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  //ToDo
  //timer1Handle.Init.Period = 999; // default
  timer1Handle.Init.Period = 827; // WRAITH32MINI
  //timer1Handle.Init.Period = 727; // WRAITH32MINI 12.0701967138822 RPM peakout ???
  //timer1Handle.Init.Period = 811; // WRAITH32MINI 11.7675238742349N
  //timer1Handle.Init.Period = 1001; // WRAITH32MINI 10.803032776985N
  timer1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer1Handle.Init.RepetitionCounter = 0;
  timer1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&timer1Handle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&timer1Handle, &sClockSourceConfig) != HAL_OK);

  while (HAL_TIM_PWM_Init(&timer1Handle) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&timer1Handle, &sMasterConfig) != HAL_OK);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  while (HAL_TIM_PWM_ConfigChannel(&timer1Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(&timer1Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(&timer1Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(&timer1Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = HBRIDGE_DEAD_TIME;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  while (HAL_TIMEx_ConfigBreakDeadTime(&timer1Handle, &sBreakDeadTimeConfig) != HAL_OK);

  HAL_TIM_MspPostInit(&timer1Handle);
}

void systemTimer2Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  timer2Handle.Instance = TIM2;
  timer2Handle.Init.Prescaler = 100;
  timer2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer2Handle.Init.Period = 5000;
  timer2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&timer2Handle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&timer2Handle, &sClockSourceConfig) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&timer2Handle, &sMasterConfig) != HAL_OK);
}

void systemTimer3Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  timer3Handle.Instance = TIM3;
  timer3Handle.Init.Prescaler = 10;
  timer3Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer3Handle.Init.Period = 65535;
  timer3Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&timer3Handle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&timer3Handle, &sClockSourceConfig) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&timer3Handle, &sMasterConfig) != HAL_OK);
}

void systemTimer15Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  timer15Handle.Instance = TIM15;
  timer15Handle.Init.Prescaler = 0;
  timer15Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  timer15Handle.Init.Period = 0xffff;
  timer15Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer15Handle.Init.RepetitionCounter = 0;
  timer15Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  while (HAL_TIM_Base_Init(&timer15Handle) != HAL_OK);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  while (HAL_TIM_ConfigClockSource(&timer15Handle, &sClockSourceConfig) != HAL_OK);
  while (HAL_TIM_IC_Init(&timer15Handle) != HAL_OK);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  while (HAL_TIMEx_MasterConfigSynchronization(&timer15Handle, &sMasterConfig) != HAL_OK);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  while (HAL_TIM_IC_ConfigChannel(&timer15Handle, &sConfigIC, TIM_CHANNEL_1) != HAL_OK);
}
