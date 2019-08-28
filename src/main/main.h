#pragma once

#include <stdbool.h>
#include <string.h>

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_comp.h"
#include "stm32f0xx_hal_adc.h"

#include "stm32f0xx.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_adc.h"

#include "target.h"
#include "build/version.h"
#include "common/common.h"
#include "config/config.h"
#include "drivers/adc.h"
#include "drivers/led.h"
#include "drivers/eeprom.h"
#include "drivers/watchdog.h"
#include "drivers/system.h"
#include "drivers/input.h"
#include "drivers/motor.h"
#include "drivers/uart.h"
#include "drivers/telemetry.h"
