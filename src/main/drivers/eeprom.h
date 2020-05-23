#pragma once

#include "main.h"

#define EEPROM_CONF_VERSION 3

#define FLASH_PAGE_COUNT                64
#define FLASH_TO_RESERVE_FOR_CONFIG     0x400  // 1K page
#define CONFIG_START_FLASH_ADDRESS      (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))


bool eepromValid(void);
void eepromRead(void);
void eepromWrite(void);
