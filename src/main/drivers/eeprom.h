#pragma once

#include "main.h"

#define EEPROM_VERSION 7

#define FLASH_PAGE_COUNT      64
#define FLASH_EEPROM_SIZE     0x400  // 1x1K page
#define FLASH_EEPROM_ADDRESS  (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_EEPROM_SIZE))

bool eepromValid(void);
void eepromRead(void);
void eepromWrite(void);
