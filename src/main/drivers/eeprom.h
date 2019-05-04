#pragma once

#include "main.h"

#define EEPROM_CONF_VERSION 2

bool eepromValid(void);
void eepromRead(void);
void eepromWrite(void);
