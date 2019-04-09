#pragma once

#define EEPROM_CONF_VERSION 1

void eepromWrite(void);
void eepromRead(void);
bool eepromValid(void);
