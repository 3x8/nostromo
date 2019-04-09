#include "main.h"

// master config struct with data independent from profiles
master_t masterConfig;

void configValidateOrReset(void) {
  if (eepromValid()) {
    return;
  }
  configReset();
}

void configReset(void) {
  memset(&masterConfig, 0, sizeof(master_t));
  masterConfig.version = EEPROM_CONF_VERSION;

  // assign default config
  escConfig()->vehicle_mode = 1;
  escConfig()->dir_reversed = 0;
  escConfig()->bi_direction = 0;

  eepromWrite();
}

void configRead(void) {
  eepromRead();
}

void configWrite(void) {
  eepromWrite();
  eepromRead();
}
