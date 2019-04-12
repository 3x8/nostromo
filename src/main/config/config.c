#include "config.h"

// master config struct with data independent from profiles
master_t masterConfig;

void configValidateOrReset(void) {
  if (eepromValid()) {
    return;
  } else {
    configReset();
  }
}

void configReset(void) {
  memset(&masterConfig, 0, sizeof(master_t));
  masterConfig.version = EEPROM_CONF_VERSION;

  // assign default config
  escConfig()->spinDirection = 0;
  escConfig()->mode3D = 0;
  escConfig()->led0 = 0;
  escConfig()->led1 = 0;
  escConfig()->led2 = 0;

  eepromWrite();
}

void configRead(void) {
  eepromRead();
}

void configWrite(void) {
  eepromWrite();
  //eepromRead();
}
