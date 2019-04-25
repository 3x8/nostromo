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
  escConfig()->motorDirection = 1;
  escConfig()->motorSlowDecay = 1;
  escConfig()->motor3Dmode = 0;
  escConfig()->motorBrakeState = BRAKE_FULL;
  escConfig()->motorBrakeProportionalStrength = 300;
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
