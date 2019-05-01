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
  escConfig()->motorDirection = 0;
  escConfig()->motorSlowDecay = 1;
  escConfig()->motorBrake = BRAKE_FULL;
  escConfig()->motorBrakeStrength = 300;
  escConfig()->motor3Dmode = 0;
  escConfig()->input3Dneutral = 1000;
  escConfig()->input3DdeadbandLow = 990;
  escConfig()->input3DdeadbandHigh = 1010;
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
