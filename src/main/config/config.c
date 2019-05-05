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
  escConfig()->motorSlowDecay = true;
  escConfig()->motorBrake = BRAKE_FULL;
  escConfig()->motorBrakeStrength = 300;
  escConfig()->motorStartThreshold = 37;
  escConfig()->motor3Dmode = false;
  escConfig()->input3Dneutral = 1000;
  escConfig()->input3DdeadbandLow = 990;
  escConfig()->input3DdeadbandHigh = 1010;
  escConfig()->led0 = false;
  escConfig()->led1 = false;
  escConfig()->led2 = false;

  eepromWrite();
}

void configRead(void) {
  eepromRead();
}

void configWrite(void) {
  eepromWrite();
  //eepromRead();
}
