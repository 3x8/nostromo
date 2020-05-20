#include "config.h"

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

  // default config
  escConfig()->motorDirection = SPIN_CW;
  escConfig()->motorComplementaryPWM = true;
  escConfig()->motorBrake = BRAKE_FULL;
  escConfig()->motorBrakeStrength = 501;
  escConfig()->motorStartThreshold = MOTOR_START_THRESHOLD;
  escConfig()->motor3Dmode = false;
  escConfig()->input3Dneutral = 1000;
  escConfig()->input3DdeadbandLow = 1;
  escConfig()->input3DdeadbandHigh = 1001;
  escConfig()->limitCurrent = HBRIDGE_MAX_CURRENT;
  escConfig()->limitTemperature = HBRIDGE_MAX_TEMPERATURE;
  escConfig()->led0 = false;
  escConfig()->led1 = false;
  escConfig()->led2 = false;
  escConfig()->adcCurrentOffset = ADC_CURRENT_OFFSET;

  eepromWrite();
}

void configRead(void) {
  eepromRead();
}

void configWrite(void) {
  eepromWrite();
}
