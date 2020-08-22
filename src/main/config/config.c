#include "config.h"

eepromStructure masterConfig;

void configValidateOrReset(void) {
  if (eepromValid()) {
    return;
  } else {
    configReset();
  }
}

void configReset(void) {
  memset(&masterConfig, 0, sizeof(eepromStructure));

  // default config
  escConfig()->motorPoles = MOTOR_POLES;
  escConfig()->motorDirection = SPIN_CW;
  escConfig()->motorComplementaryPWM = true;
  escConfig()->motorBrake = BRAKE_FULL;
  escConfig()->motorStartThreshold = MOTOR_START_THRESHOLD;
  escConfig()->motorStartupPower = MOTOR_STARTUP_POWER;
  escConfig()->motor3Dmode = false;
  escConfig()->input3Dneutral = 1000;
  escConfig()->input3DdeadbandLow = 5;
  escConfig()->input3DdeadbandHigh = 1005;
  escConfig()->limitCurrent = HBRIDGE_MAX_CURRENT;
  escConfig()->limitTemperature = HBRIDGE_MAX_TEMPERATURE;
  escConfig()->adcCurrentOffset = ADC_CURRENT_OFFSET;

  eepromWrite();
}

void configRead(void) {
  eepromRead();
}

void configWrite(void) {
  eepromWrite();
}
