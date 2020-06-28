#pragma once

#include "main.h"

#define escConfig(x) (&masterConfig.escConfig)

typedef struct {
  uint8_t motorDirection;
  uint8_t motorComplementaryPWM;
  uint8_t  motorBrake;
  uint16_t motorBrakeStrength;
  uint16_t motorStartThreshold;
  uint8_t motor3Dmode;
  uint32_t input3Dneutral;
  uint32_t input3DdeadbandLow;
  uint32_t input3DdeadbandHigh;
  uint32_t limitCurrent;
  uint32_t limitTemperature;
  int32_t adcCurrentOffset;
  uint8_t motorPoles;
} escConfigStructure;

typedef struct {
  uint8_t version;
  uint16_t size;
  uint8_t magic_be;
  escConfigStructure escConfig;
  uint8_t magic_ef;
  uint8_t chk;
} eepromStructure;

extern eepromStructure masterConfig;

void configValidateOrReset(void);
void configReset(void);
void configRead(void);
void configWrite(void);
