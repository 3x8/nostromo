#pragma once

#include "main.h"

#define escConfig(x) (&masterConfig.escConfig)

typedef struct escConfig_s {
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
} escConfig_t;

typedef struct master_s {
  uint8_t version;
  uint16_t size;
  uint8_t magic_be;
  escConfig_t escConfig;
  uint8_t magic_ef;
  uint8_t chk;
} master_t;

extern master_t masterConfig;

void configValidateOrReset(void);
void configReset(void);
void configRead(void);
void configWrite(void);
