#pragma once

#include "main.h"

#define escConfig(x) (&masterConfig.escConfig)

typedef struct escConfig_s {
  bool motorDirection;
  bool motorSlowDecay;
  uint8_t  motorBrake;
  uint16_t motorBrakeStrength;
  uint16_t motorStartThreshold;
  bool motor3Dmode;
  uint32_t input3Dneutral;
  uint32_t input3DdeadbandLow;
  uint32_t input3DdeadbandHigh;
  uint32_t limitCurrent;
  uint32_t limitTemperature;
  bool led0;
  bool led1;
  bool led2;
  int32_t adcCurrentOffset;
} escConfig_t;

typedef struct master_s {
  uint8_t version;
  uint16_t size;

  // magic number, should be 0xBE
  uint8_t magic_be;

  escConfig_t escConfig;

  // magic number, should be 0xEF
  uint8_t magic_ef;
  uint8_t chk;
} master_t;

extern master_t masterConfig;

void configValidateOrReset(void);
void configReset(void);
void configRead(void);
void configWrite(void);
