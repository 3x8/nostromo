#pragma once

#include "main.h"

#define escConfig(x) (&masterConfig.escConfig)

typedef struct escConfig_s {
  uint8_t spinDirection;
  uint8_t mode3D;
  uint8_t led0;
  uint8_t led1;
  uint8_t led2;
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
