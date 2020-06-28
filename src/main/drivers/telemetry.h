#pragma once

#include "main.h"

// from https://www.rcgroups.com/forums/showatt.php?attachmentid=8524039&d=1450424877
/*
  KISS ESC TELEMETRY PROTOCOL
  ---------------------------
  One transmission will have 10 times 8-bit bytes sent with 115200 baud and 3.6V.
  Byte 0: Temperature
  Byte 1: Voltage high byte
  Byte 2: Voltage low byte
  Byte 3: Current high byte
  Byte 4: Current low byte
  Byte 5: Consumption high byte
  Byte 6: Consumption low byte
  Byte 7: Rpm high byte
  Byte 8: Rpm low byte
  Byte 9: 8-bit CRC
*/

#define TELEMETRY_FRAME_SIZE 10

typedef struct {
  uint32_t temperature;   // Temperature (resolution 1°C)
  uint32_t voltage;       // Voltage (resolution 0.01V)
  uint32_t current;       // Current (resolution 0.01A)
  uint32_t consumption;   // Consumption (resolution 1mAh)
  uint32_t erpm;          // Electrical Rpm (resolution 100Rpm)
} telemetryDataStructure;

extern telemetryDataStructure telemetryData;

void telemetry(void);
