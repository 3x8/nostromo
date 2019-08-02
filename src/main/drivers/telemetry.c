#include "telemetry.h"

telemetryData_t telemetryData;
uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE] = { 0, };


static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u = crc;

    crc_u ^= crc_seed;
    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }
    return (crc_u);
}

static uint8_t calculateCrc8(const uint8_t *buf, const uint8_t bufLen) {
    uint8_t crc = 0;

    for (int i = 0; i < bufLen; i++) {
        crc = updateCrc8(buf[i], crc);
    }
    return (crc);
}

static void telemetryTelegram(telemetryData_t *telemetryData) {
  telemetryBuffer[0] = telemetryData->temperature;
  telemetryBuffer[1] = telemetryData->voltage >> 8;
  telemetryBuffer[2] = telemetryData->voltage & 0xFF;
  telemetryBuffer[3] = telemetryData->current >> 8;
  telemetryBuffer[4] = telemetryData->current & 0xFF;
  telemetryBuffer[5] = telemetryData->consumption >> 8;
  telemetryBuffer[6] = telemetryData->consumption & 0xFF;
  telemetryBuffer[7] = telemetryData->rpm >> 8;
  telemetryBuffer[8] = telemetryData->rpm & 0xFF;
  telemetryBuffer[9] = calculateCrc8(telemetryBuffer, 9);

  for(uint8_t i = 0; i < TELEMETRY_FRAME_SIZE; i++) {
    uartWrite(telemetryBuffer[i]);
  }
}

void telemetry(void) {
  telemetryData.temperature = adcFiltered.current;
  telemetryData.voltage = adcFiltered.current;
  telemetryData.current = adcFiltered.current;
  telemetryData.consumption = 0;
  telemetryData.rpm = 0;

  telemetryTelegram(&telemetryData);
}
