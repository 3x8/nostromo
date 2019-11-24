#include "telemetry.h"

telemetryData_t telemetryData;
uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE];

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u = crc;

  crc_u ^= crc_seed;
  for (int i = 0; i < 8; i++) {
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

static void telemetryTelegram(telemetryData_t *data) {
  telemetryBuffer[0] = data->temperature;
  telemetryBuffer[1] = data->voltage >> 8;
  telemetryBuffer[2] = data->voltage & 0xFF;
  telemetryBuffer[3] = data->current >> 8;
  telemetryBuffer[4] = data->current & 0xFF;
  telemetryBuffer[5] = data->consumption >> 8;
  telemetryBuffer[6] = data->consumption & 0xFF;
  telemetryBuffer[7] = data->erpm >> 8;
  telemetryBuffer[8] = data->erpm & 0xFF;
  telemetryBuffer[9] = calculateCrc8(telemetryBuffer, 9);

  for(uint8_t i = 0; i < TELEMETRY_FRAME_SIZE; i++) {
    #if (!defined(DEBUG_DATA_UART))
      uartWrite(telemetryBuffer[i]);
    #endif
  }
}

void telemetry(void) {
  telemetryData.temperature = adcScaled.temperature;
  telemetryData.voltage = adcScaled.voltage;

  if (adcScaled.current < 0) {
    telemetryData.current = 0;
  } else {
    telemetryData.current = adcScaled.current;
  }

  if (consumptionMah < 0) {
    telemetryData.consumption = 0;
  } else {
    telemetryData.consumption =  (int)consumptionMah;
  }

  telemetryData.erpm = 542137.4/motor.CommutationInterval;

  telemetryTelegram(&telemetryData);
}
