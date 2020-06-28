#include "telemetry.h"

telemetryDataStructure telemetryData;
uint8_t telemetryBuffer[TELEMETRY_FRAME_SIZE];

static uint8_t crc8helper(uint8_t crc, uint8_t crcSeed) {
  uint8_t crcBuffer = crc;

  crcBuffer ^= crcSeed;
  for (int i = 0; i < 8; i++) {
    crcBuffer = (crcBuffer & 0x80) ? (0x7 ^ (crcBuffer << 1)) : (crcBuffer << 1);
  }
  return (crcBuffer);
}

static uint8_t crc8calculate(const uint8_t *buf, const uint8_t bufLen) {
  uint8_t crcBuffer = 0;

  for (int i = 0; i < bufLen; i++) {
    crcBuffer = crc8helper(buf[i], crcBuffer);
  }
  return (crcBuffer);
}

static void telemetryTelegram(telemetryDataStructure *data) {
  telemetryBuffer[0] = data->temperature;
  telemetryBuffer[1] = data->voltage >> 8;
  telemetryBuffer[2] = data->voltage & 0xFF;
  telemetryBuffer[3] = data->current >> 8;
  telemetryBuffer[4] = data->current & 0xFF;
  telemetryBuffer[5] = data->consumption >> 8;
  telemetryBuffer[6] = data->consumption & 0xFF;
  telemetryBuffer[7] = data->erpm >> 8;
  telemetryBuffer[8] = data->erpm & 0xFF;
  telemetryBuffer[9] = crc8calculate(telemetryBuffer, 9);

  for(uint8_t i = 0; i < TELEMETRY_FRAME_SIZE; i++) {
    #if (!defined(DEBUG_DATA_UART))
      uartWrite(telemetryBuffer[i]);
    #endif
  }
}

void telemetry(void) {
  #if (!defined(DEBUG_DATA_QUALITY))
    telemetryData.temperature = adcScaled.temperature;
  #else
    telemetryData.temperature = input.DataErrorCounter;
  #endif

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

  telemetryData.erpm = (RPM_CONSTANT / MOTOR_POLES) / motor.CommutationInterval;

  telemetryTelegram(&telemetryData);
}
