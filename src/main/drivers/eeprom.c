#include "eeprom.h"

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length) {
  uint8_t checksum = 0;
  const uint8_t *byteOffset;

  for (byteOffset = data; byteOffset < (data + length); byteOffset++) {
    checksum ^= *byteOffset;
  }
  return (checksum);
}

bool eepromValid(void) {
  const eepromStructure *temp = (const eepromStructure *) FLASH_EEPROM_ADDRESS;
  uint8_t checksum = 0;

  if (CONFIG_EEPROM_VERSION != temp->version) {
    return (false);
  }

  if ((temp->size != sizeof(eepromStructure)) || (temp->magic_be != 0xbe) || (temp->magic_ef != 0xef)) {
    return (false);
  }

  checksum = calculateChecksum((const uint8_t *) temp, sizeof(eepromStructure));
  if (checksum != 0) {
    return (false);
  }
  return (true);
}

void eepromRead(void) {
  if (!eepromValid()) {
    // reset esc, iwdg timeout
    while (true);
  }
  memcpy(&masterConfig, (char *) FLASH_EEPROM_ADDRESS, sizeof(eepromStructure));
}

void eepromWrite(void) {
  HAL_StatusTypeDef status;
  int8_t attemptsRemaining = 3;

  masterConfig.version = CONFIG_EEPROM_VERSION;
  masterConfig.size = sizeof(eepromStructure);
  masterConfig.magic_be = 0xbe;
  masterConfig.magic_ef = 0xef;
  masterConfig.chk = 0;
  masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(eepromStructure));

  HAL_FLASH_Unlock();
  while (attemptsRemaining--) {
    for (uint32_t wordOffset = 0; wordOffset < sizeof(eepromStructure); wordOffset += 4) {
      if (wordOffset % FLASH_PAGE_SIZE == 0) {
        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
        eraseInit.PageAddress = FLASH_EEPROM_ADDRESS + wordOffset;
        eraseInit.NbPages = 1;
        uint32_t eraseError = 0;

        status = HAL_FLASHEx_Erase(&eraseInit, &eraseError);
        if (status != HAL_OK) {
          break;
        }
      }

      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_EEPROM_ADDRESS + wordOffset, *(uint32_t *) ((char *) &masterConfig + wordOffset));
      if (status != HAL_OK) {
        break;
      }
    }

    if (status == HAL_OK) {
      break;
    }
  }

  HAL_FLASH_Lock();
  if ((status != HAL_OK) || !eepromValid()) {
  }
}
