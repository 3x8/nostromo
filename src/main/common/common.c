#include "common.h"

uint32_t map(uint32_t input, uint32_t inputMin, uint32_t inputMax, uint32_t outputMin, uint32_t outputMax) {
  if (input < inputMin) {
    input = inputMin;
  }
  if (input > inputMax) {
    input = inputMax;
  }
  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}

uint32_t constrain(uint32_t input, uint32_t valueMin, uint32_t valueMax) {
  if (input < valueMin){
    return (valueMin);
  }
  else if (input > valueMax) {
      return (valueMax);
  }
  else {
      return (input);
  }
}
