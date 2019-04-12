#include "common.h"

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
