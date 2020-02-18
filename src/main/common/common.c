#include "common.h"

const char *byteToString(uint8_t x) {
  static char b[9];
  b[0] = '\0';

  for (int z = 128; z > 0; z >>= 1){
    strcat(b, ((x & z) == z) ? "1" : "0");
  }
  return (b);
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

// kalman filter
void kalmanInit(kalman_t *filter, float q, uint32_t w) {
  memset(filter, 0, sizeof(kalman_t));
  filter->q = q * 0.000001f;
  filter->w = w;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
FAST_CODE float kalmanUpdate(kalman_t *filter, float input) {
  const float windowSizeInverse = 1.0f/(filter->w - 1);

  // project the state ahead using acceleration
  filter->x += (filter->x - filter->lastX);

  // update last state
  filter->lastX = filter->x;

  // prediction update
  filter->p = filter->p + filter->q;

  // measurement update
  filter->k = filter->p / (filter->p + filter->r);
  filter->x += filter->k * (input - filter->x);
  filter->p = (1.0f - filter->k) * filter->p;

  // update variance
  filter->window[filter->windowIndex] = input;

  filter->meanSum += filter->window[filter->windowIndex];
  filter->varianceSum = filter->varianceSum + (filter->window[filter->windowIndex] * filter->window[filter->windowIndex]);

  if (++filter->windowIndex >= filter->w) {
    filter->windowIndex = 0;
  }

  filter->meanSum -= filter->window[filter->windowIndex];
  filter->varianceSum = filter->varianceSum - (filter->window[filter->windowIndex] * filter->window[filter->windowIndex]);

  filter->mean = filter->meanSum * windowSizeInverse;
  filter->variance = ABS(filter->varianceSum * windowSizeInverse - (filter->mean * filter->mean));
  filter->r = sqrtf(filter->variance);

  return (filter->x);
}
#pragma GCC pop_options


// median filter
#pragma GCC push_options
#pragma GCC optimize("O3")
void medianInit(median_t *filter, uint32_t w) {
  memset(filter, 0, sizeof(median_t));
  filter->w = w;
}

void medianPush(median_t *filter, uint32_t newValue) {
  filter->window[filter->windowIndex] = newValue;

  if (++filter->windowIndex >= filter->w) {
    filter->windowIndex = 0;
  }
}

uint32_t medianCalculate(median_t *filter) {
  uint32_t medianSumm;
  uint8_t i;

  for (i = 0; i < filter->w; i++) {
    medianSumm += filter->window[i];
  }

  return(medianSumm / filter->w);
}
#pragma GCC pop_options
