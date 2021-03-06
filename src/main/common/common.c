#include "common.h"

#pragma GCC push_options
#pragma GCC optimize("O3")

const char *byteToString(uint8_t x) {
  static char b[9];
  b[0] = '\0';

  for (int z = 128; z > 0; z >>= 1){
    strcat(b, ((x & z) == z) ? "1" : "0");
  }
  return (b);
}

INLINE_CODE uint32_t constrain(uint32_t input, uint32_t valueMin, uint32_t valueMax) {
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

INLINE_CODE int32_t scaleLinear(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  if (x < in_min) {
    x = in_min;
  }

  if (x > in_max){
    x = in_max;
  }
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);
}

// kalman filter
void kalmanInit(kalmanStructure *filter, float q, uint32_t w) {
  memset(filter, 0, sizeof(kalmanStructure));
  filter->q = q * 0.000001f;
  filter->w = w + 1;
  if (filter->w > MAX_WINDOW_SIZE) {
    filter->w = MAX_WINDOW_SIZE;
  }
}

INLINE_CODE float kalmanUpdate(kalmanStructure *filter, float input) {
  const float windowSizeInverse = 1.0f / (filter->w - 1);

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

// median filter
void medianInit(medianStructure *filter, uint32_t w) {
  memset(filter, 0, sizeof(medianStructure));
  filter->windowSize = w + 1;
  if (filter->windowSize > MAX_WINDOW_SIZE) {
    filter->windowSize = MAX_WINDOW_SIZE;
  }
}

INLINE_CODE void medianPush(medianStructure *filter, uint32_t newValue) {
  filter->window[filter->windowIndex] = newValue;
  filter->meanSum += filter->window[filter->windowIndex];
  if (++filter->windowIndex >= filter->windowSize) {
    filter->windowIndex = 0;
  }
  filter->meanSum -= filter->window[filter->windowIndex];
}

INLINE_CODE uint32_t medianCalculate(medianStructure *filter) {
  return (filter->meanSum / (filter->windowSize - 1));
}

INLINE_CODE uint32_t medianSumm(medianStructure *filter) {
  return (filter->meanSum);
}
#pragma GCC pop_options
