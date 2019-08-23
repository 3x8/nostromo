#include "common.h"

const char *byteToString(uint8_t x) {
    static char b[9];
    b[0] = '\0';

    for (int z = 128; z > 0; z >>= 1){
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return (b);
}

uint32_t scaleInputToOutput(uint32_t input, uint32_t inputMin, uint32_t inputMax, uint32_t outputMin, uint32_t outputMax) {
  if (input < inputMin) {
    input = inputMin;
  }
  if (input > inputMax) {
    input = inputMax;
  }
  return ((input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin);
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
    filter->q     = q * 0.000001f;
    filter->w     = w;
}

#pragma GCC push_options
#pragma GCC optimize("O3")

#define VARIANCE_SCALE 1.0f
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
    filter->r = sqrtf(filter->variance) * VARIANCE_SCALE;

    return (filter->x);
}

#pragma GCC pop_options
