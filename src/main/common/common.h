#pragma once

#include <math.h>
#include "main.h"

#define MIN(a,b) __extension__ ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define MAX(a,b) __extension__ ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define ABS(x) __extension__ ({ __typeof__ (x) _x = (x); _x > 0 ? _x : -_x; })

uint32_t scaleInputToOutput(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
uint32_t constrain(uint32_t amt, uint32_t low, uint32_t high);

// kalman filter
#define FAST_CODE                   __attribute__((section(".tcm_code")))
#define MAX_WINDOW_SIZE 32

typedef struct kalman_s {
    uint32_t w;    // window size
    float q;       // process noise covariance
    float r;       // measurement noise covariance
    float p;       // estimation error covariance matrix
    float k;       // kalman gain
    float x;       // state
    float lastX;   // previous state

    float window[MAX_WINDOW_SIZE];
    float variance;
    float varianceSum;
    float mean;
    float meanSum;
    uint32_t windowIndex;
} kalman_t;

void kalmanInit(kalman_t *filter, float q, uint32_t w);
FAST_CODE float kalmanUpdate(kalman_t *filter, float input);
