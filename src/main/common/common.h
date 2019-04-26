#pragma once

#include "main.h"

#define MIN(a,b) __extension__ ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define MAX(a,b) __extension__ ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define ABS(x) __extension__ ({ __typeof__ (x) _x = (x); _x > 0 ? _x : -_x; })

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
uint32_t constrain(uint32_t amt, uint32_t low, uint32_t high);
