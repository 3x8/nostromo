#include "common.h"

uint32_t map(uint32_t input, uint32_t inputMin, uint32_t inputMax, uint32_t outputMin, uint32_t outputMax) {
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


#define WINDOW 5
uint32_t  Values[WINDOW];
float  SortList[WINDOW];
int pos;

uint32_t Filter(uint32_t newValue) {
  float retVal = 0;

  Values[pos++] = newValue;
  if (pos > WINDOW) pos = 0;

  for (int i = 0; i < WINDOW; i++) {
    SortList[i] = Values[i];
  }

  for (int j = 0; j < WINDOW; ++j) {
    //   Find position of minimum element
    int min = j;
    for (int k = j + 1; k < WINDOW; ++k)
      if (SortList[k] < SortList[min])
        min = k;
    //   Put found minimum element in its place
    float temp = SortList[j];
    SortList[j] = SortList[min];
    SortList[min] = temp;
  }

  for (int i = 1; i < WINDOW - 1 ; i++) {
    retVal = retVal + SortList[i];
  }

  return ((uint32_t)(retVal /(WINDOW - 2)));
}
