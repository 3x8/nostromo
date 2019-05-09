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
uint32_t  SortList[WINDOW];
int pos;

uint32_t FilterCalculate(uint32_t newValue) {
  uint32_t retVal = 0;

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
    double temp = SortList[j];
    SortList[j] = SortList[min];
    SortList[min] = temp;
  }

  for (int i = 2; i < WINDOW - 2 ; i++) {
    retVal = retVal + SortList[i];
  }
  retVal = retVal /(WINDOW - 4);

  return retVal;
}
