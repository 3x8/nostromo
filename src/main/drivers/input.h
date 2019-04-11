#pragma once

#include "main.h"

void detectInput();
void computeProshotDMA();
void computeMSInput();
void computeOS125Input();
void computeOS42Input();
void computeServoInput();
void computeDshotDMA();
void transferComplete();
long map(long x, long in_min, long in_max, long out_min, long out_max);
