#include "util.h"

#include <math.h>

float fast_sin(float x) {
  // two term power series approximation
  return x - (x*x*x)/6;
}

float fast_cos(float x) {
  // two term power series approximation 
  return 1 - (x*x)/2;
}

float fast_tan(float x) {
  // two term power series approximation 
  return x + (x*x*x)/3;
}

float fast_acos(float x) {
  //Serial.print("\tx: ");
  //Serial.print(x);
  if (x >= 1) {
    return 0;
  }
  // two term power series approximation
  float sqrt_2y = sqrt(2 * (1 - x));
  //Serial.print("\tsqrt:");
  //Serial.print(sqrt_2y);
  return sqrt_2y + (sqrt_2y * sqrt_2y * sqrt_2y) / 24;
}

float mapf(float in, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (outMax - outMin) / (inMax - inMin) * (in - inMin);
}
