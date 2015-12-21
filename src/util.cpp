#include "util.h"

#include <math.h>

float sin(float x) {
  // two term power series approximation
  return x - (x*x*x)/6;
}

float cos(float x) {
  // two term power series approximation 
  return 1 - (x*x)/2;
}

float tan(float x) {
  // two term power series approximation 
  return x + (x*x*x)/3;
}

float acos(float x) {
  // two term power series approximation
  float sqrt_2y = sqrt(2 * (1 - x));
  return sqrt_2y + (sqrt_2y * sqrt_2y * sqrt_2y) / 24;
}

float mapf(float in, float inMin, float inMax, float outMin, float outMax) {
  return outMin + (outMax - outMin) / (inMax - inMin) * (in - inMin);
}