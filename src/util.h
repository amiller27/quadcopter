#ifndef QUADCOPTER_UTIL_H_
#define QUADCOPTER_UTIL_H_

float fast_sin(float);
float fast_cos(float);
float fast_tan(float);

float fast_acos(float); // only good near 1

float mapf(float, float, float, float, float);

struct Orientation {
  // Orientation is calculated as intrinsic Tait-Bryan angles about the..
  float heading;    // z  axis
  float attitude;   // y' axis
  float bank;       // x" axis
};

#endif
