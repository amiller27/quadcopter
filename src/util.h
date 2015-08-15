#ifndef QUADCOPTER_UTIL_H_
#define QUADCOPTER_UTIL_H_

float sin(float);
float cos(float);
float tan(float);

float mapf(float, float, float, float, float);

struct Orientation {
  //Orientation is calculated as intrinsic Tait-Bryan angles about the..
  float heading;    // z  axis
  float attitude;   // y' axis
  float bank;       // x" axis
};

#endif
