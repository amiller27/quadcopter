#ifndef QUADCOPTER_UTIL_H_
#define QUADCOPTER_UTIL_H_

#include "math.h"

float sin(float);
float cos(float);
float tan(float);

class Vector {
 public:
  float x_ = 0;
  float y_ = 0;
  float z_ = 0;

  Vector(float x, float y, float z);
  ~Vector();

  float GetLength();
};

class Quaternion {
 public:
  float a_ = 0;
  float b_ = 0;
  float c_ = 0;
  float d_ = 0;

  Quaternion(float a, float b, float c, float d);
  Quaternion(Vector& v, float theta);
  ~Quaternion();

  // operators
  Quaternion& operator*=(const Quaternion& rhs);
  const Quaternion operator*(const Quaternion& other) const {
    return Quaternion(*this) *= other;
  }
};

struct Orientation {
  //Orientation is calculated as intrinsic Tait-Bryan angles about the..
  float heading;    // z  axis
  float attitude;   // y' axis
  float bank;       // x" axis
};

#endif
