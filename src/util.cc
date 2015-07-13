#include "util.h"

float sin(float x) {
  // not yet implemented
  return 0;
}

Vector::Vector(float x, float y, float z) : x_(x), y_(y), z_(z) {}

double Vector::GetLength() {
  return sqrt((x * x) + (y * y) + (z * z));
}

Quaternion::Quaternion(float a, float b, float c, float d)
    : a_(a),
      b_(b),
      c_(c),
      d_(d) {
}

Quaternion::Quaternion(Vector& v, float theta) {
  a_ = cos(theta / 2);
  float sin_half_theta = sin(theta / 2);
  b_ = v.x_ * sin_half_theta;
  c_ = v.y_ * sin_half_theta;
  d_ = v.z_ * sin_half_theta;
}

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
  a = (a * rhs.a) - (b * rhs.b) - (c * rhs.c) - (d * rhs.d);
  b = (a * rhs.b) + (b * rhs.a) + (c * rhs.d) - (d * rhs.c);
  c = (a * rhs.c) - (b * rhs.d) + (c * rhs.a) + (d * rhs.b);
  d = (a * rhs.d) + (b * rhs.c) - (c * rhs.b) + (d * rhs.a);
}
