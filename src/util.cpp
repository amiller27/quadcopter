#include "util.h"

float sin(float x) {
  // not yet implemented
  return 0;
}

Vector::Vector(float x, float y, float z) : x_(x), y_(y), z_(z) {}

float Vector::GetLength() {
  return sqrt((x_ * x_) + (y_ * y_) + (z_ * z_));
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

Quaternion& Quaternion::operator*=(const Quaternion& rhs) {
  a_ = (a_ * rhs.a_) - (b_ * rhs.b_) - (c_ * rhs.c_) - (d_ * rhs.d_);
  b_ = (a_ * rhs.b_) + (b_ * rhs.a_) + (c_ * rhs.d_) - (d_ * rhs.c_);
  c_ = (a_ * rhs.c_) - (b_ * rhs.d_) + (c_ * rhs.a_) + (d_ * rhs.b_);
  d_ = (a_ * rhs.d_) + (b_ * rhs.c_) - (c_ * rhs.b_) + (d_ * rhs.a_);
}
