#include "util.h"

double sin(double x) {
  // not yet implemented
  return 0;
}

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
  a = (a * rhs.a) - (b * rhs.b) - (c * rhs.c) - (d * rhs.d);
  b = (a * rhs.b) + (b * rhs.a) + (c * rhs.d) - (d * rhs.c);
  c = (a * rhs.c) - (b * rhs.d) + (c * rhs.a) + (d * rhs.b);
  d = (a * rhs.d) + (b * rhs.c) - (c * rhs.b) + (d * rhs.a);
}

double Vector::GetLength() {
  return sqrt((x * x) + (y * y) + (z * z));
}