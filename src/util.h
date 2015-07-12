#ifndef QUADCOPTER_UTIL_H_
#define QUADCOPTER_UTIL_H_

double sin(double);

class Quaternion
{
public:
  double a = 0;
  double b = 0;
  double c = 0;
  double d = 0;

  Quaternion(double a, double b, double c, double d);
  ~Quaternion();

  // operators
  Quaternion& operator+=(const Quaternion& rhs);
  const Quaternion operator+(const Quaternion& other) const {
    return Quaternion(*this) += other;
  }
};

class Vector
{
public:
  double x = 0;
  double y = 0;
  double z = 0;

  Vector(double x, double y, double z);
  ~Vector();

  double GetLength();
};

struct GpsLocation
{
  double latitude = 0;
  double longitude = 0;
};

#endif
