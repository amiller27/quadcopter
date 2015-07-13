#ifndef QUADCOPTER_UTIL_H_
#define QUADCOPTER_UTIL_H_

float sin(float);

class Vector {
 public:
  float x_ = 0;
  float y_ = 0;
  float z_ = 0;

  Vector(float x, float y, float z);
  ~Vector();

  float GetLength();
};

struct GpsLocation {
  float latitude = 0;
  float longitude = 0;
};

struct Orientation {
  float pitch = 0;
  float roll = 0;
  float heading = 0;
};

struct ImuData {
  Orientation orientation;
  float pressure;
  float temperature;
  float altitude;
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

#endif
