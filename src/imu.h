#ifndef QUADCOPTER_IMU_H_
#define QUADCOPTER_IMU_H_

class Imu {
 public:
  Imu();
  ~Imu();
 private:
  const double kMagnetometerXOffset = 25;
  const double kMagnetometerYOffset = 10;
  const double kMagnetometerZOffset = -5;
};

#endif