#ifndef QUADCOPTER_IMU_H_
#define QUADCOPTER_IMU_H_

#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include <L3G.h>

#include "util.h"

class Imu {
 public:
  Imu();
  ~Imu();

  void Update(Orientation& out);
  void Update(ImuData& out);
 private:
  static const float kMagnetometerXOffset = 25; // in uT
  static const float kMagnetometerYOffset = 10; // in uT
  static const float kMagnetometerZOffset = -5; // in uT:
  static const float kMagneticDeclination = -0.20624; // in radians
  static const float kGyroscopeConversionFactor = 0.0001527; // in (rad/s)/LSB

  Adafruit_ADXL345_Unified accel_ = Adafruit_ADXL345_Unified(12345);
  sensors_event_t accel_event_;

  L3G gyro_ = L3G();
  
  Adafruit_HMC5883_Unified mag_ = Adafruit_HMC5883_Unified(12345);
  sensors_event_t mag_event_;

  Adafruit_BMP085_Unified baro_ = Adafruit_BMP085_Unified(12345);
  sensors_event_t baro_event_;
  float temperature_;
};

#endif
