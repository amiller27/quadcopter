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

  double GetHeading();
  void Update();
  Vector GetOrientation();
 private:
  static const double kMagnetometerXOffset = 25; // in uT
  static const double kMagnetometerYOffset = 10; // in uT
  static const double kMagnetometerZOffset = -5; // in uT:
  static const double kMagneticDeclination = -0.20624; // in radians
  static const double kGyroscopeConversionFactor = 8.75; // in mdps/LSB

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
