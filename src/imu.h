#ifndef QUADCOPTER_IMU_H_
#define QUADCOPTER_IMU_H_

#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include <L3G.h>

#include "Arduino.h"

#include "util.h"

struct ImuData {
  Orientation orientation;
  float pressure;
  float temperature;
  float altitude;
};

class Imu {
 public:
  Imu();
  ~Imu();

  void GetHeading(float& out);
  void UpdateOrientation();
  void GetHeading(float& out);
  void GetOrientation(Orientation& out);
  void UpdateAll();
  void GetAllData(ImuData& out);

 private:
  unsigned long last_sensor_time;
  ImuData all_data_;

  //complimentary filter constant
  static const float acclelerometerWeight = 0.03; //out of 1

  // SENSORS

  sensors_event_t sensor_event_;

  //magnetometer 
  Adafruit_HMC5883_Unified mag_ = Adafruit_HMC5883_Unified(12345);
  static const float kMagnetometerXOffset = 25; // in uT
  static const float kMagnetometerYOffset = 10; // in uT
  static const float kMagnetometerZOffset = -5; // in uT
  static const float kMagneticDeclination = -0.20624; // in radians

  //gyroscope
  static const float kGyroscopeConversionFactor = 0.0087491; // in (deg/s)/LSB
  L3G gyro_ = L3G();

  //accelerometer
  Adafruit_ADXL345_Unified accel_ = Adafruit_ADXL345_Unified(12345);
  static const float kAccelXOffset = 0; //in deg
  static const float kAccelYOffset = 0; //in deg
  static const float kAccelZOffset = 0; //in deg
  
  //barometer
  Adafruit_BMP085_Unified baro_ = Adafruit_BMP085_Unified(12345);
};

#endif
