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
  Imu(bool &successful);
  ~Imu();

  void CalibrateMagnetometer(int cycles);

  void UpdateAll(bool update_heading);
  void UpdateOrientation(bool update_heading);

  void GetAllData(ImuData& out);
  void GetAltitude(float& out);
  void GetHeading(float& out);
  void GetOrientation(Orientation& out);

 private:
  unsigned long last_sensor_time_;
  ImuData all_data_;

  //complimentary filter constant
  static const float kAccelerometerWeight = 0.01; //out of 1

  // SENSORS

  sensors_event_t sensor_event_;

  //magnetometer 
  Adafruit_HMC5883_Unified mag_ = Adafruit_HMC5883_Unified(12345);
  static const float kMagnetometerXOffset = 25; // in uT
  static const float kMagnetometerYOffset = 10; // in uT
  static const float kMagnetometerZOffset = -5; // in uT
  static const float kMagFilterConst = 0.2;
  sensors_vec_t last_mag_;

  float heading_offset;

  //gyroscope
  static const float kGyroscopeConversionFactor = 0.07; // in (deg/s)/LSB
  static const float kGyroscopeXOffset = 85;
  static const float kGyroscopeYOffset = 15;
  static const float kGyroscopeZOffset = 127;
  static const float kGyroscopeFilterConst = 0.2;
  L3G gyro_ = L3G();
  L3G::vector<int16_t> last_gyro_;

  //accelerometer
  Adafruit_ADXL345_Unified accel_ = Adafruit_ADXL345_Unified(12345);
  static const float kAccelXOffset = 0;
  static const float kAccelYOffset = 0;
  static const float kAccelZOffset = 0;
  static const float kAccelBankOffset = 3.0;
  
  //barometer
  Adafruit_BMP085_Unified baro_ = Adafruit_BMP085_Unified(12345);
  unsigned long last_baro_time_ = 0;
  //static const int kBaroDataLength = 20;
  //float baro_data_[kBaroDataLength] = {};
  //int baro_data_position_ = 0;
};

#endif
