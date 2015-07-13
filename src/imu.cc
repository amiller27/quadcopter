#include "imu.h"

Imu::Imu() {

  // set up accelerometer
  if (!accel_.begin()) {
    // problem with accelerometer connection
  }

  accel_.setRange(ADXL345_RANGE_16_G);

  // set up gyroscope
  Wire.begin();

  if (!gyro_.init()) {
    // problem with gyro connection
  }

  gyro_.enableDefault();

  if (!mag_.begin()) {
    // problem with magnetometer connection
  }

  if (!baro_.begin()) {
    // problem with barometer connection
  }
}

void Imu::Update(Orientation& out) {
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&accel_event_);

  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();

  //////////////////  CALCULATE ORIENTATION VECTOR HERE //////////////////

  ////////////////////////////////////////////////////////////////////////

  // has event.magnetic.(x|y|z) in uT
  mag_.getEvent(&mag_event_);

  //////////////////////// CALCULATE HEADING HERE ////////////////////////

  ////////////////////////////////////////////////////////////////////////

}

void Imu::Update(ImuData& out) {
  Update(out.orientation);

  // has event.pressure
  baro_.getEvent(&baro_event_);
  if (baro_event_.pressure) {
    out.pressure = baro_event_.pressure;
    baro_.getTemperature(out.temperature);

    out.altitude = baro_.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                              baro_event_.pressure);
  } else {
    // sensor error
  }
}
