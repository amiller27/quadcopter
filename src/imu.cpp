#include "imu.h"

#include <math.h>

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

  last_sensor_time = micros();
}

void Imu::GetOrientation(Orientation& out) {
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&accel_event_);
  
  float accel_bank = asin(pow(pow(accel_event_.acceleration.y, 2) +
                 pow(accel_event_.acceleration.z, 2), 0.5)) *
                         radianDegreeConversionFactor;

  float accel_attitude = asin(pow(pow(accel_event_.acceleration.x, 2) +
                 pow(accel_event_.acceleration.z, 2), 0.5)) *
                         radianDegreeConversionFactor;

  float accel_heading = asin(pow(pow(accel_event_.acceleration.x, 2) +
                 pow(accel_event_.acceleration.y, 2), 0.5)) *
                         radianDegreeConversionFactor;
  ///////////////////////////////////////////////////////////////////////////


  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  ///////////////////////////////////////////////////////////////////////////

  
  ////////////////////////////// MAGNOMETER /////////////////////////////////
  // has event.magnetic.(x|y|z) in uT
  mag_.getEvent(&mag_event_);
  ///////////////////////////////////////////////////////////////////////////
  

  Orientation p = orientation_;

  unsigned long current = micros();
  float dt = current - last_sensor_time;
  last_sensor_time = current;

  dt /= 1000000.0; //convert dt to seconds for sensor compatibility
  Serial.print("last sensor time: ");
  Serial.println(last_sensor_time);
  Serial.print("dt: ");
  Serial.println(dt);


  /////////////////////////////// BANK ///////////////////////////////////////
  orientation_.bank = (1-acclelerometerWeight) * (p.bank + 
kGyroscopeConversionFactor * gyro_.g.x * dt) + acclelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  orientation_.attitude = (1-acclelerometerWeight) * 
  (p.attitude + kGyroscopeConversionFactor * dt * 
  (gyro_.g.y * cos(p.bank / radianDegreeConversionFactor) + gyro_.g.z * 
  sin(p.bank / radianDegreeConversionFactor))) + acclelerometerWeight * accel_heading;
  ////////////////////////////////////////////////////////////////////////////
 

  /////////////////////////////// HEADING ////////////////////////////////////
  orientation_.heading = atan2(mag_event_.magnetic.y * cos(p.bank / radianDegreeConversionFactor), 
                               mag_event_.magnetic.x * cos(p.attitude / radianDegreeConversionFactor)) *
                               radianDegreeConversionFactor;
  ////////////////////////////////////////////////////////////////////////////


  out.bank = orientation_.bank;
  out.attitude = orientation_.attitude;
  out.heading = orientation_.heading;
}

void Imu::GetAllData(ImuData& out) {
  GetOrientation(out.orientation);

  baro_.getEvent(&baro_event_);
  if (baro_event_.pressure) {
    out.pressure = baro_event_.pressure;
    baro_.getTemperature(&(out.temperature));

    out.altitude = baro_.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                            baro_event_.pressure);
  } else {
    // barrometer error
    Serial.println("barrometer error");
  }
}
