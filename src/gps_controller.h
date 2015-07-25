#ifndef QUADCOPTER_GPS_CONTROLLER_H_
#define QUADCOPTER_GPS_CONTROLLER_H_

#include <Adafruit_GPS.h>
#include <CustomSoftwareSerial.h>

#include "controller.h"
#include "imu.h"

struct GpsLocation {
  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
};

class GpsController {
 public:
  static GpsController* Create(Controller* controller, Imu* imu);
  GpsController(Controller* controller, Imu* imu);
  ~GpsController();

  // MORE GPS FUNCTIONS WILL BE NEEDED HERE //
  void AddWaypoint(GpsLocation waypoint);
  void AddWaypoints(GpsLocation waypoints[], int number_of_waypoints);
  ////////////////////////////////////////////

  /**
   * Reads one character from the GPS.  Meant to be called in the timer interrupt.
   */
  void Read();
  void Update();

 private:
  // Pins
  static const int kGpsTxPin = 3; // Tx on GPS, Rx on Arduino
  static const int kGpsRxPin = 2; // Rx on GPS, Tx on Arduino

  // PID constants
  static const float kTiltP = 1;
  static const float kTiltI = 1;
  static const float kThrottleP = 1;
  static const float kThrottleI = 1;

  // Target Speeds
  static const float kTargetHorizontalSpeed = 20; // in knots
  static const float kTargetVerticalSpeed = 1; // in m/s

  // amount that the quad is allowed to be off from the waypoint and
  // still be considered to have hit it
  static const float kHorizontalErrorTolerance = 1; // in meters
  static const float kVerticalErrorTolerance = 3; // in meters

  // amount that the quad is allowed to be off from the heading it needs
  // to travel toward the next waypoint
  static const float kHeadingErrorTolerance = 5; // in degrees

  // calculated assuming 39.8 degrees north latitude
  static const float kMetersPerDegreeLatitude = 111030.76454339818;
  static const float kMetersPerDegreeLongitude = 85642.43721853253;

  Controller* controller_;
  ControllerCommands controller_commands_;
  Imu* imu_;

  GpsLocation waypoints_[];
  int number_of_waypoints_;
  int next_waypoint_ = 0;

  GpsLocation last_gps_location_;
  float last_heading_;

  SoftwareSerial serial_ = SoftwareSerial(kGpsTxPin, kGpsRxPin);
  Adafruit_GPS gps_ = Adafruit_GPS(&serial_);
};

#endif
