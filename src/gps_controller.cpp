#include "gps_controller.h"

GpsController* g_interrupt_gps_controller = 0;

// function to run on interrupt from timer 0
ISR(TIMER0_COMPA_vect) {
  if (g_interrupt_gps_controller != 0) {
    g_interrupt_gps_controller->Read();
  }
}

GpsController* GpsController::Create(Controller* controller, Imu* imu) {
  if (g_interrupt_gps_controller == 0) {
    g_interrupt_gps_controller = new GpsController(controller, imu);
  }

  return g_interrupt_gps_controller;
}

GpsController::GpsController(Controller* controller, Imu* imu)
    : controller_(controller),
      imu_(imu) {
  gps_.begin(9600);
  gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  gps_.sendCommand(PGCMD_ANTENNA);


  // set compare match register for timer 0
  OCR0A = 0xAC; // this number doesn't actually matter, because timer 0 overflows every 1.024 ms
  // enable timer 0 compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

// MORE GPS FUNCTIONS WILL GO HERE //

/////////////////////////////////////

void GpsController::Read() {
  gps_.read();
}

void GpsController::Update() {
  // 0 <= heading <= 360
  float heading;
  imu_->GetHeading(heading);

  // GET GPS DATA
  if (gps_.newNMEAreceived() && gps_.parse(gps_.lastNMEA())) {
    if (gps_.fix) {
      Serial.print(F("FIX\t\t"));
      gps_location_.latitude = gps_.latitudeDegrees;
      gps_location_.longitude = gps_.longitudeDegrees;
      gps_location_.altitude = gps_.altitude;
    } else {
      Serial.print(F("NO FIX\t\t"));
    }
  } else {
    Serial.print(F("NO CONNECTION\t"));
  }
  
  // PRELIMINARY CALCULATIONS
  unsigned long update_time_ = micros();
  float dt = (update_time_ - last_update_time_) / 1000000.0; // in s
  last_update_time_ = update_time_;

  // in meters
  float horizontal_distance_to_target = hypot(
    (gps_location_.latitude - waypoints_[next_waypoint_].latitude)
      * kMetersPerDegreeLatitude,
    (gps_location_.longitude - waypoints_[next_waypoint_].longitude)
      * kMetersPerDegreeLongitude);

  // check if we are at the next waypoint
  if (horizontal_distance_to_target < kHorizontalErrorTolerance &&
      abs(gps_location_.altitude - waypoints_[next_waypoint_].altitude)
        < kVerticalErrorTolerance) {
    ++next_waypoint_;
  }

  ControllerCommands controller_commands_;

  ////////////////// HEADING ////////////////////

  // -180 <= target_heading <= 180
  float target_heading;

  if (waypoints_[next_waypoint_].longitude == gps_location_.longitude &&
      waypoints_[next_waypoint_].latitude == gps_location_.latitude) {
    // atan2 fails in this case
    // we are already over the waypoint anyway, so hold our heading
    target_heading = heading;
    if (target_heading > 180) {
      target_heading -= 360;
    }
  } else {
    // we aren't near the prime meridian, so we can assume both longitudes are
    // on the same side of the prime meridian
    target_heading = 180 / M_PI * atan2(
      (waypoints_[next_waypoint_].longitude - gps_location_.longitude)
        * kMetersPerDegreeLongitude,
      (waypoints_[next_waypoint_].latitude - gps_location_.latitude)
        * kMetersPerDegreeLatitude);
  }


  // the angle between the target heading and the current heading
  
  // -180 < error_heading < 180
  float error_heading = target_heading - heading;
  if (error_heading < -180) {
    error_heading += 360;
  }

  controller_commands_.heading = target_heading;

  ///////////////////////////////////////////////


  //////////////////// BANK /////////////////////
  
  // in meters
  float error_right = horizontal_distance_to_target * sin(error_heading);

  accumulated_error_right_ += error_right * dt;

  float correction_right = kTiltRightP * error_right +
                           kTiltRightI * accumulated_error_right_;

  controller_commands_.bank = correction_right < 0 ?
    max(correction_right, -kMaxBank) :
    min(correction_right, kMaxBank);

  ///////////////////////////////////////////////


  ////////////////// ATTITUDE ///////////////////

  ///////////////////////////////////////////////


  ////////////////// THROTTLE ///////////////////

  // are we at the right altitude?
  if (abs(gps_location_.altitude - waypoints_[next_waypoint_].altitude)
        < kVerticalErrorTolerance) {
    // yes, we are
  } else {
    // no, we aren't
  }

  ///////////////////////////////////////////////

  // are we at the right heading?
  if (min(abs(heading - target_heading),
          360 - abs(heading - target_heading)) < kHeadingErrorTolerance) {
    // yes, we are
    // set attitude and bank
  } else {
    // no, we aren't
    controller_commands_.attitude = 0;
    controller_commands_.bank = 0;
  }


  controller_->SetCommands(controller_commands_);
}
