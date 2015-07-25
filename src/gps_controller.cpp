#include "gps_controller.h"

GpsController* g_interrupt_gps_controller = 0;

// function to run on interrupt from timer 0
ISR(TIMER0_COMPA_vect) {
  g_interrupt_gps_controller->Read();
}

GpsController* GpsController::Create(Controller* controller, Imu* imu) {
  GpsController* result = new GpsController(controller, imu);

  if (!g_interrupt_gps_controller) {
    g_interrupt_gps_controller = result;
  }

  return result;
}

GpsController::GpsController(Controller* controller, Imu* imu)
    : controller_(controller),
      imu_(imu) {
  gps_.begin(9600);
  gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  gps_.sendCommand(PGCMD_ANTENNA);

  // set compare match register for timer 0
  OCR0A = 0xAF;
  // enable timer 0 compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

// // MORE GPS FUNCTIONS WILL GO HERE //

// /////////////////////////////////////

void GpsController::Read() {
  gps_.read();
}

void GpsController::Update() {
  imu_->GetHeading(last_heading_);

  if (gps_.newNMEAreceived() && gps_.parse(gps_.lastNMEA())) {
    if (gps_.fix) {
      last_gps_location_.latitude = gps_.latitudeDegrees;
      last_gps_location_.longitude = gps_.longitudeDegrees;
      last_gps_location_.altitude = gps_.altitude;
    }
  } else {
    // a new sentence is not available, or the parsing failed
  }

  // check if we are at the next waypoint
  if (abs(last_gps_location_.latitude - waypoints_[next_waypoint_].latitude)
        < kHorizontalErrorTolerance &&
      abs(last_gps_location_.longitude - waypoints_[next_waypoint_].longitude)
        < kHorizontalErrorTolerance &&
      abs(last_gps_location_.altitude - waypoints_[next_waypoint_].altitude)
        < kVerticalErrorTolerance) {
    ++next_waypoint_;
  }

  // calculate needed heading
  // we aren't near the prime meridian, so we can assume both longitudes are
  // on the same side of the prime meridian
  float target_heading = 180 / M_PI * atan2(
    (waypoints_[next_waypoint_].longitude - last_gps_location_.longitude)
      * kMetersPerDegreeLongitude,
    (waypoints_[next_waypoint_].latitude - last_gps_location_.latitude)
      * kMetersPerDegreeLatitude);

  // are we at the right heading?
  if (min(abs(last_heading_ - target_heading),
          360 - abs(last_heading_ - target_heading)) < kHeadingErrorTolerance) {
    // yes, we are
  } else {
    // no, we aren't
    controller_commands_.heading = target_heading;
    controller_commands_.attitude = 0;
    controller_commands_.bank = 0;
  }

  // are we at the right altitude?
  if (abs(last_gps_location_.altitude - waypoints_[next_waypoint_].altitude)
        < kVerticalErrorTolerance) {
    // yes, we are
  } else {
    // no, we aren't
  }

  controller_->Update(controller_commands_);
}
