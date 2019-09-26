/*
  Copyright (C) 2011 - Curtis L. Olson curtolson@flightgear.org
*/

/*
  Task: configure autopilot settings to fly to a circle around a specified
  point.  Compensate circle track using wind estimate to try to achieve a
  better circle form.
*/

#include <cstdio>
#include <cmath>

#include "circle_mgr.h"

static const double r2d = 180.0 / M_PI;
static const double d2r = M_PI / 180.0;

static ElementPtr vn_node;
static ElementPtr ve_node;
static ElementPtr track_node;
static ElementPtr lat_rad_node;
static ElementPtr lon_rad_node;
static ElementPtr gps_fix_node;
static ElementPtr course_error_node;

CircleMgr::CircleMgr() {
};


CircleMgr::~CircleMgr() {
};


// FIXME: make sure we have sane default values setup some where
// Maybe: setup home as the default circle location if not otherwise set?
// How does this play with the mission system?  We don't want to do
// nothing and have the aircraft just fly off in a straight line forever.
void CircleMgr::init( const rapidjson::Value& Config ) {
  // configuration
  if ( Config.HasMember("Direction") ) {
    set_direction( Config["Direction"].GetString() );
  } else {
    set_direction( "left" );
  }
  if ( Config.HasMember("Radius") ) {
    set_radius( Config["Radius"].GetFloat() );
  } else {
    set_radius( 100.0 );
  }

  // input signals
  vn_node = deftree.getElement("/Sensor-Processing/NorthVelocity_ms", true);
  ve_node = deftree.getElement("/Sensor-Processing/EastVelocity_ms", true);
  track_node = deftree.getElement("/Sensor-Processing/Track_rad", true);
  lat_rad_node = deftree.getElement("/Sensor-Processing/Latitude_rad", true);
  lon_rad_node = deftree.getElement("/Sensor-Processing/Longitude_rad", true);
  gps_fix_node = deftree.getElement("/Sensors/uBlox/Fix");

  // output signals
  course_error_node = deftree.initElement("/Route/nav_course_error_rad", "Route manager course error", LOG_FLOAT, LOG_NONE);

  initialized = true;
}


void CircleMgr::update() {
  if ( !initialized ) {
    // bail out if we were never initialized (i.e. no Circle section
    // in the config file.)
    return;
  }

  /* float gs_mps = sqrt(vn_node->getFloat() * vn_node->getFloat()
                         + ve_node->getFloat() * ve_node->getFloat()); */
  float track_deg = track_node->getFloat() * r2d;
  double lat_deg = lat_rad_node->getDouble() * r2d;
  double lon_deg = lon_rad_node->getDouble() * r2d;

  if ( !pos_set && gps_fix_node->getInt() == 1 ) {
    printf("Setting circle center...\n");
    set_center(lat_deg, lon_deg);
    pos_set = true;
  }

  // compute course and distance to center of target circle
  float course_deg;
  float dist_m;
  target.CourseAndDistance( lon_deg, lat_deg, &course_deg, &dist_m );

  // compute ideal ground course to be on the circle perimeter if at
  // ideal radius
  double ideal_crs = course_deg + direction * 90;
  if ( ideal_crs > 360.0 ) { ideal_crs -= 360.0; }
  if ( ideal_crs < 0.0 ) { ideal_crs += 360.0; }

  // (in)sanity check
  if ( radius_m < 25.0 ) { radius_m = 25.0; }
  if ( radius_m > 600.0 ) { radius_m = 600.0; }

  // compute a target ground course based on our actual radius distance
  double target_crs = ideal_crs;
  if ( dist_m < radius_m ) {
    // inside circle, adjust target heading to expand our circling
    // radius
    double offset_deg = direction * 90.0 * (1.0 - dist_m / radius_m);
    target_crs += offset_deg;
    if ( target_crs > 360.0 ) { target_crs -= 360.0; }
    if ( target_crs < 0.0 ) { target_crs += 360.0; }
  } else if ( dist_m > radius_m ) {
    // outside circle, adjust target heading to tighten our
    // circling radius
    double offset_dist = dist_m - radius_m;
    if ( offset_dist > radius_m ) { offset_dist = radius_m; }
    double offset_deg = direction * 90 * offset_dist / radius_m;
    target_crs -= offset_deg;
    if ( target_crs > 360.0 ) { target_crs -= 360.0; }
    if ( target_crs < 0.0 ) { target_crs += 360.0; }
  }

  // compute course error
  float course_error = target_crs - track_deg;
  if ( course_error < -180.0 ) {
    course_error += 360.0;
  } else if ( course_error > 180.0 ) {
    course_error -= 360.0;
  }
  course_error_node->setFloat(course_error * d2r);

  // // L1 'mathematical' response to error

  // double L1_period = L1_node.getDouble("period");	// gain
  // const double sqrt_of_2 = 1.41421356237309504880;
  // double omegaA = sqrt_of_2 * M_PI / L1_period;
  // double VomegaA = gs_mps * omegaA;
  // double course_error = orient_node.getDouble("groundtrack_deg") - target_crs;
  // if ( course_error < -180.0 ) { course_error += 360.0; }
  // if ( course_error >  180.0 ) { course_error -= 360.0; }
  // // accel: is the lateral acceleration we need to compensate for
  // // heading error
  // double accel = 2.0 * sin(course_error * SG_DEGREES_TO_RADIANS) * VomegaA;

  // // ideal_accel: the steady state lateral accel we would expect
  // // when we are in the groove exactly on our target radius
  // // double ideal_accel = direction * gs_mps * gs_mps / radius_m;

  // // circling acceleration needed for our current distance from center
  // double turn_accel = direction * gs_mps * gs_mps / dist_m;

  // // compute desired acceleration = acceleration required for course
  // // correction + acceleration required to maintain turn at current
  // // distance from center.
  // double total_accel = accel + turn_accel;

  // static const double gravity = 9.81; // m/sec^2
  // double target_bank = -atan( total_accel / gravity );
  // double target_bank_deg = target_bank * SG_RADIANS_TO_DEGREES;

  // double bank_limit_deg = L1_node.getDouble("bank_limit_deg");
  // if ( target_bank_deg < -bank_limit_deg ) {
  //   target_bank_deg = -bank_limit_deg;
  // }
  // if ( target_bank_deg > bank_limit_deg ) {
  //   target_bank_deg = bank_limit_deg;
  // }
  // // printf("   circle: tgt bank = %.0f  bank limit = %.0f\n",
  // //	   target_bank_deg, bank_limit_deg);

  // targets_node.setDouble( "roll_deg", target_bank_deg );

  // // printf("circle: ideal ground crs = %.1f aircraft ground crs = %.1f\n",
  // //	   course_deg, orient_node.getDouble("groundtrack_deg") );
};


void CircleMgr::set_center( const double lat_deg, const double lon_deg ) {
  target = SGWayPoint( lon_deg, lat_deg );
}


void CircleMgr::set_direction( const string value ) {
  if ( value == "left" ) {
    direction = 1.0;
  } else if ( value == "right" ) {
    direction = -1.0;
  } else {
    printf("invalid direction string!\n");
  }
}

void CircleMgr::set_radius( const float value ) {
  radius_m = value;
}
