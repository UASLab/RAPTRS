// route_mgr.cc - manage a route

#include <math.h>
#include <stdlib.h>

#include "waypoint.hxx"
#include "route_mgr.hxx"


static const double r2d = 180.0 / M_PI;
static const double d2r = M_PI / 180.0;

static ElementPtr vn_node;
static ElementPtr ve_node;
static ElementPtr track_node;
static ElementPtr lat_rad_node;
static ElementPtr lon_rad_node;
static ElementPtr gps_fix_node;
static ElementPtr course_error_node;
static ElementPtr nav_course_error_node;
static ElementPtr xtrack_node;
static ElementPtr nav_dist_node;

FGRouteMgr::FGRouteMgr() :
  active( new SGRoute ),
  standby( new SGRoute ),
  last_lon( 0.0 ),
  last_lat( 0.0 ),
  last_az( 0.0 ),
  pos_set( false ),
  start_mode( FIRST_WPT ),
  completion_mode( LOOP ),
  xtrack_gain( 0.0 ),
  dist_remaining_m( 0.0 )
{
}


FGRouteMgr::~FGRouteMgr() {
  delete standby;
  delete active;
}


void FGRouteMgr::init( const rapidjson::Value& Config ) {
  printf("Initializing Route Manager...\n");

  // configuration
  if ( Config.HasMember("XtrackGain") ) {
    xtrack_gain = Config["XtrackGain"].GetFloat();
  } else {
    xtrack_gain = 1.0;
  }
  
  // input signals
  vn_node = deftree.getElement("/Sensor-Processing/NorthVelocity_ms", true);
  ve_node = deftree.getElement("/Sensor-Processing/EastVelocity_ms", true);
  track_node = deftree.getElement("/Sensor-Processing/Track_rad", true);
  lat_rad_node = deftree.getElement("/Sensor-Processing/Latitude_rad", true);
  lon_rad_node = deftree.getElement("/Sensor-Processing/Longitude_rad", true);
  gps_fix_node = deftree.getElement("/Sensors/uBlox/Fix");

  // output signals
  course_error_node = deftree.initElement("/Route/course_error_rad", "Route manager course error", LOG_FLOAT, LOG_NONE);
  nav_course_error_node = deftree.initElement("/Route/nav_course_error_rad", "Route manager course (corrected for xtrack) error", LOG_FLOAT, LOG_NONE);
  xtrack_node = deftree.initElement("/Route/xtrack_m", "Route manager cross track error", LOG_FLOAT, LOG_NONE);
  nav_dist_node = deftree.initElement("/Route/dist_m", "Route manager distance remaining on leg", LOG_FLOAT, LOG_NONE);
  
  active->clear();
  standby->clear();

  if ( ! build(Config) ) {
    printf("Detected an internal inconsistency in the route\n");
    printf(" configuration.  See earlier errors for details.\n" );
    exit(-1);
  }

  // build() constructs the new route in the "standby" slot,
  // swap it to "active"
  swap();

  initialized = true;
}


void FGRouteMgr::update() {
  if ( !initialized ) {
    // bail out if we were never initialized (i.e. no route in the config file.)
    return;
  }
  
  float direct_course, direct_distance;
  float leg_distance;

  float gs_mps = sqrt(vn_node->getFloat() * vn_node->getFloat()
                      + ve_node->getFloat() * ve_node->getFloat());
  float track_deg = track_node->getFloat() * r2d;
  double lat_deg = lat_rad_node->getDouble() * r2d;
  double lon_deg = lon_rad_node->getDouble() * r2d;

  if ( !pos_set && gps_fix_node->getInt() == 1 ) {
    printf("Positioning relative waypoints...\n");
    active->refresh_offset_positions(SGWayPoint(lon_deg, lat_deg), 0.0);
    pos_set = true;
  }
    
  // route_node.setLong("route_size", active->size());
  if ( active->size() > 0 ) {
    // route start up logic: if start_mode == first_wpt then
    // there is nothing to do, we simply continue to track wpt
    // 0 if that is the current waypoint.  If start_mode ==
    // "first_leg", then if we are tracking wpt 0, increment
    // it so we track the 2nd waypoint along the first leg.
    // If only a 1 point route is given along with first_leg
    // startup behavior, then don't do that again, force some
    // sort of sane route parameters instead!
    if ( (start_mode == FIRST_LEG)
         && (active->get_waypoint_index() == 0) ) {
      if ( active->size() > 1 ) {
        active->increment_current();
      } else {
        start_mode = FIRST_WPT;
      }
    }

    // track current waypoint of route (only if we have fresh gps data)
    SGWayPoint prev = active->get_previous();
    SGWayPoint wp = active->get_current();

    // compute direct-to course and distance
    wp.CourseAndDistance( lon_deg, lat_deg,
                          &direct_course, &direct_distance );

    // compute leg course and distance
    float leg_course;
    wp.CourseAndDistance( prev, &leg_course, &leg_distance );

    // difference between ideal (leg) course and direct course
    float angle = leg_course - direct_course;
    if ( angle < -180.0 ) {
      angle += 360.0;
    } else if ( angle > 180.0 ) {
      angle -= 360.0;
    }

    // compute course error
    float course_error = leg_course - track_deg;
    if ( course_error < -180.0 ) {
      course_error += 360.0;
    } else if ( course_error > 180.0 ) {
      course_error -= 360.0;
    }
    course_error_node->setFloat(course_error * d2r);
        
    // compute cross-track error
    float angle_rad = angle * d2r;
    float xtrack_m = sin( angle_rad ) * direct_distance;
    float dist_m = cos( angle_rad ) * direct_distance;
    /* printf("direct_dist = %.1f angle = %.1f dist_m = %.1f\n",
       direct_distance, angle, dist_m); */

    xtrack_node->setFloat(xtrack_m);
    nav_dist_node->setFloat(dist_m);

    // compute navigation course with xtrack correction
    float xtrack_comp = xtrack_m * xtrack_gain;
    if ( xtrack_comp < -45.0 ) { xtrack_comp = -45.0; }
    if ( xtrack_comp > 45.0 ) { xtrack_comp = 45.0; }
    float nav_course = leg_course - xtrack_comp;
    if ( nav_course < 0.0 ) {
      nav_course += 360.0;
    } else if ( nav_course > 360.0 ) {
      nav_course -= 360.0;
    }
    // compute navigation course error
    float nav_course_error = nav_course - track_deg;
    if ( nav_course_error < -180.0 ) {
      nav_course_error += 360.0;
    } else if ( nav_course_error > 180.0 ) {
      nav_course_error -= 360.0;
    }
    nav_course_error_node->setFloat(nav_course_error * d2r);
    
    // default distance for waypoint acquisition = direct
    // distance to the target waypoint.  This can be
    // overridden later by leg following and replaced with
    // distance remaining along the leg.
    //nav_dist_m = direct_distance;
    float nav_dist_m = dist_m;

    // static int count = 0;
    // if ( count++ > 10 ) {
    //   printf("crs:%.0f err:%.0f xtrk:%.1f dist:%.0f\n", leg_course, course_error, xtrack_m, nav_dist_m);
    //   count = 0;
    // }

    // estimate distance remaining to completion of route
    dist_remaining_m = nav_dist_m
      + active->get_remaining_distance_from_current_waypoint();

    // logic to mark completion of leg and move to next leg.
    if ( completion_mode == LOOP ) {
      if ( nav_dist_m < 50.0 ) {
        active->set_acquired( true );
        active->increment_current();
      }
    } else if ( completion_mode == EXTEND_LAST_LEG ) {
      if ( nav_dist_m < 50.0 ) {
        active->set_acquired( true );
        if ( active->get_waypoint_index() < active->size() - 1 ) {
          active->increment_current();
        } else {
          // follow the last leg forever
        }
      }
    }

    // publish current target waypoint
    // route_node.setLong( "target_waypoint_idx",
    //                     active->get_waypoint_index() );

  } else {
    // FIXME: we've been commanded to follow a route, but no route
    // has been defined.  Assert something?  Print a warning message?
  }

  if ( gs_mps > 0.1 ) {
    // route_node.setDouble( "wp_eta_sec", direct_distance / gs_mps );
  } else {
    // route_node.setDouble( "wp_eta_sec", 0.0 );
  }
}


bool FGRouteMgr::swap() {
  if ( !standby->size() ) {
    // standby route is empty
    return false;
  }

  // swap standby <=> active routes
  SGRoute *tmp = active;
  active = standby;
  standby = tmp;

  // set target way point to the first waypoint in the new active route
  active->set_current( 0 );
  pos_set = false;

  return true;
}


// build a route from a property (sub) tree
bool FGRouteMgr::build( const rapidjson::Value& Config ) {
  standby->clear();
  if ( Config.HasMember("waypoints") ) {
    const rapidjson::Value& waypoints = Config["waypoints"];
    for (rapidjson::Value::ConstValueIterator it = waypoints.Begin(); it != waypoints.End(); ++it) {
      SGWayPoint wpt(*it);
      standby->add_waypoint(wpt);
    }
  }
  printf("loaded %d waypoints\n", standby->size());
  return true;
}


int FGRouteMgr::new_waypoint( const double field1, const double field2,
			      const int mode )
{
  if ( mode == 0 ) {
    // relative waypoint
    SGWayPoint wp( field2, field1, SGWayPoint::RELATIVE );
    standby->add_waypoint( wp );
  } else if ( mode == 1 ) {
    // absolute waypoint
    SGWayPoint wp( field1, field2, SGWayPoint::ABSOLUTE );
    standby->add_waypoint( wp );
  }

  return 1;
}
