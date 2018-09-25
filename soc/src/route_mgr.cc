// route_mgr.cc - manage a route

#include <math.h>
#include <stdlib.h>

//#include "comms/logging.hxx"
//#include "comms/remote_link.hxx"
//#include "include/globaldefs.h"
//#include "init/globals.hxx"
//#include "sensors/gps_mgr.hxx"
//#include "util/exception.hxx"
//#include "util/sg_path.hxx"
//#include "util/strutils.hxx"
//#include "util/wind.hxx"

#include "waypoint.hxx"
#include "route_mgr.hxx"


static const double r2d = 180.0 / M_PI;
static const double d2r = M_PI / 180.0;

static float *vn_ptr = NULL;
static float *ve_ptr = NULL;
static double *lat_rad_ptr = NULL;
static double *lon_rad_ptr = NULL;


FGRouteMgr::FGRouteMgr() :
    active( new SGRoute ),
    standby( new SGRoute ),
    last_lon( 0.0 ),
    last_lat( 0.0 ),
    last_az( 0.0 ),
    start_mode( FIRST_WPT ),
    completion_mode( LOOP ),
    dist_remaining_m( 0.0 )
{
    bind();
}


FGRouteMgr::~FGRouteMgr() {
    delete standby;
    delete active;
}


// bind property nodes
void FGRouteMgr::bind() {
    // property nodes
    //pos_node = pyGetNode("/position", true);
    //vel_node = pyGetNode("/velocity", true);
    //orient_node = pyGetNode("/orientation", true);
    //route_node = pyGetNode("/task/route", true);
    //home_node = pyGetNode("/task/home", true);
    //targets_node = pyGetNode("/autopilot/targets", true);

    // defaults
    //route_node.setString("start_mode", "first_wpt");
    //route_node.setString("completion_mode", "loop");
}


void FGRouteMgr::init( const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr ) {
    vn_ptr = DefinitionTreePtr->GetValuePtr<float*>("/Sensor-Processing/NorthVelocity_ms");
    ve_ptr = DefinitionTreePtr->GetValuePtr<float*>("/Sensor-Processing/EastVelocity_ms");
    lat_rad_ptr = DefinitionTreePtr->GetValuePtr<double*>("/Sensor-Processing/Latitude_rad");
    lon_rad_ptr = DefinitionTreePtr->GetValuePtr<double*>("/Sensor-Processing/Longitude_rad");

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
}


void FGRouteMgr::update() {
    double direct_course, direct_distance;
    double leg_course, leg_distance;

    double nav_course = 0.0;
    double nav_dist_m = 0.0;

    double wp_agl_m = 0.0;
    double wp_msl_m = 0.0;

    double gs_mps = sqrt(*vn_ptr * *vn_ptr + *ve_ptr * *ve_ptr);

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
        double lat_deg = *lat_rad_ptr * r2d;
        double lon_deg = *lon_rad_ptr * r2d;
        wp.CourseAndDistance( lon_deg, lat_deg,
                              &direct_course, &direct_distance );

        // compute leg course and distance
        wp.CourseAndDistance( prev, &leg_course, &leg_distance );

        // difference between ideal (leg) course and direct course
        double angle = leg_course - direct_course;
        if ( angle < -180.0 ) {
            angle += 360.0;
        } else if ( angle > 180.0 ) {
            angle -= 360.0;
        }

        // compute cross-track error
        double angle_rad = angle * d2r;
        double xtrack_m = sin( angle_rad ) * direct_distance;
        double dist_m = cos( angle_rad ) * direct_distance;
        /* printf("direct_dist = %.1f angle = %.1f dist_m = %.1f\n",
           direct_distance, angle, dist_m); */
        //route_node.setDouble( "xtrack_dist_m", xtrack_m );
        //route_node.setDouble( "projected_dist_m", dist_m );

        // default distance for waypoint acquisition = direct
        // distance to the target waypoint.  This can be
        // overridden later by leg following and replaced with
        // distance remaining along the leg.
        nav_dist_m = direct_distance;

        // estimate distance remaining to completion of route
        dist_remaining_m = nav_dist_m
            + active->get_remaining_distance_from_current_waypoint();
        // route_node.setDouble("dist_remaining_m", dist_remaining_m);

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
        // has been defined.

        // We are in ill-defined territory, should we do some sort of
        // circle of our home position?

	// FIXME: need to go to circle mode somehow here!!!!
	/* mission_mgr.request_task_circle(); */
    }

    // route_node.setDouble( "wp_dist_m", direct_distance );

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
