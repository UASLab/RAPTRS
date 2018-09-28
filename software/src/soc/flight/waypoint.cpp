// waypoint.cxx -- Class to hold data and return info relating to a waypoint


#include <stdio.h>

#include "wgs84.hxx"
#include "waypoint.hxx"


// Constructor
SGWayPoint::SGWayPoint( const double field1, const double field2,
                        const modetype m ):
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 )
 {
    mode = m;
    if ( mode == ABSOLUTE ) {
	target_lon = field1;
	target_lat = field2;
    } else if ( mode == RELATIVE ) {
	offset_hdg_deg = field1;
	offset_dist_m = field2;
    }
}


SGWayPoint::SGWayPoint( const rapidjson::Value& Config ):
    mode( INVALID ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 )
{
    if ( Config.HasMember("lon") ) {
	target_lon = Config["lon"].GetFloat();
	mode = ABSOLUTE;
    }
    if ( Config.HasMember("lat") ) {
	target_lat = Config["lat"].GetFloat();
	mode = ABSOLUTE;
    }
    if ( Config.HasMember("offset_heading_deg") ) {
	offset_hdg_deg = Config["offset_heading_deg"].GetFloat();
	mode = RELATIVE;
    }
    if ( Config.HasMember("offset_dist_m") ) {
	offset_dist_m = Config["offset_dist_m"].GetFloat();
	mode = RELATIVE;
    }
    if ( mode == INVALID ) {
	printf("Error in waypoint config logic, " );
    } else if ( mode == ABSOLUTE ) {
	printf("WPT: %.8f %.8f\n",
	       target_lon, target_lat);
    } else if ( mode == RELATIVE ) {
	printf("WPT: %4.0f deg %.0fm\n",
	       offset_hdg_deg, offset_dist_m);
    }
}


SGWayPoint::SGWayPoint():
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 )
{
}


// Destructor
SGWayPoint::~SGWayPoint() {
}


// Calculate course and distances.  For WGS84 and SPHERICAL
// coordinates lat, lon, and course are in degrees, alt and distance
// are in meters.
void SGWayPoint::CourseAndDistance( const double cur_lon,
				    const double cur_lat,
				    double *course, double *dist ) {
    double reverse = 0.0;
    geo_inverse_wgs_84(cur_lat, cur_lon, target_lat, target_lon,
                       course, &reverse, dist);
}

// Calculate course and distances between two waypoints
void SGWayPoint::CourseAndDistance( const SGWayPoint &wp,
                                    double *course, double *dist ) {
    CourseAndDistance( wp.get_target_lon(),
		       wp.get_target_lat(),
		       course, dist );
}

/**
 * Update the target_lon and target_lat values of this waypoint
 * based on this waypoint's offset heading and offset distance
 * values.  The new target location is computed relative to the
 * provided reference point and reference heading.
 */
void SGWayPoint::update_relative_pos( const SGWayPoint &ref,
                                      const double ref_heading_deg )
{
    double course = ref_heading_deg + offset_hdg_deg;
    if ( course < 0.0 ) { course += 360.0; }
    if ( course > 360.0 ) { course -= 360.0; }
    course = 360.0 - course; // invert to make this routine happy

    double az2 = 0.0;
    geo_direct_wgs_84(ref.get_target_lat(), ref.get_target_lon(),
                      course, offset_dist_m,
                      &target_lat, &target_lon, &az2);

    // printf("ref_hdg = %.1f offset=%.1f course=%.1f dist=%.1f\n",
    // 	   ref_heading_deg, offset_hdg_deg,
    // 	   360.0 - course, offset_dist_m);
    // printf("ref = %.6f %.6f  new = %.6f %.6f\n",
    // 	   ref.get_target_lon(), ref.get_target_lat(),
    // 	   target_lon, target_lat);
}
