// waypoint.hxx -- Provides a class to manage waypoints
//
// Written by Curtis Olson, started September 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@hfrl.umn.edu
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


#pragma once

#include "rapidjson/document.h"

#include <string>
using std::string;


/**
 * A class to manage waypoints.
 */

class SGWayPoint {

public:

  /**
   * Waypoint mode.
   * <li> ABSOLUTE: coordinates are longitude, latitude (deg)
   * <li> RELATIVE: coordinates are distance (m), heading (deg)
   */
  enum modetype {
    INVALID = 0,
    ABSOLUTE = 1,
    RELATIVE = 2
  };

private:

  modetype mode;

  double target_lon;
  double target_lat;

  // relative waypoints will have their actual target_lon & lat
  // computed as an offset angle and distance from a specified
  // reference location and heading.  These values should be set to
  // zero to indicate this waypoint is absolute and the target_lon &
  // lat should not ever be updated.
  double offset_hdg_deg;
  double offset_dist_m;

  double distance;

public:

  /**
   * Construct a waypoint
   * @param field1 target longitude deg if mode == ABSOLUTE
   * @param field2 target latitude deg if mode == ABSOLUTE
   * @param field1 (offset heading deg if mode == RELATIVE
   * @param dist_m (offset distance m if mode == RELATIVE
   * @param mode type of coordinates/math to use
   * @param s waypoint identifier
   */
  SGWayPoint( const double field1, const double field2,
              const modetype m = ABSOLUTE );

  /**
   * Construct a waypoint from a property sub tree
   * @param node a pointer to a property subtree with configuration values
   */
  SGWayPoint( const rapidjson::Value& Config );

  /**
   * Construct a null waypoint
   */
  SGWayPoint();

  /** Destructor */
  ~SGWayPoint();

  /**
   * Calculate course and distances.  For WGS84 and SPHERICAL
   * coordinates lat, lon, and course are in degrees, alt and
   * distance are in meters.
   * @param cur_lon (in) current longitude
   * @param cur_lat (in) current latitude
   * @param course (out) heading from current location to this waypoint
   * @param dist (out) distance from current location to this waypoint
   */
  void CourseAndDistance( const double cur_lon, const double cur_lat,
                          float *course, float *dist );

  /**
   * Calculate course and distances between a specified starting waypoint
   * and this waypoint.
   * @param wp (in) original waypoint
   * @param course (out) heading from current location to this waypoint
   * @param dist (out) distance from current location to this waypoint
   */
  void CourseAndDistance( const SGWayPoint &wp,
                          float *course, float *dist );

  /**
   * Update the target_lon and target_lat values of this waypoint
   * based on this waypoint's offset heading and offset distance
   * values.  The new target location is computed relative to the
   * provided reference point and reference heading.
   * @param ref the reference waypoint
   * @param ref_heading the reference heading/course
   */
  void update_relative_pos( const SGWayPoint &ref,
                            const double ref_heading_deg );

  /** @return waypoint mode */
  inline modetype get_mode() const { return mode; }

  /** @return waypoint longitude */
  inline double get_target_lon() const { return target_lon; }

  /** @return waypoint latitude */
  inline double get_target_lat() const { return target_lat; }

  /** @return offset heading */
  inline double get_offset_hdg_deg() const { return offset_hdg_deg; }
 
  /** @return offset heading */
  inline double get_offset_dist_m() const { return offset_dist_m; }

  /**
   * This value is not calculated by this class.  It is simply a
   * placeholder for the user to store a distance value.  This is useful
   * when you stack waypoints together into a route.  You can calculate the
   * distance from the previous waypoint once and save it here.  Adding up
   * all the distances here plus the distance to the first waypoint gives you
   * the total route length.  Note, you must update this value yourself, this
   * is for your convenience only.
   * @return waypoint distance holder (what ever the user has stashed here)
   */
  inline double get_distance() const { return distance; }

  /**
   * Set the waypoint distance value to a value of our choice.
   * @param d distance 
   */
  inline void set_distance( double d ) { distance = d; }
};
