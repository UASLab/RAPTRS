// route.hxx -- Provides a class to manage a list of waypoints (i.e. a route).
//
// Written by Curtis Olson, started October 2000.
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
//


#pragma once

#include <vector>
using std::vector;

#include "waypoint.h"

/**
 * A class to manage a list of waypoints (i.e. a route).
 */

class SGRoute {

private:

  typedef vector < SGWayPoint > route_list;
  route_list route;
  int current_wp;
  bool acquired;		// optional if the route_mgr wishes to use

  void update_distance(unsigned int index);

public:

  /** Constructor */
  SGRoute();

  /** Destructor */
  ~SGRoute();

  /** Clear the entire route */
  inline void clear() {
    route.clear();
    current_wp = 0;
  }

  /**
   * Add waypoint
   * @param wp a waypoint
   */
  void add_waypoint( const SGWayPoint &wp );

  /**
   * Add waypoint (default), or insert waypoint at position n.
   * @param wp a waypoint
   */
  void replace_waypoint( const SGWayPoint &wp, unsigned int n );

  /**
   * Get the number of waypoints (i.e. route length )
   * @return route length
   */
  inline int size() const { return route.size(); }

  /**
   * Get the front waypoint.
   * @return the first waypoint.
   */
  inline SGWayPoint get_first() const {
    if ( route.size() ) {
      return route[0];
    } else {
      return SGWayPoint();
    }
  }

  /**
   * Get the current waypoint
   * @return the current waypoint
   */
  inline SGWayPoint get_current() const {
    if ( current_wp < (int)route.size() ) {
      return route[current_wp];
    } else {
      return SGWayPoint();
    }
  }

  /**
   * Get the previous waypoint
   * @return the previous waypoint
   */
  inline SGWayPoint get_previous() const {
    int prev_wp = current_wp - 1;
    if ( prev_wp < 0 ) {
      prev_wp = (int)route.size() - 1;
    }
    if ( prev_wp < (int)route.size() ) {
      return route[prev_wp];
    } else {
      return SGWayPoint();
    }
  }

  /**
   * Set the current waypoint
   * @param number of waypoint to make current.
   */
  inline void set_current( int n ) {
    if ( n >= 0 && n < (int)route.size() ) {
      current_wp = n;
    }
  }

  /** Increment the current waypoint pointer, loop back to first
   *  waypoint after route is finished (hardcoded behavior). */
  inline void increment_current() {
    if ( current_wp < (int)route.size() - 1 ) {
      ++current_wp;
    } else {
      current_wp = 0;
    }
  }

  /**
   * Get the nth waypoint
   * @param n waypoint number
   * @return the nth waypoint
   */
  inline SGWayPoint get_waypoint( const int n ) const {
    if ( n < (int)route.size() ) {
      return route[n];
    } else {
      return SGWayPoint();
    }
  }

  /** Return the current waypoint *index* number */
  inline int get_waypoint_index() const {
    return current_wp;
  }

  /** Delete the front waypoint */
  inline void delete_first() { delete_waypoint(0); }

  /** Delete waypoint waypoint with index n  (last one if n < 0) */
  void delete_waypoint( unsigned int n = 0 );

  /** Refresh relative/offset positions of all waypoints in this
   *  route that have relative offsets.  CAUTION: this traverses and
   *  updates the entire route in one shot, so you might see a
   *  performance glitch with very large routes.
   */
  void refresh_offset_positions( const SGWayPoint &ref,
                                 const double ref_heading_deg );

  /** Get the distance remaining in the route starting at the next
      (target) waypoint.  It is up to the calling layer to compute
      the distance to the next waypoint and add it to the remaining
      distance of the route.  The logic here is that the calling
      layer is already computing distance to the next waypoint, and
      then we need to know the distance from the next waypoint to
      the end to compute the total distance to the end of the route.
  */
  double get_remaining_distance_from_current_waypoint();

#if 0
  /**
   * Calculate perpendicular distance from the current route segment
   * This routine assumes all points are laying on a flat plane and
   * ignores the altitude (or Z) dimension.  For most accurate
   * results, use with CARTESIAN way points.
   */
  double distance_off_route( double x, double y ) const;
#endif

  inline bool is_acquired() const {
    return acquired;
  }

  inline void set_acquired( bool val ) {
    acquired = val;
  }
};
