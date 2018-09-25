// route_mgr.hxx - manage a route (i.e. a collection of waypoints)
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#include <string>
#include <vector>
using std::string;
using std::vector;

#include "rapidjson/document.h"

#include "definition-tree.hxx"
#include "route.hxx"


/**
 * Top level route manager class
 * 
 */

class FGRouteMgr {

public:

    enum StartMode {
	FIRST_WPT = 0,		// Go to first waypoint
	FIRST_LEG = 1,		// Go to 2nd waypoint along route leg
    };

    enum CompletionMode {
	LOOP = 0,		// loop the route when finished
	EXTEND_LAST_LEG = 1	// track the last route leg indefinitely
    };

private:

    SGRoute *active;
    SGRoute *standby;

    //pyPropertyNode pos_node;
    //pyPropertyNode vel_node;
    //pyPropertyNode orient_node;
    //pyPropertyNode route_node;
    //pyPropertyNode targets_node;
    //pyPropertyNode home_node;
    
    double last_lon;
    double last_lat;
    double last_az;

    // route behaviors
    StartMode start_mode;
    CompletionMode completion_mode;

    // stats
    double dist_remaining_m;

    SGWayPoint make_waypoint( const string& wpt_string );

    // build a route from a property (sub) tree
    bool build( const rapidjson::Value& Config );
    
public:

    FGRouteMgr();
    ~FGRouteMgr();

    void bind();

    void init( const rapidjson::Value& Config,
               DefinitionTree *DefinitionTreePtr );

    // set route start mode
    inline void set_start_mode( enum StartMode mode ) {
	start_mode = mode;
    }

    // set route completion mode
    inline void set_completion_mode( enum CompletionMode mode ) {
	completion_mode = mode;
    }

    void update();

    // swap the "active" and the "standby" routes, but only if the
    // "standby" route has some waypoints.
    bool swap();

    // these modify the "standby" route
    inline void clear_standby() {
	standby->clear();
    }
    int new_waypoint( const double lon, const double lat,
		      const int mode );

    // returns info on the "active" route
    inline SGWayPoint get_waypoint( int i ) const {
        return active->get_waypoint(i);
    }
    inline int get_waypoint_index() const {
	return active->get_waypoint_index();
    }
    inline int size() const {
        return active->size();
    }
    inline double get_dist_remaining_m() const {
	return dist_remaining_m;
    }

    // restart the route from the beginning
    inline void restart() {
	active->set_acquired( false );
	active->set_current( 0 );
    }
};
