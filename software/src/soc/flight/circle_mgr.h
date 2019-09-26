/*
  Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
*/

#pragma once

#include <string>
using std::string;

#include "rapidjson/document.h"
#include "definition-tree2.h"

#include "waypoint.h"

class CircleMgr {

private:

  bool initialized{false};
  bool pos_set{false};
  SGWayPoint target;
  float direction;
  float radius_m;

public:

  CircleMgr();
  ~CircleMgr();

  void init( const rapidjson::Value& Config );
  void update();

  void set_center( const double lat_deg, const double lon_deg ); // degrees
  void set_direction( const string value ); // "left" or "right"
  void set_radius( const float value );    // meters
};
