/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

/*
"WaypointRef" is either "WGS84", "WGS84_deg", or a previously named element of "WaypointDef"
All coordinates are provided as NED and internally stored and used in an NED system relative to "Home"
WaypointDef["Home"] is stored in Geodetic [rad,rad,m]

"Route": {
  "InputDef": {
    "Lat": "/Sensor-Processing/Latitude_rad",
    "Lon": "/Sensor-Processing/Longitude_rad",
    "Alt": "/Sensor-Processing/Alt_rad",
    "Heading": "/Sensor-Processing/Heading_rad"
  },

  "OutputDef": {
    "RefAlt": "refAlt_m",
    "Crosstrack": "xTrack_m",
    "RefHeading": "heading_rad"
  },

  "WaypointDef": [
    "Home": {"Waypoint": [Lat, Lon, Alt], "WaypointRef": "WGS84_deg"},
    "Other": {"Waypoint": [0.0, 0.0, -Height]}
  },

  "RouteDef": [
    "Loiter": {
      "Type": "CircleHold", "Radius": 200, "Direction": "Left",
      "Waypoint": [0.0, 0.0, -Height], "LeadTime_s": 2
    },

    "Path001": {
      "Type": "Waypoints", "LeadTime_s": 2
      "WaypointList": [
        [N, E, D],
        [N, E, D],
        [N, E, D] ]
    },

    "LandWest": {
      "Type": "Land", "AltFinal": 50, "GlideSlope": -3, "Heading": 270,
      "Waypoint": [0.0, 10.0, 0.0]
    },

    "LandEast": {
      "Type": "Landing", "AltFinal": 50, "GlideSlope": -3, "Heading": 90,
      "Waypoint": [0.0, -10.0, 0.0]
    }
  ]
}
*/

#pragma once

#include "configuration.h"
#include "definition-tree2.h"
#include "nav-functions.h"

#include "generic-function.h"
#include "general-functions.h"

#include <stdio.h>
#include <vector>

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

// Base Class for Route Types
class RoutePathBase {
  public:
    virtual void Configure(const rapidjson::Value& Config) {}
    virtual void Run(Vector3f pCurr_L_m) {}
    virtual Vector3f Get_Adj() { return pAdj_L_m_; }
    virtual Vector3f Get_Lead() { return pLead_L_m_; }
    virtual void Clear() {}
  private:
    Vector3f pAdj_L_m_;
    Vector3f pLead_L_m_;
};

class RouteCircleHold: public RoutePathBase {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(Vector3f pCurr_L_m);
    inline Vector3f Get_Adj() { return pAdj_L_m_; }
    inline Vector3f Get_Lead() { return pLead_L_m_; }
    void Clear() {}
  private:
    Vector3f pCenter_L_m_;
    float distRadius_m_;
    std::string Direction_;
    float distLead_m_;

    float headingSeg_rad_; // Segment heading

    Vector3f pAdj_L_m_;
    Vector3f pLead_L_m_;
};

// Route Waypoints
class RouteWaypoints: public RoutePathBase {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(Vector3f pCurr_L_m);
    inline Vector3f Get_Adj() { return pAdj_L_m_; }
    inline Vector3f Get_Lead() { return pLead_L_m_; }
    void Clear() {}
  private:
    void ComputeSegment();

    std::vector<Vector3f> WaypointList_L_;
    float distLead_m_;

    size_t numWaypoints_;
    size_t indxSeg_ = 0;
    size_t indxPrev_ = 0;
    size_t indxNext_ = 0;
    Vector3f pPrev_L_m_;
    Vector3f pNext_L_m_;
    float lenSeg_m_;
    Vector3f vSegUnit_L_;
    float headingSeg_rad_; // Segment heading

    Vector3f pAdj_L_m_;
    Vector3f pLead_L_m_;
};

// Route Manager
class RouteMgr {
  public:
    void Configure(const rapidjson::Value& Config);
    void Set_RouteSel(std::string RouteSel) { RouteSel_ = RouteSel; }
    void Run();
    void Clear() {}
  private:
    std::string RootPath_ = "/Route";

    struct StructNodeIn {
      ElementPtr Lat, Lon, Alt, Heading;
      std::string LatKey, LonKey, AltKey, HeadingKey;
    } NodeIn_;

    struct StructNodeOut {
      ElementPtr RefAlt, CrossTrack, HeadingError;
    } NodeOut_;

    Vector3d pHome_D_rrm_;
    Vector3d pHome_E_m_;
    Matrix3f T_E2L_;

    std::vector<std::string> WaypointNames_;
    std::map<std::string, Vector3f> WaypointMap_;

    std::string RouteSel_;
    std::vector<std::string> RouteNames_;
    std::map<std::string, std::shared_ptr<RoutePathBase>> RouteMap_;
};
