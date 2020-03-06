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
    "Crosstrack": "crosstrack_m",
    "HeadingError": "headingErr_rad"
  },

  "WaypointDef": {
    "Home": {"Waypoint": [44.725801, -93.075866, 285.2111], "WaypointRef": "WGS84_deg"}
  },

  "RouteDef": {
    "Loiter": {
      "Type": "CircleHold", "Radius": 400, "Direction": "Left",
      "Waypoint": [0.0, 0.0, -75], "LeadDist": 100
    },

    "Path_1": {
      "Type": "Waypoints", "LeadDist": 100,
      "WaypointList": [
        [400, 0, -75],
        [0, 400, -75],
        [-400, 0, -75],
        [0, -400, -75] ]
    }
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
    virtual void Run(Vector3f pCurr_NED_m) {}
    virtual Vector3f Get_Adj() { return pAdj_NED_m_; }
    virtual Vector3f Get_Lead() { return pLead_NED_m_; }
    virtual Vector3f Get_Trail() { return pTrail_NED_m_; }
    virtual bool Get_HoldFlag() { return holdFlag_; }
    virtual void Clear() {}
  private:
    Vector3f pAdj_NED_m_;
    Vector3f pLead_NED_m_;
    Vector3f pTrail_NED_m_;
    bool holdFlag_ = false; // true is tracking within distHold
};

class RouteCircleHold: public RoutePathBase {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(Vector3f pCurr_NED_m);
    inline Vector3f Get_Adj() { return pAdj_NED_m_; }
    inline Vector3f Get_Lead() { return pLead_NED_m_; }
    inline Vector3f Get_Trail() { return pTrail_NED_m_; }
    inline bool Get_HoldFlag() { return holdFlag_; }
    void Clear() {}
  private:
    Vector3f pCenter_NED_m_;
    float distRadius_m_;
    std::string Direction_;
    float distLead_m_;
    float distHold_m_;
    bool holdFlag_ = false; // true is tracking within distHold

    float headingSeg_rad_; // Segment heading

    Vector3f pAdj_NED_m_;
    Vector3f pLead_NED_m_;
    Vector3f pTrail_NED_m_;
};

// Route Waypoints
class RouteWaypoints: public RoutePathBase {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(Vector3f pCurr_NED_m);
    inline Vector3f Get_Adj() { return pAdj_NED_m_; }
    inline Vector3f Get_Lead() { return pLead_NED_m_; }
    inline Vector3f Get_Trail() { return pTrail_NED_m_; }
    inline bool Get_HoldFlag() { return holdFlag_; }
    void Clear() {}
  private:
    void ComputeSegment();

    std::vector<Vector3f> WaypointList_NED_;
    float distLead_m_;
    float distHold_m_;
    bool holdFlag_ = false; // true if tracking within distHold

    size_t numWaypoints_;
    size_t indxSeg_ = 0;
    size_t indxPrev_ = 0;
    size_t indxNext_ = 0;
    Vector3f pPrev_NED_m_;
    Vector3f pNext_NED_m_;
    float lenSeg_m_;
    Vector3f vSegUnit_NED_;
    float headingSeg_rad_; // Segment heading

    Vector3f pAdj_NED_m_;
    Vector3f pLead_NED_m_;
    Vector3f pTrail_NED_m_;
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
      ElementPtr AltRef, AltError, CrossTrack, HeadingRef, HeadingError;
    } NodeOut_;

    Vector3d pHome_D_rrm_;
    Vector3d pHome_E_m_;
    Matrix3f T_E2NED_;

    std::vector<std::string> WaypointNames_;
    std::map<std::string, Vector3f> WaypointMap_;

    std::string RouteSel_;
    std::vector<std::string> RouteNames_;
    std::map<std::string, std::shared_ptr<RoutePathBase>> RouteMap_;
};
