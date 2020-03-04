/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

#include "route_mgr.h"
#include <iostream>

// Route Manager
void RouteMgr::Configure(const rapidjson::Value& RouteConfig) {

  // Configure Input Definitions
  if (!RouteConfig.HasMember("InputDef")) { // ControlDef not defined
    throw std::runtime_error(std::string("ERROR - InputDef not found in Route."));
  }
  const rapidjson::Value& InputDef = RouteConfig["InputDef"];

  LoadInput(InputDef, RootPath_, "Lat", &NodeIn_.Lat, &NodeIn_.LatKey);
  LoadInput(InputDef, RootPath_, "Lon", &NodeIn_.Lon, &NodeIn_.LonKey);
  LoadInput(InputDef, RootPath_, "Alt", &NodeIn_.Alt, &NodeIn_.AltKey);
  LoadInput(InputDef, RootPath_, "Heading", &NodeIn_.Heading, &NodeIn_.HeadingKey);

  // Configure Output Definitions
  if (!RouteConfig.HasMember("OutputDef")) { // ControlDef not defined
    throw std::runtime_error(std::string("ERROR - OutputDef not found in Route."));
  }
  const rapidjson::Value& OutputDef = RouteConfig["OutputDef"];

  LoadOutput(OutputDef, RootPath_, "RefAlt", &NodeOut_.RefAlt);
  LoadOutput(OutputDef, RootPath_, "Crosstrack", &NodeOut_.CrossTrack);
  LoadOutput(OutputDef, RootPath_, "HeadingError", &NodeOut_.HeadingError);

  // Configure Reference Waypoints
  if (!RouteConfig.HasMember("WaypointDef")) { // WaypointDef not defined
    throw std::runtime_error(std::string("ERROR - WaypointDef not found in Route."));
  }
  const rapidjson::Value& WaypointDef = RouteConfig["WaypointDef"];

  // Loop through each of the defined Waypoints
  for (rapidjson::Value::ConstMemberIterator WaypointDefInst = WaypointDef.MemberBegin(); WaypointDefInst != WaypointDef.MemberEnd(); ++WaypointDefInst) {
    // Get the (Name, Object)
    std::string WaypointName = WaypointDefInst->name.GetString();
    const rapidjson::Value& WaypointConfig = WaypointDefInst->value;

    // Read the Waypoint values
    Vector3d WaypointVal;
    LoadVal(WaypointConfig, "Waypoint", &WaypointVal, true);

    // Read the Waypoint
    std::string WaypointRef;

    if (WaypointName == "Home") {
      pHome_D_rrm_ = WaypointVal;

      // Read the Waypoint Reference type
      LoadVal(WaypointConfig, "WaypointRef", &WaypointRef, true);

      if (WaypointRef == "WGS84_deg") { // GPS coordinates [deg, deg, m]
        pHome_D_rrm_.segment(0, 2) *= M_PI/180.0; // convert to [rad, rad, m]
      } else if (WaypointRef == "WGS84") { // GPS coordinates [rad, rad, m]
        // Do nothing
      } else {
        throw std::runtime_error(std::string("ERROR - ") + std::string("WaypointDef - Home WaypointRef must be WGS84 or WGS84_deg."));
      }

      // Retain the Home position in Geodetic, ECEF, and as a DCM
      pHome_E_m_ = D2E(pHome_D_rrm_); // ECEF position of Home
      T_E2L_ = TransE2L(pHome_D_rrm_).cast <float> (); // Compute ECEF to NED with double precision, cast to float
    } // if "Home"

    // WaypointNames_.push_back(WaypointName);
    // WaypointMap_.make_pair(WaypointName, WaypointVal.cast<float>()); // a float precision will get stored into the map
  } // for WaypointDef

  // Route Definitions
  if (!RouteConfig.HasMember("RouteDef")) { // RouteDef not defined
    throw std::runtime_error(std::string("ERROR - RouteDef not found in Route."));
  }
  const rapidjson::Value& RouteDef = RouteConfig["RouteDef"];

  // Loop through each of the defined Routes
  for (rapidjson::Value::ConstMemberIterator RouteDefInst = RouteDef.MemberBegin(); RouteDefInst != RouteDef.MemberEnd(); ++RouteDefInst) {
    // Get the (Name, Object)
    std::string RouteName = RouteDefInst->name.GetString();
    const rapidjson::Value& RouteDefConfig = RouteDefInst->value;

    // Add the Name to the Name Vector
    RouteNames_.push_back(RouteName);

    // Route type
    std::string RouteType;
    LoadVal(RouteDefConfig, "Type", &RouteType, true);

    // Create the Component Object, add to Map
    if (RouteType == "CircleHold") {
      RouteMap_.insert(std::make_pair(RouteName, std::make_shared<RouteCircleHold>()));
    } else if (RouteType == "Waypoints") {
      RouteMap_.insert(std::make_pair(RouteName, std::make_shared<RouteWaypoints>()));
    // } else if (RouteType == "Land") {
    //   RouteMap_.insert(std::make_pair(RouteName, std::make_shared<RouteLand>()));
    } else {
      throw std::runtime_error(std::string("ERROR - ") + RouteType + std::string(": Route Type does not match known types."));
    } // if Type

    // Call Configuration for Component
    RouteMap_[RouteName]->Configure(RouteDefConfig);

  } // for RouteDef
}

void RouteMgr::Run() {
  // Get the Current Position, Geodetic [rad, rad, m]
  Vector3d pCurr_D_rrm;
  pCurr_D_rrm[0] = NodeIn_.Lat->getDouble();
  pCurr_D_rrm[1] = NodeIn_.Lon->getDouble();
  pCurr_D_rrm[2] = NodeIn_.Alt->getDouble();

  // Convert Geodetic to NED
  Vector3f pCurr_L_m = T_E2L_ * (D2E(pCurr_D_rrm) - pHome_E_m_).cast <float> ();// Compute position error double precision, cast to float, apply transformation

  Vector3f pAdj_L_m; pAdj_L_m.setZero();
  Vector3f pLead_L_m; pLead_L_m.setZero();
  if (RouteSel_ != "None") { // If "None" just skip
    // Run the selected route
    RouteMap_[RouteSel_]->Run(pCurr_L_m);

    // Get the Adjacent and Lead Waypoints
    pAdj_L_m = RouteMap_[RouteSel_]->Get_Adj();
    pLead_L_m = RouteMap_[RouteSel_]->Get_Lead();
  }

  // Compute Crosstrack
  Vector3f vAdj2Curr = pCurr_L_m - pAdj_L_m; // pCurr wrt pAdj
  float crosstrack_m = vAdj2Curr[1]; // Cross track error is positive if pCurr to the right of line segment

  // Compute Heading to Lead Point
  Vector3f vCurr2Lead = pLead_L_m - pCurr_L_m; // pLead wrt pCurr
  // float dist2Lead_m = vCurr2Lead.norm();
  float heading2Lead_rad = atan2(vCurr2Lead[1], vCurr2Lead[0]);

  // Compute Heading error
  float heading_rad = NodeIn_.Heading->getFloat();
  float headingErr_rad = WrapToPi(heading2Lead_rad - heading_rad);

  NodeOut_.RefAlt->setFloat(pAdj_L_m[2]);
  NodeOut_.CrossTrack->setFloat(crosstrack_m);
  NodeOut_.HeadingError->setFloat(headingErr_rad);
}


/* Route Type - Circle Hold */
void RouteCircleHold::Configure(const rapidjson::Value& RouteConfig) {
  LoadVal(RouteConfig, "Radius", &distRadius_m_, true);
  LoadVal(RouteConfig, "Direction", &Direction_, true);
  LoadVal(RouteConfig, "Waypoint", &pCenter_L_m_, true);
  LoadVal(RouteConfig, "LeadDist", &distLead_m_, true);
}

void RouteCircleHold::Run(Vector3f pCurr_L_m) {
  Vector3f vCenter2Curr_m = pCurr_L_m - pCenter_L_m_; // Vector from pCenter to pCurr
  Vector3f vCenter2Curr_nd = vCenter2Curr_m / vCenter2Curr_m.norm();

  // Adj Point, Radius from Center along unit vector
  pAdj_L_m_ = distRadius_m_ * vCenter2Curr_nd ;

  // Heading of the Circle at Adj
  if (Direction_ == "Left") {
    headingSeg_rad_ = atan2(-vCenter2Curr_nd[0], vCenter2Curr_nd[1]);
  } else { // Right
    headingSeg_rad_ = atan2(vCenter2Curr_nd[0], -vCenter2Curr_nd[1]);
  }

  // Compute the angle around circle to Lead, distLead ahead of pAdj
  float angleLead_rad = distLead_m_ / distRadius_m_;
  if (Direction_ == "Left") {
    angleLead_rad = -angleLead_rad;
  }

  // Compute the Lead point, rotate vCenter2Curr_nd by angleLead
  Matrix3f T;
  T = AngleAxisf(angleLead_rad, Vector3f::UnitZ());
  pLead_L_m_ = pCenter_L_m_ + (T * vCenter2Curr_nd) * distRadius_m_;
}


/* Route Type - Waypoints */
void RouteWaypoints::Configure(const rapidjson::Value& RouteConfig) {
  LoadVal(RouteConfig, "WaypointList", &WaypointList_L_, true);
  LoadVal(RouteConfig, "LeadDist", &distLead_m_, true);

  numWaypoints_ = WaypointList_L_.size();
}

void RouteWaypoints::ComputeSegment() {
  // Ensure indx in range
  if (indxSeg_ > (numWaypoints_-1)) {
    indxSeg_ = 0;
  }

  // Set index of Previous Waypoint
  indxPrev_ = indxSeg_;

  // Set index of Previous Waypoint, make sure it is in range
  indxNext_ = indxPrev_ + 1;
  if (indxNext_ > (numWaypoints_-1)) {
    indxNext_ = 0;
  }

  // Retrieve Waypoint from the List
  pPrev_L_m_ = WaypointList_L_[indxPrev_];
  pNext_L_m_ = WaypointList_L_[indxNext_];

  // Segment values
  Vector3f vSeg_L_m = pNext_L_m_ - pPrev_L_m_; // Vector along segment
  lenSeg_m_ = vSeg_L_m.norm(); // Length of segment
  vSegUnit_L_ = vSeg_L_m / lenSeg_m_; // unit vector along segment

  headingSeg_rad_ = atan2(vSeg_L_m[1], vSeg_L_m[0]);
}

void RouteWaypoints::Run(Vector3f pCurr_L_m) {
  // pCurr along segment to Next
  Vector3f vCurr2Next = pNext_L_m_ - pCurr_L_m;
  // Vector3f vAdj2Next = vCurr2Next.() * vSegUnit_L_ ;
  Vector3f vAdj2Next = vCurr2Next.cwiseProduct(vSegUnit_L_) ;

  // Check if we're past Next, or within the threshold
  float distNextThresh = 1.0;
  if (vAdj2Next[0] <= distNextThresh) { // pCurr is beyond pNext
    indxSeg_ = indxNext_; // advance indx to Next
    ComputeSegment(); // Compute segment values

    vCurr2Next = pNext_L_m_ - pCurr_L_m;
    vAdj2Next = vCurr2Next.cwiseProduct(vSegUnit_L_) ;
  }

    Vector3f vPrev2Next = pNext_L_m_ - pPrev_L_m_;
std::cout << "Index: " << indxSeg_ << std::endl;
std::cout << "Dist to go along Seg: " << vAdj2Next[0] << std::endl;
std::cout << "Heading Prev2Next Seg: " << WrapTo2Pi (atan2(vPrev2Next[1], vPrev2Next[0])) *180.0/M_PI << std::endl;

  // pCurr along segment from Previous
  Vector3f vPrev2Curr = pCurr_L_m - pPrev_L_m_;
  Vector3f vPrev2Adj = vPrev2Curr.norm() * vSegUnit_L_ ;

  // Compute the location of pAdj, Point along the segment
  pAdj_L_m_ = pPrev_L_m_ + vPrev2Adj;
  Vector3f vCurr2Adj = pAdj_L_m_ - pCurr_L_m;
  std::cout << "Distance to Adj: " << vCurr2Adj.norm() << std::endl;
  std::cout << "Heading to Adj: " << WrapTo2Pi (atan2(vCurr2Adj[1], vCurr2Adj[0])) *180.0/M_PI << std::endl;

  // Compute the Lead point Location, distLead ahead of pAdj
  pLead_L_m_ = pAdj_L_m_ + (distLead_m_ * vSegUnit_L_);

  Vector3f vCurr2Lead = pLead_L_m_ - pCurr_L_m;
std::cout << "Distance to go: " << vCurr2Lead.norm() << std::endl;
std::cout << "Heading to Lead: " << WrapTo2Pi (atan2(vCurr2Lead[1], vCurr2Lead[0])) *180.0/M_PI << std::endl;
}
