/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
*/

#include "route_mgr.h"

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
  LoadInput(InputDef, RootPath_, "VelNorth", &NodeIn_.vNorth, &NodeIn_.vNorthKey);
  LoadInput(InputDef, RootPath_, "VelEast", &NodeIn_.vEast, &NodeIn_.vEastKey);
  LoadInput(InputDef, RootPath_, "VelDown", &NodeIn_.vDown, &NodeIn_.vDownKey);
  LoadInput(InputDef, RootPath_, "Heading", &NodeIn_.Heading, &NodeIn_.HeadingKey);

  // Configure Output Definitions
  if (!RouteConfig.HasMember("OutputDef")) { // ControlDef not defined
    throw std::runtime_error(std::string("ERROR - OutputDef not found in Route."));
  }
  const rapidjson::Value& OutputDef = RouteConfig["OutputDef"];

  LoadOutput(OutputDef, RootPath_, "AltRef", &NodeOut_.AltRef);
  LoadOutput(OutputDef, RootPath_, "AltError", &NodeOut_.AltError);
  LoadOutput(OutputDef, RootPath_, "Crosstrack", &NodeOut_.CrossTrack);
  LoadOutput(OutputDef, RootPath_, "HeadingRef", &NodeOut_.HeadingRef);
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
      T_E2NED_ = TransE2NED(pHome_D_rrm_).cast <float> (); // Compute ECEF to NED with double precision, cast to float
    } // if "Home"

    // Retain Names in a Vector and Waypoints in a map
    WaypointNames_.push_back(WaypointName);
    WaypointMap_.insert(std::make_pair(WaypointName, WaypointVal.cast<float>())); // a float precision will get stored into the map
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

  // Get the Current heading, radians
  float heading_rad = NodeIn_.Heading->getFloat();

  // Get the Current Velocity, NED [m/s]
  Vector3f vCurr_NED_mps;
  vCurr_NED_mps[0] = NodeIn_.vNorth->getFloat();
  vCurr_NED_mps[1] = NodeIn_.vEast->getFloat();
  vCurr_NED_mps[2] = NodeIn_.vDown->getFloat();

  // Convert poition from Geodetic to NED
  Vector3f pCurr_NED_m = T_E2NED_ * (D2E(pCurr_D_rrm) - pHome_E_m_).cast <float> ();// Compute position error double precision, cast to float, apply transformation

  Vector3f pAdj_NED_m; pAdj_NED_m.setZero();
  Vector3f pLead_NED_m; pLead_NED_m.setZero();
  Vector3f pTrail_NED_m; pTrail_NED_m.setZero();
  bool holdFlag; holdFlag = false;
  if (RouteSel_ != "None") { // If "None" just skip
    // Run the selected route
    RouteMap_[RouteSel_]->Run(pCurr_NED_m, vCurr_NED_mps);

    // Get the Adjacent and Lead Waypoints
    pAdj_NED_m = RouteMap_[RouteSel_]->Get_Adj();
    pLead_NED_m = RouteMap_[RouteSel_]->Get_Lead();
    pTrail_NED_m = RouteMap_[RouteSel_]->Get_Trail();
    holdFlag = RouteMap_[RouteSel_]->Get_HoldFlag();
  }

  // Compute Heading reference, between trail and lead
  Vector3f vecTrail2Lead = pLead_NED_m - pTrail_NED_m; // pLead wrt pTrail
  float headingRef_rad = atan2(vecTrail2Lead[1], vecTrail2Lead[0]);

  // Compute Heading to Lead Point
  Vector3f vecCurr2Lead = pLead_NED_m - pCurr_NED_m; // pLead wrt pCurr
  // float dist2Lead_m = vecCurr2Lead.norm();
  float headingLead_rad = atan2(vecCurr2Lead[1], vecCurr2Lead[0]);

  // Compute Heading error
  float headingErr_rad = WrapToPi(headingLead_rad - heading_rad);

  // Compute Crosstrack
  Vector3f vecAdj2Curr = pCurr_NED_m - pAdj_NED_m; // pCurr wrt pAdj
  float crosstrack_m = vecAdj2Curr[1]; // Cross track error is positive if pCurr to the right of line segment

  // If the crosstrack is large, or heading is more the 90deg out, then zero the crosstrack
  if ((holdFlag == false) | (abs(headingErr_rad) > M_PI/2.0)) {
    crosstrack_m = 0.0;
  }

  // Outputs to Nodes
  NodeOut_.AltRef->setFloat(-pAdj_NED_m[2]);
  NodeOut_.AltError->setFloat(-pAdj_NED_m[2] - -pCurr_NED_m[2]);
  NodeOut_.CrossTrack->setFloat(crosstrack_m);
  NodeOut_.HeadingRef->setFloat(headingRef_rad);
  NodeOut_.HeadingError->setFloat(headingErr_rad);
}


/* Route Type - Circle Hold */
void RouteCircleHold::Configure(const rapidjson::Value& RouteConfig) {
  LoadVal(RouteConfig, "Radius", &distRadius_m_, true);
  LoadVal(RouteConfig, "Direction", &Direction_, true);
  LoadVal(RouteConfig, "Waypoint", &pCenter_NED_m_, true);
  LoadVal(RouteConfig, "LeadTime", &tLead_s_, true);
  LoadVal(RouteConfig, "HoldDist", &distHold_m_, true);
}

void RouteCircleHold::Run(Vector3f pCurr_NED_m, Vector3f vCurr_NED_mps) {
  Vector3f vecCenter2Curr_m = pCurr_NED_m - pCenter_NED_m_; // Vector from pCenter to pCurr

  // Adj Point, Radius from Center along vector from Center to Current
  pAdj_NED_m_.segment(0,2) = (distRadius_m_ / vecCenter2Curr_m.segment(0,2).norm()) * vecCenter2Curr_m.segment(0,2) ;
  pAdj_NED_m_[2] = pCenter_NED_m_[2];


  // Heading of the Circle at Adj
  if (Direction_ == "Left") {
    headingSeg_rad_ = atan2(-vecCenter2Curr_m[0], vecCenter2Curr_m[1]);
  } else { // Right
    headingSeg_rad_ = atan2(vecCenter2Curr_m[0], -vecCenter2Curr_m[1]);
  }

  // Velocity along segment
  Matrix3f T_seg;
  T_seg = AngleAxisf(headingSeg_rad_, Vector3f::UnitZ());
  Vector3f vSeg_mps = T_seg * vCurr_NED_mps;

  float distLead_m = vSeg_mps[0] * tLead_s_;

  // Compute Crosstrack, set Hold/Agcuire Flag
  Vector3f vecAdj2Curr = pCurr_NED_m - pAdj_NED_m_; // pCurr wrt pAdj
  float crosstrack_m = vecAdj2Curr[1]; // Cross track error is positive if pCurr to the right of line segment

  holdFlag_ = false;
  if (fabs(crosstrack_m) < distHold_m_) {
    holdFlag_ = true;
  }

  // Compute the angle around circle to Lead, distLead ahead of pAdj
  float angleLead_rad = distLead_m / distRadius_m_;
  if (Direction_ == "Left") {
    angleLead_rad = -angleLead_rad;
  }

  // Compute the Lead point and Trail point, rotate vecCenter2Adj_m by angleLead
  Matrix3f T_Lead;
  T_Lead = AngleAxisf(angleLead_rad, Vector3f::UnitZ());
  Vector3f vecCenter2Adj_m = pAdj_NED_m_ - pCenter_NED_m_;
  pLead_NED_m_ = pCenter_NED_m_ + (T_Lead * vecCenter2Adj_m);
  pTrail_NED_m_ = pCenter_NED_m_ - (T_Lead * vecCenter2Adj_m);
}


/* Route Type - Waypoints */
void RouteWaypoints::Configure(const rapidjson::Value& RouteConfig) {
  LoadVal(RouteConfig, "WaypointList", &WaypointList_NED_, true);
  LoadVal(RouteConfig, "LeadTime", &tLead_s_, true);
  LoadVal(RouteConfig, "HoldDist", &distHold_m_, true);

  numWaypoints_ = WaypointList_NED_.size();
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
  pPrev_NED_m_ = WaypointList_NED_[indxPrev_];
  pNext_NED_m_ = WaypointList_NED_[indxNext_];

  // Segment values
  Vector3f vecSeg_NED_m = pNext_NED_m_ - pPrev_NED_m_; // Vector along segment
  lenSeg_m_ = vecSeg_NED_m.norm(); // Length of segment
  vecSegUnit_NED_ = vecSeg_NED_m / lenSeg_m_; // unit vector along segment
}

void RouteWaypoints::Run(Vector3f pCurr_NED_m, Vector3f vCurr_NED_mps) {
  // pCurr along segment to Next
  Vector3f vecCurr2Next = pNext_NED_m_ - pCurr_NED_m;
  Vector3f vecvAdj2Next = vecCurr2Next.cwiseProduct(vecSegUnit_NED_) ;

  // Check if we're past Next, or within the threshold
  float distNextThresh = 1.0;
  if (vecvAdj2Next[0] <= distNextThresh) { // pCurr is beyond pNext
    indxSeg_ = indxNext_; // advance indx to Next
    ComputeSegment(); // Compute segment values

    vecCurr2Next = pNext_NED_m_ - pCurr_NED_m;
    vecvAdj2Next = vecCurr2Next.cwiseProduct(vecSegUnit_NED_) ;
  }

  // pCurr along segment from Previous
  Vector3f vecPrev2Curr = pCurr_NED_m - pPrev_NED_m_;
  Vector3f vecPrev2Adj = vecPrev2Curr.norm() * vecSegUnit_NED_ ;

  // Compute the location of pAdj, Point along the segment
  pAdj_NED_m_ = pPrev_NED_m_ + vecPrev2Adj;

  // Compute Crosstrack, set Hold/Agcuire Flag
  Vector3f vecAdj2Curr = pCurr_NED_m - pAdj_NED_m_; // pCurr wrt pAdj
  float crosstrack_m = vecAdj2Curr[1]; // Cross track error is positive if pCurr to the right of line segment

  holdFlag_ = false;
  if (fabs(crosstrack_m) < distHold_m_) {
    holdFlag_ = true;
  }

  // Velocity along segment
  Vector3f vSeg_mps = vCurr_NED_mps.cwiseProduct(vecSegUnit_NED_);
  float distLead_m = vSeg_mps[0] * tLead_s_;

  // Compute the Lead point and Trail point Locations, distLead from of pAdj
  pLead_NED_m_ = pAdj_NED_m_ + (distLead_m * vecSegUnit_NED_);
  pTrail_NED_m_ = pAdj_NED_m_ - (distLead_m * vecSegUnit_NED_);
}
