/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "uNavINS.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"
#include "generic-function.h"
#include <sys/time.h>
#include <iostream>

/*
15 State EKF INS - 15 State Extended Kalman Filter Inertial Navigation System,
Estimates inertial position and velocity and estimates gyro and accelerometer bias
using GPS and IMU input
Example JSON configuration:
{
  "Type": "EKF15StateINS",
  "Output": "OutputName",
  "Time": X,
  "GPS": X,
  "IMU": X
}
Where:
   * Output gives a convenient name for the block (i.e. EKF).
   * Time is the time data source
   * GPS is the GPS data source
   * IMU is the IMU data source
*/
class Ekf15StateIns: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    ElementPtr time_node;
    ElementPtr GpsTow, GpsFix;
    ElementPtr GpsVn, GpsVe, GpsVd;
    ElementPtr GpsLat, GpsLon, GpsAlt;
    ElementPtr ImuGx, ImuGy, ImuGz;
    ElementPtr ImuAx, ImuAy, ImuAz;
    ElementPtr ImuHx, ImuHy, ImuHz;

    ElementPtr Mode_node, Fix;
    ElementPtr Ax, Ay, Az;
    ElementPtr Gx, Gy, Gz;
    ElementPtr Axb, Ayb, Azb;
    ElementPtr Gxb, Gyb, Gzb;
    ElementPtr Pitch, Roll, Heading, Track;
    ElementPtr Lat, Lon, Alt;
    ElementPtr Vn, Ve, Vd;

    uNavINS uNavINS_;
    bool Initialized_ = false;
    Vector3d pGpsMeas_D_rrm;
    Vector3f gyroMeas_B_rps, accelMeas_B_mps2, magMeas_B_uT, vGpsMeas_L_mps;
};
