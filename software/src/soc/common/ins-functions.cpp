/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include <iostream>
using std::cout;
using std::endl;

#include "ins-functions.h"

void Ekf15StateIns::Configure(const rapidjson::Value& Config,std::string SystemPath) {
  // get gps source
  if (Config.HasMember("GPS")) {
    std::string GpsKey = Config["GPS"].GetString();
    std::string FixKey = GpsKey+"/Fix";
    std::string TowKey = GpsKey+"/TOW";
    std::string VnKey = GpsKey+"/NorthVelocity_ms";
    std::string VeKey = GpsKey+"/EastVelocity_ms";
    std::string VdKey = GpsKey+"/DownVelocity_ms";
    std::string LatKey = GpsKey+"/Latitude_rad";
    std::string LonKey = GpsKey+"/Longitude_rad";
    std::string AltKey = GpsKey+"/Altitude_m";

    GpsFix = deftree.getElement(FixKey);
    if ( !GpsFix ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS fix source ")+FixKey+std::string(" not found in global data."));
    }
    GpsTow = deftree.getElement(TowKey);
    if ( !GpsTow ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS TOW source ")+TowKey+std::string(" not found in global data."));
    }
    GpsVn = deftree.getElement(VnKey);
    if ( !GpsVn ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS north velocity source ")+VnKey+std::string(" not found in global data."));
    }
    GpsVe = deftree.getElement(VeKey);
    if ( !GpsVe ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS east velocity source ")+VeKey+std::string(" not found in global data."));
    }
    GpsVd = deftree.getElement(VdKey);
    if ( !GpsVd ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS down velocity source ")+VdKey+std::string(" not found in global data."));
    }
    GpsLat = deftree.getElement(LatKey);
    if ( ! GpsLat ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS latitude source ")+LatKey+std::string(" not found in global data."));
    }
    GpsLon = deftree.getElement(LonKey);
    if ( !GpsLon ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS longitude source ")+LonKey+std::string(" not found in global data."));
    }
    GpsAlt = deftree.getElement(AltKey);
    if ( !GpsAlt ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS altitude source ")+AltKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS source not specified in configuration."));
  }
  // get imu source
  if (Config.HasMember("IMU")) {
    std::string ImuKey = Config["IMU"].GetString();
    std::string GxKey = ImuKey+"/GyroX_rads";
    std::string GyKey = ImuKey+"/GyroY_rads";
    std::string GzKey = ImuKey+"/GyroZ_rads";
    std::string AxKey = ImuKey+"/AccelX_mss";
    std::string AyKey = ImuKey+"/AccelY_mss";
    std::string AzKey = ImuKey+"/AccelZ_mss";
    std::string HxKey = ImuKey+"/MagX_uT";
    std::string HyKey = ImuKey+"/MagY_uT";
    std::string HzKey = ImuKey+"/MagZ_uT";
    ImuGx = deftree.getElement(GxKey);
    if ( !ImuGx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X gyro source ")+GxKey+std::string(" not found in global data."));
    }
    ImuGy = deftree.getElement(GyKey);
    if ( !ImuGy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y gyro source ")+GyKey+std::string(" not found in global data."));
    }
    ImuGz = deftree.getElement(GzKey);
    if ( !ImuGz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z gyro source ")+GzKey+std::string(" not found in global data."));
    }
    ImuAx = deftree.getElement(AxKey);
    if ( !ImuAx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X accelerometer source ")+AxKey+std::string(" not found in global data."));
    }
    ImuAy = deftree.getElement(AyKey);
    if ( !ImuAy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y accelerometer source ")+AyKey+std::string(" not found in global data."));
    }
    ImuAz = deftree.getElement(AzKey);
    if ( !ImuAz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z accelerometer source ")+AzKey+std::string(" not found in global data."));
    }
    ImuHx = deftree.getElement(HxKey);
    if ( !ImuHx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X magnetometer source ")+HxKey+std::string(" not found in global data."));
    }
    ImuHy = deftree.getElement(HyKey);
    if ( !ImuHy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y magnetometer source ")+HyKey+std::string(" not found in global data."));
    }
    ImuHz = deftree.getElement(HzKey);
    if ( !ImuHz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z magnetometer source ")+HzKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": IMU source not specified in configuration."));
  }

  // get time source
  std::string TimeKey;
  LoadInput(Config, SystemPath, "Time", &time_node, &TimeKey);

  float InitTime = 0;
  LoadVal(Config, "Initialization-Time", &InitTime);

  // Get EKF Configurable parameters
  // none of the values should be negative - FIXIT
  float val;
  // Accel
  val = 0; LoadVal(Config, "AccelSigma", &val);
  if (val > 0) uNavINS_.Set_AccelSigma(val);

  val = 0; LoadVal(Config, "AccelMarkov", &val);
  if (val > 0) uNavINS_.Set_AccelMarkov(val);

  val = 0; LoadVal(Config, "AccelTau", &val);
  if (val > 0) uNavINS_.Set_AccelTau(val);

  // Gyro
  val = 0; LoadVal(Config, "GyroSigma", &val);
  if (val > 0) uNavINS_.Set_RotRateSigma(val);

  val = 0; LoadVal(Config, "GyroMarkov", &val);
  if (val > 0) uNavINS_.Set_RotRateMarkov(val);

  val = 0; LoadVal(Config, "GyroTau", &val);
  if (val > 0) uNavINS_.Set_RotRateTau(val);

  // GPS Position
  val = 0; LoadVal(Config, "PosSigmaNE", &val);
  if (val > 0) uNavINS_.Set_PosSigmaNE(val);

  val = 0; LoadVal(Config, "PosSigmaD", &val);
  if (val > 0) uNavINS_.Set_PosSigmaD(val);

  // GPS Velocity
  val = 0; LoadVal(Config, "VelSigmaNE", &val);
  if (val > 0) uNavINS_.Set_VelSigmaNE(val);

  val = 0; LoadVal(Config, "VelSigmaD", &val);
  if (val > 0) uNavINS_.Set_VelSigmaD(val);

  // pointer to log run mode data
  Mode_node = deftree.initElement(SystemPath+"/Mode","Run mode", LOG_UINT8, LOG_NONE);
  Mode_node->setInt(kStandby);

  // pointers to log data
  Ax = deftree.initElement(SystemPath+"/AccelX_mss", "X accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  Axb = deftree.initElement(SystemPath+"/AccelXBias_mss", "X accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  Ay = deftree.initElement(SystemPath+"/AccelY_mss", "Y accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  Ayb = deftree.initElement(SystemPath+"/AccelYBias_mss", "Y accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  Az = deftree.initElement(SystemPath+"/AccelZ_mss", "Z accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  Azb = deftree.initElement(SystemPath+"/AccelZBias_mss", "Z accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  Gx = deftree.initElement(SystemPath+"/GyroX_rads", "X rotation rate with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  Gxb = deftree.initElement(SystemPath+"/GyroXBias_rads", "X rotation rate estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  Gy = deftree.initElement(SystemPath+"/GyroY_rads", "Y rotation rate with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  Gyb = deftree.initElement(SystemPath+"/GyroYBias_rads", "Y rotation rate estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  Gz = deftree.initElement(SystemPath+"/GyroZ_rads", "Z rotation rate with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  Gzb = deftree.initElement(SystemPath+"/GyroZBias_rads", "Z rotation rate estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  Pitch = deftree.initElement(SystemPath+"/Pitch_rad", "Pitch, rad", LOG_FLOAT, LOG_NONE);
  Roll = deftree.initElement(SystemPath+"/Roll_rad", "Roll, rad", LOG_FLOAT, LOG_NONE);
  Heading = deftree.initElement(SystemPath+"/Heading_rad", "Heading, rad", LOG_FLOAT, LOG_NONE);
  Lat = deftree.initElement(SystemPath+"/Latitude_rad", "Latitude, rad", LOG_DOUBLE, LOG_NONE);
  Lon = deftree.initElement(SystemPath+"/Longitude_rad", "Longitude, rad", LOG_DOUBLE, LOG_NONE);
  Alt = deftree.initElement(SystemPath+"/Altitude_m", "Altitude, m", LOG_FLOAT, LOG_NONE);
  Vn = deftree.initElement(SystemPath+"/NorthVelocity_ms", "North velocity, m/s", LOG_FLOAT, LOG_NONE);
  Ve = deftree.initElement(SystemPath+"/EastVelocity_ms", "East velocity, m/s", LOG_FLOAT, LOG_NONE);
  Vd = deftree.initElement(SystemPath+"/DownVelocity_ms", "Down velocity, m/s", LOG_FLOAT, LOG_NONE);
  Track = deftree.initElement(SystemPath+"/Track_rad", "Track, rad", LOG_FLOAT, LOG_NONE);
  Fix = deftree.initElement(SystemPath+"/Fix", "GPS fix", LOG_UINT8, LOG_NONE);

  uNavINS_.Configure();
}

void Ekf15StateIns::Initialize() {
  if ( GpsFix->getInt() && !Initialized_ ) {

    pGpsMeas_D_rrm(0) = GpsLat->getDouble();
    pGpsMeas_D_rrm(1) = GpsLon->getDouble();
    pGpsMeas_D_rrm(2) = GpsAlt->getDouble();

    vGpsMeas_L_mps(0) = GpsVn->getFloat();
    vGpsMeas_L_mps(1) = GpsVe->getFloat();
    vGpsMeas_L_mps(2) = GpsVd->getFloat();

    gyroMeas_B_rps(0) = ImuGx->getFloat();
    gyroMeas_B_rps(1) = ImuGy->getFloat();
    gyroMeas_B_rps(2) = ImuGz->getFloat();

    accelMeas_B_mps2(0) = ImuAx->getFloat();
    accelMeas_B_mps2(1) = ImuAy->getFloat();
    accelMeas_B_mps2(2) = ImuAz->getFloat();

    magMeas_B_uT(0) = ImuHx->getFloat();
    magMeas_B_uT(1) = ImuHy->getFloat();
    magMeas_B_uT(2) = ImuHz->getFloat();

    // Call uNav Initialize
    uNavINS_.Initialize(gyroMeas_B_rps, accelMeas_B_mps2, magMeas_B_uT, pGpsMeas_D_rrm, vGpsMeas_L_mps);

    if (uNavINS_.Initialized()) {
      Initialized_ = true;
      cout << "EKF INITIALIZED" << endl;
    }
  }
}

bool Ekf15StateIns::Initialized() {
  return Initialized_;
}

void Ekf15StateIns::Run(Mode mode) {
  Mode_node->setInt(mode);
  Fix->setInt(GpsFix->getInt());

  if (mode!=kStandby) {

    gyroMeas_B_rps(0) = ImuGx->getFloat();
    gyroMeas_B_rps(1) = ImuGy->getFloat();
    gyroMeas_B_rps(2) = ImuGz->getFloat();

    accelMeas_B_mps2(0) = ImuAx->getFloat();
    accelMeas_B_mps2(1) = ImuAy->getFloat();
    accelMeas_B_mps2(2) = ImuAz->getFloat();

    magMeas_B_uT(0) = ImuHx->getFloat();
    magMeas_B_uT(1) = ImuHy->getFloat();
    magMeas_B_uT(2) = ImuHz->getFloat();

    pGpsMeas_D_rrm(0) = GpsLat->getDouble();
    pGpsMeas_D_rrm(1) = GpsLon->getDouble();
    pGpsMeas_D_rrm(2) = GpsAlt->getDouble();

    vGpsMeas_L_mps(0) = GpsVn->getFloat();
    vGpsMeas_L_mps(1) = GpsVe->getFloat();
    vGpsMeas_L_mps(2) = GpsVd->getFloat();

    // Call uNav Update
    uNavINS_.Update(time_node->getLong(), GpsTow->getInt(), gyroMeas_B_rps, accelMeas_B_mps2, magMeas_B_uT, pGpsMeas_D_rrm, vGpsMeas_L_mps);

    // Accels
    Vector3f aEst_mps2 = uNavINS_.Get_AccelEst();
    Ax->setFloat(aEst_mps2(0));
    Ay->setFloat(aEst_mps2(1));
    Az->setFloat(aEst_mps2(2));

    Vector3f aBias_mps2 = uNavINS_.Get_AccelBias();
    Axb->setFloat(aBias_mps2(0));
    Ayb->setFloat(aBias_mps2(1));
    Azb->setFloat(aBias_mps2(2));

    // Gyros (Rotation Rate)
    Vector3f wEst_rps = uNavINS_.Get_RotRateEst();
    Gx->setFloat(wEst_rps(0));
    Gy->setFloat(wEst_rps(1));
    Gz->setFloat(wEst_rps(2));

    Vector3f wBias_rps = uNavINS_.Get_RotRateBias();
    Gxb->setFloat(wBias_rps(0));
    Gyb->setFloat(wBias_rps(1));
    Gzb->setFloat(wBias_rps(2));

    // Orientation
    Vector3f orientEst_rad = uNavINS_.Get_OrientEst();
    Roll->setFloat(orientEst_rad(0));
    Pitch->setFloat(orientEst_rad(1));
    Heading->setFloat(orientEst_rad(2));

    // Position
    Vector3d pEst_rrm = uNavINS_.Get_PosEst();
    Lat->setDouble(pEst_rrm(0));
    Lon->setDouble(pEst_rrm(1));
    Alt->setFloat(pEst_rrm(2));

    // Velocity
    Vector3f vEst_mps = uNavINS_.Get_VelEst();
    Vn->setFloat(vEst_mps(0));
    Ve->setFloat(vEst_mps(1));
    Vd->setFloat(vEst_mps(2));

    // Ground Track
    Track->setFloat(uNavINS_.Get_Track());
  }
}

void Ekf15StateIns::Clear() {
  Mode_node->setInt(kStandby);
  Ax->setFloat(0.0f);
  Ay->setFloat(0.0f);
  Az->setFloat(0.0f);
  Gx->setFloat(0.0f);
  Gy->setFloat(0.0f);
  Gz->setFloat(0.0f);
  Axb->setFloat(0.0f);
  Ayb->setFloat(0.0f);
  Azb->setFloat(0.0f);
  Gxb->setFloat(0.0f);
  Gyb->setFloat(0.0f);
  Gzb->setFloat(0.0f);
  Pitch->setFloat(0.0f);
  Roll->setFloat(0.0f);
  Heading->setFloat(0.0f);
  Lat->setDouble(0);
  Lon->setDouble(0);
  Alt->setFloat(0);
  Vn->setFloat(0);
  Ve->setFloat(0);
  Vd->setFloat(0);
  Track->setFloat(0.0f);
}
