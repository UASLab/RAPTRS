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
    config_.GpsFix = deftree.getElement(FixKey);
    if ( !config_.GpsFix ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS fix source ")+FixKey+std::string(" not found in global data."));
    }
    config_.GpsTow = deftree.getElement(TowKey);
    if ( !config_.GpsTow ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS TOW source ")+TowKey+std::string(" not found in global data."));
    }
    config_.GpsVn = deftree.getElement(VnKey);
    if ( !config_.GpsVn ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS north velocity source ")+VnKey+std::string(" not found in global data."));
    }
    config_.GpsVe = deftree.getElement(VeKey);
    if ( !config_.GpsVe ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS east velocity source ")+VeKey+std::string(" not found in global data."));
    }
    config_.GpsVd = deftree.getElement(VdKey);
    if ( !config_.GpsVd ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS down velocity source ")+VdKey+std::string(" not found in global data."));
    }
    config_.GpsLat = deftree.getElement(LatKey);
    if ( ! config_.GpsLat ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS latitude source ")+LatKey+std::string(" not found in global data."));
    }
    config_.GpsLon = deftree.getElement(LonKey);
    if ( !config_.GpsLon ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": GPS longitude source ")+LonKey+std::string(" not found in global data."));
    }
    config_.GpsAlt = deftree.getElement(AltKey);
    if ( !config_.GpsAlt ) {
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
    config_.ImuGx = deftree.getElement(GxKey);
    if ( !config_.ImuGx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X gyro source ")+GxKey+std::string(" not found in global data."));
    }
    config_.ImuGy = deftree.getElement(GyKey);
    if ( !config_.ImuGy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y gyro source ")+GyKey+std::string(" not found in global data."));
    }
    config_.ImuGz = deftree.getElement(GzKey);
    if ( !config_.ImuGz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z gyro source ")+GzKey+std::string(" not found in global data."));
    }
    config_.ImuAx = deftree.getElement(AxKey);
    if ( !config_.ImuAx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X accelerometer source ")+AxKey+std::string(" not found in global data."));
    }
    config_.ImuAy = deftree.getElement(AyKey);
    if ( !config_.ImuAy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y accelerometer source ")+AyKey+std::string(" not found in global data."));
    }
    config_.ImuAz = deftree.getElement(AzKey);
    if ( !config_.ImuAz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z accelerometer source ")+AzKey+std::string(" not found in global data."));
    }
    config_.ImuHx = deftree.getElement(HxKey);
    if ( !config_.ImuHx ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": X magnetometer source ")+HxKey+std::string(" not found in global data."));
    }
    config_.ImuHy = deftree.getElement(HyKey);
    if ( !config_.ImuHy ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Y magnetometer source ")+HyKey+std::string(" not found in global data."));
    }
    config_.ImuHz = deftree.getElement(HzKey);
    if ( !config_.ImuHz ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Z magnetometer source ")+HzKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": IMU source not specified in configuration."));
  }
  // get time source
  if (Config.HasMember("Time")) {
    std::string TimeKey = Config["Time"].GetString();
    config_.t = deftree.getElement(TimeKey);
    if ( !config_.t ) {
      throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(":Time source ")+TimeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemPath+std::string(": Time source not specified in configuration."));
  }
  // get error characteristic config
  if (Config.HasMember("Config")) {
    if (Config["Config"].HasMember("sig-w-a")){
      uNavINS_.setSig_W_A(Config["Config"]["sig-w-a"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-w-g")){
      uNavINS_.setSig_W_G(Config["Config"]["sig-w-g"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-a-d")){
      uNavINS_.setSig_A_D(Config["Config"]["sig-a-d"].GetFloat());
    }
    if (Config["Config"].HasMember("tau-a")){
      uNavINS_.setTau_A(Config["Config"]["tau-a"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-g-d")){
      uNavINS_.setSig_G_D(Config["Config"]["sig-g-d"].GetFloat());
    }
    if (Config["Config"].HasMember("tau-g")){
      uNavINS_.setTau_G(Config["Config"]["tau-g"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-gps-p-ne")){
      uNavINS_.setSig_GPS_P_NE(Config["Config"]["sig-gps-p-ne"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-gps-p-d")){
      uNavINS_.setSig_GPS_P_D(Config["Config"]["sig-gps-p-d"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-gps-v-ne")){
      uNavINS_.setSig_GPS_V_NE(Config["Config"]["sig-gps-v-ne"].GetFloat());
    }
    if (Config["Config"].HasMember("sig-gps-v-d")){
      uNavINS_.setSig_GPS_V_D(Config["Config"]["sig-gps-v-d"].GetFloat());
    }
  }
  // pointer to log run mode data
  ModeKey_ = SystemPath+"/Mode";
  data_.Mode = deftree.initElement(ModeKey_,"Run mode", LOG_UINT8, LOG_NONE);
  data_.Mode->setInt(kStandby);

  // pointers to log data
  AxKey_ = SystemPath+"/AccelX_mss";
  data_.Ax = deftree.initElement(AxKey_, "X accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  AxbKey_ = SystemPath+"/AccelXBias_mss";
  data_.Axb = deftree.initElement(AxbKey_, "X accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  AyKey_ = SystemPath+"/AccelY_mss";
  data_.Ay = deftree.initElement(AyKey_, "Y accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  AybKey_ = SystemPath+"/AccelYBias_mss";
  data_.Ayb = deftree.initElement(AybKey_, "Y accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  AzKey_ = SystemPath+"/AccelZ_mss";
  data_.Az = deftree.initElement(AzKey_, "Z accelerometer with bias removed, m/s/s", LOG_FLOAT, LOG_NONE);
  AzbKey_ = SystemPath+"/AccelZBias_mss";
  data_.Azb = deftree.initElement(AzbKey_, "Z accelerometer estimated bias, m/s/s", LOG_FLOAT, LOG_NONE);
  GxKey_ = SystemPath+"/GyroX_rads";
  data_.Gx = deftree.initElement(GxKey_, "X gyro with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  GxbKey_ = SystemPath+"/GyroXBias_rads";
  data_.Gxb = deftree.initElement(GxbKey_, "X gyro estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  GyKey_ = SystemPath+"/GyroY_rads";
  data_.Gy = deftree.initElement(GyKey_, "Y gyro with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  GybKey_ = SystemPath+"/GyroYBias_rads";
  data_.Gyb = deftree.initElement(GybKey_, "Y gyro estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  GzKey_ = SystemPath+"/GyroZ_rads";
  data_.Gz = deftree.initElement(GzKey_, "Z gyro with bias removed, rad/s", LOG_FLOAT, LOG_NONE);
  GzbKey_ = SystemPath+"/GyroZBias_rads";
  data_.Gzb = deftree.initElement(GzbKey_, "Z gyro estimated bias, rad/s", LOG_FLOAT, LOG_NONE);
  PitchKey_ = SystemPath+"/Pitch_rad";
  data_.Pitch = deftree.initElement(PitchKey_, "Pitch, rad", LOG_FLOAT, LOG_NONE);
  RollKey_ = SystemPath+"/Roll_rad";
  data_.Roll = deftree.initElement(RollKey_, "Roll, rad", LOG_FLOAT, LOG_NONE);
  YawKey_ = SystemPath+"/Yaw_rad";
  data_.Yaw = deftree.initElement(YawKey_, "Yaw, rad", LOG_FLOAT, LOG_NONE);
  HeadingKey_ = SystemPath+"/Heading_rad";
  data_.Heading = deftree.initElement(HeadingKey_, "Heading, rad", LOG_FLOAT, LOG_NONE);
  TrackKey_ = SystemPath+"/Track_rad";
  data_.Track = deftree.initElement(TrackKey_, "Track, rad", LOG_FLOAT, LOG_NONE);
  LatKey_ = SystemPath+"/Latitude_rad";
  data_.Lat = deftree.initElement(LatKey_, "Latitude, rad", LOG_DOUBLE, LOG_NONE);
  LonKey_ = SystemPath+"/Longitude_rad";
  data_.Lon = deftree.initElement(LonKey_, "Longitude, rad", LOG_DOUBLE, LOG_NONE);
  AltKey_ = SystemPath+"/Altitude_m";
  data_.Alt = deftree.initElement(AltKey_, "Altitude, m", LOG_FLOAT, LOG_NONE);
  VnKey_ = SystemPath+"/NorthVelocity_ms";
  data_.Vn = deftree.initElement(VnKey_, "North velocity, m/s", LOG_FLOAT, LOG_NONE);
  VeKey_ = SystemPath+"/EastVelocity_ms";
  data_.Ve = deftree.initElement(VeKey_, "East velocity, m/s", LOG_FLOAT, LOG_NONE);
  VdKey_ = SystemPath+"/DownVelocity_ms";
  data_.Vd = deftree.initElement(VdKey_, "Down velocity, m/s", LOG_FLOAT, LOG_NONE);
  FixKey_ = SystemPath+"/Fix";
  data_.Fix = deftree.initElement(FixKey_, "GPS fix", LOG_UINT8, LOG_NONE);
}

void Ekf15StateIns::Initialize() {
  if ( config_.GpsFix->getInt() && !Initialized_ ) {
    uNavINS_.update(config_.t->getLong(),config_.GpsTow->getInt(),config_.GpsVn->getFloat(),config_.GpsVe->getFloat(),config_.GpsVd->getFloat(),config_.GpsLat->getDouble(),config_.GpsLon->getDouble(),config_.GpsAlt->getFloat(),config_.ImuGx->getFloat(),config_.ImuGy->getFloat(),config_.ImuGz->getFloat(),config_.ImuAx->getFloat(),config_.ImuAy->getFloat(),config_.ImuAz->getFloat(),config_.ImuHx->getFloat(),config_.ImuHy->getFloat(),config_.ImuHz->getFloat());
    if (uNavINS_.initialized()) {
      Initialized_ = true;
      cout << "EKF INITIALIZED" << endl;
    }
  }
}

bool Ekf15StateIns::Initialized() {
  return Initialized_;
}

void Ekf15StateIns::Run(Mode mode) {
  data_.Mode->setInt(mode);
  data_.Fix->setInt(config_.GpsFix->getInt());
  if (mode!=kStandby) {
    uNavINS_.update(config_.t->getLong(),config_.GpsTow->getInt(),config_.GpsVn->getFloat(),config_.GpsVe->getFloat(),config_.GpsVd->getFloat(),config_.GpsLat->getDouble(),config_.GpsLon->getDouble(),config_.GpsAlt->getFloat(),config_.ImuGx->getFloat(),config_.ImuGy->getFloat(),config_.ImuGz->getFloat(),config_.ImuAx->getFloat(),config_.ImuAy->getFloat(),config_.ImuAz->getFloat(),config_.ImuHx->getFloat(),config_.ImuHy->getFloat(),config_.ImuHz->getFloat());
    data_.Axb->setFloat(uNavINS_.getAccelBiasX_mss());
    data_.Ayb->setFloat(uNavINS_.getAccelBiasY_mss());
    data_.Azb->setFloat(uNavINS_.getAccelBiasZ_mss());
    data_.Gxb->setFloat(uNavINS_.getGyroBiasX_rads());
    data_.Gyb->setFloat(uNavINS_.getGyroBiasY_rads());
    data_.Gzb->setFloat(uNavINS_.getGyroBiasZ_rads());
    data_.Pitch->setFloat(uNavINS_.getPitch_rad());
    data_.Roll->setFloat(uNavINS_.getRoll_rad());
    data_.Yaw->setFloat(uNavINS_.getYaw_rad());
    data_.Heading->setFloat(uNavINS_.getHeading_rad());
    data_.Track->setFloat(uNavINS_.getGroundTrack_rad());
    data_.Lat->setDouble(uNavINS_.getLatitude_rad());
    data_.Lon->setDouble(uNavINS_.getLongitude_rad());
    data_.Alt->setFloat(uNavINS_.getAltitude_m());
    data_.Vn->setFloat(uNavINS_.getVelNorth_ms());
    data_.Ve->setFloat(uNavINS_.getVelEast_ms());
    data_.Vd->setFloat(uNavINS_.getVelDown_ms());
    data_.Ax->setFloat(config_.ImuAx->getFloat() - data_.Axb->getFloat());
    data_.Ay->setFloat(config_.ImuAy->getFloat() - data_.Ayb->getFloat());
    data_.Az->setFloat(config_.ImuAz->getFloat() - data_.Azb->getFloat());
    data_.Gx->setFloat(config_.ImuGx->getFloat() - data_.Gxb->getFloat());
    data_.Gy->setFloat(config_.ImuGy->getFloat() - data_.Gyb->getFloat());
    data_.Gz->setFloat(config_.ImuGz->getFloat() - data_.Gzb->getFloat());
  }
}

void Ekf15StateIns::Clear() {
  data_.Mode->setInt(kStandby);
  data_.Ax->setFloat(0.0f);
  data_.Ay->setFloat(0.0f);
  data_.Az->setFloat(0.0f);
  data_.Gx->setFloat(0.0f);
  data_.Gy->setFloat(0.0f);
  data_.Gz->setFloat(0.0f);
  data_.Axb->setFloat(0.0f);
  data_.Ayb->setFloat(0.0f);
  data_.Azb->setFloat(0.0f);
  data_.Gxb->setFloat(0.0f);
  data_.Gyb->setFloat(0.0f);
  data_.Gzb->setFloat(0.0f);
  data_.Pitch->setFloat(0.0f);
  data_.Roll->setFloat(0.0f);
  data_.Yaw->setFloat(0.0f);
  data_.Heading->setFloat(0.0f);
  data_.Track->setFloat(0.0f);
  data_.Lat->setDouble(0);
  data_.Lon->setDouble(0);
  data_.Alt->setFloat(0);
  data_.Vn->setFloat(0);
  data_.Ve->setFloat(0);
  data_.Vd->setFloat(0);
  deftree.Erase(ModeKey_);
  deftree.Erase(AxKey_);
  deftree.Erase(AyKey_);
  deftree.Erase(AzKey_);
  deftree.Erase(AxbKey_);
  deftree.Erase(AybKey_);
  deftree.Erase(AzbKey_);
  deftree.Erase(GxKey_);
  deftree.Erase(GyKey_);
  deftree.Erase(GzKey_);
  deftree.Erase(GxbKey_);
  deftree.Erase(GybKey_);
  deftree.Erase(GzbKey_);
  deftree.Erase(PitchKey_);
  deftree.Erase(RollKey_);
  deftree.Erase(YawKey_);
  deftree.Erase(HeadingKey_);
  deftree.Erase(TrackKey_);
  deftree.Erase(LatKey_);
  deftree.Erase(LonKey_);
  deftree.Erase(AltKey_);
  deftree.Erase(VnKey_);
  deftree.Erase(VeKey_);
  deftree.Erase(VdKey_);
  deftree.Erase(FixKey_);
  ModeKey_.clear();
  AxKey_.clear();
  AyKey_.clear();
  AzKey_.clear();
  AxbKey_.clear();
  AybKey_.clear();
  AzbKey_.clear();
  GxKey_.clear();
  GyKey_.clear();
  GzKey_.clear();
  GxbKey_.clear();
  GybKey_.clear();
  GzbKey_.clear();
  PitchKey_.clear();
  RollKey_.clear();
  YawKey_.clear();
  HeadingKey_.clear();
  TrackKey_.clear();
  LatKey_.clear();
  LonKey_.clear();
  AltKey_.clear();
  VnKey_.clear();
  VeKey_.clear();
  VdKey_.clear();
  FixKey_.clear();
}
