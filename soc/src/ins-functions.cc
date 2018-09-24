/*
ins-functions.cc
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ins-functions.hxx"

void Ekf15StateIns::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
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
    if (DefinitionTreePtr->GetValuePtr<uint8_t*>(FixKey)) {
      config_.GpsFix = DefinitionTreePtr->GetValuePtr<uint8_t*>(FixKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS fix source ")+FixKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<uint32_t*>(TowKey)) {
      config_.GpsTow = DefinitionTreePtr->GetValuePtr<uint32_t*>(TowKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS TOW source ")+TowKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(VnKey)) {
      config_.GpsVn = DefinitionTreePtr->GetValuePtr<double*>(VnKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS north velocity source ")+VnKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(VeKey)) {
      config_.GpsVe = DefinitionTreePtr->GetValuePtr<double*>(VeKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS east velocity source ")+VeKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(VdKey)) {
      config_.GpsVd = DefinitionTreePtr->GetValuePtr<double*>(VdKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS down velocity source ")+VdKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(LatKey)) {
      config_.GpsLat = DefinitionTreePtr->GetValuePtr<double*>(LatKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS latitude source ")+LatKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(LonKey)) {
      config_.GpsLon = DefinitionTreePtr->GetValuePtr<double*>(LonKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS longitude source ")+LonKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<double*>(AltKey)) {
      config_.GpsAlt = DefinitionTreePtr->GetValuePtr<double*>(AltKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS altitude source ")+AltKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": GPS source not specified in configuration."));
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
    if (DefinitionTreePtr->GetValuePtr<float*>(GxKey)) {
      config_.ImuGx = DefinitionTreePtr->GetValuePtr<float*>(GxKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": X gyro source ")+GxKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(GyKey)) {
      config_.ImuGy = DefinitionTreePtr->GetValuePtr<float*>(GyKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Y gyro source ")+GyKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(GzKey)) {
      config_.ImuGz = DefinitionTreePtr->GetValuePtr<float*>(GzKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Z gyro source ")+GzKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(AxKey)) {
      config_.ImuAx = DefinitionTreePtr->GetValuePtr<float*>(AxKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": X accelerometer source ")+AxKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(AyKey)) {
      config_.ImuAy = DefinitionTreePtr->GetValuePtr<float*>(AyKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Y accelerometer source ")+AyKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(AzKey)) {
      config_.ImuAz = DefinitionTreePtr->GetValuePtr<float*>(AzKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Z accelerometer source ")+AzKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(HxKey)) {
      config_.ImuHx = DefinitionTreePtr->GetValuePtr<float*>(HxKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": X magnetometer source ")+HxKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(HyKey)) {
      config_.ImuHy = DefinitionTreePtr->GetValuePtr<float*>(HyKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Y magnetometer source ")+HyKey+std::string(" not found in global data."));
    }
    if (DefinitionTreePtr->GetValuePtr<float*>(HzKey)) {
      config_.ImuHz = DefinitionTreePtr->GetValuePtr<float*>(HzKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Z magnetometer source ")+HzKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": IMU source not specified in configuration."));
  }
  // get time source
  if (Config.HasMember("Time")) {
    std::string TimeKey = Config["Time"].GetString();
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(TimeKey)) {
      config_.t = DefinitionTreePtr->GetValuePtr<uint64_t*>(TimeKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(":Time source ")+TimeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time source not specified in configuration."));
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointers to log data
  AxKey_ = OutputName+"/AccelX_mss";
  DefinitionTreePtr->InitMember(AxKey_,&data_.Ax,"X accelerometer with bias removed, m/s/s",true,false);
  AxbKey_ = OutputName+"/AccelXBias_mss";
  DefinitionTreePtr->InitMember(AxbKey_,&data_.Axb,"X accelerometer estimated bias, m/s/s",true,false);
  AyKey_ = OutputName+"/AccelY_mss";
  DefinitionTreePtr->InitMember(AyKey_,&data_.Ay,"Y accelerometer with bias removed, m/s/s",true,false);
  AybKey_ = OutputName+"/AccelYBias_mss";
  DefinitionTreePtr->InitMember(AybKey_,&data_.Ayb,"Y accelerometer estimated bias, m/s/s",true,false);
  AzKey_ = OutputName+"/AccelZ_mss";
  DefinitionTreePtr->InitMember(AzKey_,&data_.Az,"Z accelerometer with bias removed, m/s/s",true,false);
  AzbKey_ = OutputName+"/AccelZBias_mss";
  DefinitionTreePtr->InitMember(AzbKey_,&data_.Azb,"Z accelerometer estimated bias, m/s/s",true,false);
  GxKey_ = OutputName+"/GyroX_rads";
  DefinitionTreePtr->InitMember(GxKey_,&data_.Gx,"X gyro with bias removed, rad/s",true,false);
  GxbKey_ = OutputName+"/GyroXBias_rads";
  DefinitionTreePtr->InitMember(GxbKey_,&data_.Gxb,"X gyro estimated bias, rad/s",true,false);
  GyKey_ = OutputName+"/GyroY_rads";
  DefinitionTreePtr->InitMember(GyKey_,&data_.Gy,"Y gyro with bias removed, rad/s",true,false);
  GybKey_ = OutputName+"/GyroYBias_rads";
  DefinitionTreePtr->InitMember(GybKey_,&data_.Gyb,"Y gyro estimated bias, rad/s",true,false);
  GzKey_ = OutputName+"/GyroZ_rads";
  DefinitionTreePtr->InitMember(GzKey_,&data_.Gz,"Z gyro with bias removed, rad/s",true,false);
  GzbKey_ = OutputName+"/GyroZBias_rads";
  DefinitionTreePtr->InitMember(GzbKey_,&data_.Gzb,"Z gyro estimated bias, rad/s",true,false);
  PitchKey_ = OutputName+"/Pitch_rad";
  DefinitionTreePtr->InitMember(PitchKey_,&data_.Pitch,"Pitch, rad",true,false);
  RollKey_ = OutputName+"/Roll_rad";
  DefinitionTreePtr->InitMember(RollKey_,&data_.Roll,"Roll, rad",true,false);
  YawKey_ = OutputName+"/Yaw_rad";
  DefinitionTreePtr->InitMember(YawKey_,&data_.Yaw,"Yaw, rad",true,false);
  HeadingKey_ = OutputName+"/Heading_rad";
  DefinitionTreePtr->InitMember(HeadingKey_,&data_.Heading,"Heading, rad",true,false);
  TrackKey_ = OutputName+"/Track_rad";
  DefinitionTreePtr->InitMember(TrackKey_,&data_.Track,"Track, rad",true,false);
  LatKey_ = OutputName+"/Latitude_rad";
  DefinitionTreePtr->InitMember(LatKey_,&data_.Lat,"Latitude, rad",true,false);
  LonKey_ = OutputName+"/Longitude_rad";
  DefinitionTreePtr->InitMember(LonKey_,&data_.Lon,"Longitude, rad",true,false);
  AltKey_ = OutputName+"/Altitude_m";
  DefinitionTreePtr->InitMember(AltKey_,&data_.Alt,"Altitude, m",true,false);
  VnKey_ = OutputName+"/NorthVelocity_ms";
  DefinitionTreePtr->InitMember(VnKey_,&data_.Vn,"North velocity, m/s",true,false);
  VeKey_ = OutputName+"/EastVelocity_ms";
  DefinitionTreePtr->InitMember(VeKey_,&data_.Ve,"East velocity, m/s",true,false);
  VdKey_ = OutputName+"/DownVelocity_ms";
  DefinitionTreePtr->InitMember(VdKey_,&data_.Vd,"Down velocity, m/s",true,false);
}

void Ekf15StateIns::Initialize() {
  if ((*config_.GpsFix)&&(!Initialized_)) {
    uNavINS_.update(*config_.t,*config_.GpsTow,*config_.GpsVn,*config_.GpsVe,*config_.GpsVd,*config_.GpsLat,*config_.GpsLon,*config_.GpsAlt,*config_.ImuGx,*config_.ImuGy,*config_.ImuGz,*config_.ImuAx,*config_.ImuAy,*config_.ImuAz,*config_.ImuHx,*config_.ImuHy,*config_.ImuHz);
    if (uNavINS_.initialized()) {
      Initialized_ = true;
    }
  }
}

bool Ekf15StateIns::Initialized() {
  return Initialized_;
}

void Ekf15StateIns::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    uNavINS_.update(*config_.t,*config_.GpsTow,*config_.GpsVn,*config_.GpsVe,*config_.GpsVd,*config_.GpsLat,*config_.GpsLon,*config_.GpsAlt,*config_.ImuGx,*config_.ImuGy,*config_.ImuGz,*config_.ImuAx,*config_.ImuAy,*config_.ImuAz,*config_.ImuHx,*config_.ImuHy,*config_.ImuHz);
    data_.Axb = uNavINS_.getAccelBiasX_mss();
    data_.Ayb = uNavINS_.getAccelBiasY_mss();
    data_.Azb = uNavINS_.getAccelBiasZ_mss();
    data_.Gxb = uNavINS_.getGyroBiasX_rads();
    data_.Gyb = uNavINS_.getGyroBiasY_rads();
    data_.Gzb = uNavINS_.getGyroBiasZ_rads();
    data_.Pitch = uNavINS_.getPitch_rad();
    data_.Roll = uNavINS_.getRoll_rad();
    data_.Yaw = uNavINS_.getYaw_rad();
    data_.Heading = uNavINS_.getHeading_rad();
    data_.Track = uNavINS_.getGroundTrack_rad();
    data_.Lat = uNavINS_.getLatitude_rad();
    data_.Lon = uNavINS_.getLongitude_rad();
    data_.Alt = uNavINS_.getAltitude_m();
    data_.Vn = uNavINS_.getVelNorth_ms();
    data_.Ve = uNavINS_.getVelEast_ms();
    data_.Vd = uNavINS_.getVelDown_ms();
    data_.Ax = *config_.ImuAx - data_.Axb;
    data_.Ay = *config_.ImuAy - data_.Ayb;
    data_.Az = *config_.ImuAz - data_.Azb;
    data_.Gx = *config_.ImuGx - data_.Gxb;
    data_.Gy = *config_.ImuGy - data_.Gyb;
    data_.Gz = *config_.ImuGz - data_.Gzb;
  }
}

void Ekf15StateIns::Clear(DefinitionTree *DefinitionTreePtr) {
  data_.Mode = kStandby;
  data_.Ax = 0.0f;
  data_.Ay = 0.0f;
  data_.Az = 0.0f;
  data_.Gx = 0.0f;
  data_.Gy = 0.0f;
  data_.Gz = 0.0f;
  data_.Axb = 0.0f;
  data_.Ayb = 0.0f;
  data_.Azb = 0.0f;
  data_.Gxb = 0.0f;
  data_.Gyb = 0.0f;
  data_.Gzb = 0.0f;
  data_.Pitch = 0.0f;
  data_.Roll = 0.0f;
  data_.Yaw = 0.0f;
  data_.Heading = 0.0f;
  data_.Track = 0.0f;
  data_.Lat = 0;
  data_.Lon = 0;
  data_.Alt = 0;
  data_.Vn = 0;
  data_.Ve = 0;
  data_.Vd = 0;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(AxKey_);
  DefinitionTreePtr->Erase(AyKey_);
  DefinitionTreePtr->Erase(AzKey_);
  DefinitionTreePtr->Erase(AxbKey_);
  DefinitionTreePtr->Erase(AybKey_);
  DefinitionTreePtr->Erase(AzbKey_);
  DefinitionTreePtr->Erase(GxKey_);
  DefinitionTreePtr->Erase(GyKey_);
  DefinitionTreePtr->Erase(GzKey_);
  DefinitionTreePtr->Erase(GxbKey_);
  DefinitionTreePtr->Erase(GybKey_);
  DefinitionTreePtr->Erase(GzbKey_);
  DefinitionTreePtr->Erase(PitchKey_);
  DefinitionTreePtr->Erase(RollKey_);
  DefinitionTreePtr->Erase(YawKey_);
  DefinitionTreePtr->Erase(HeadingKey_);
  DefinitionTreePtr->Erase(TrackKey_);
  DefinitionTreePtr->Erase(LatKey_);
  DefinitionTreePtr->Erase(LonKey_);
  DefinitionTreePtr->Erase(AltKey_);
  DefinitionTreePtr->Erase(VnKey_);
  DefinitionTreePtr->Erase(VeKey_);
  DefinitionTreePtr->Erase(VdKey_);
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
}
