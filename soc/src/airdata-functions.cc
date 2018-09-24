/*
airdata-functions.cc
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

#include "airdata-functions.hxx"

void IndicatedAirspeed::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  // get pressure sources
  if (Config.HasMember("Differential-Pressure")) {
    const rapidjson::Value& PressureSources = Config["Differential-Pressure"];
    for (auto &PressureSource : PressureSources.GetArray()) {
      DifferentialPressureKeys_.push_back(PressureSource.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKeys_.back())) {
        config_.DifferentialPressure.push_back(DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure source ")+DifferentialPressureKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure sources not specified in configuration."));
  }
  // resize bias vector
  data_.DifferentialPressureBias.resize(config_.DifferentialPressure.size());
  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Ias_ms,"Indicated airspeed, m/s",true,false);
}

void IndicatedAirspeed::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;
  // if less than init time, compute bias
  if (ElapsedTime < config_.InitTime) {
    for (size_t i=0; i < config_.DifferentialPressure.size(); i++) {
      data_.DifferentialPressureBias[i] = data_.DifferentialPressureBias[i] + (*config_.DifferentialPressure[i]-data_.DifferentialPressureBias[i])/((float)NumberSamples_);
    }
    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool IndicatedAirspeed::Initialized() {
  return Initialized_;
}

void IndicatedAirspeed::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute average differential pressure
    data_.AvgDifferentialPressure = 0.0f;
    for (size_t i=0; i < config_.DifferentialPressure.size(); i++) {
      data_.AvgDifferentialPressure += (*config_.DifferentialPressure[i]-data_.DifferentialPressureBias[i])/config_.DifferentialPressure.size();
    }
    // compute indicated airspeed
    data_.Ias_ms = AirData_.getIAS(data_.AvgDifferentialPressure);
  }
}

void IndicatedAirspeed::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.DifferentialPressure.clear();
  config_.InitTime = 0.0f;
  data_.DifferentialPressureBias.clear();
  data_.Mode = kStandby;
  data_.Ias_ms = 0.0f;
  data_.AvgDifferentialPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  DifferentialPressureKeys_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}

uint64_t IndicatedAirspeed::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void AglAltitude::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  // get pressure sources
  if (Config.HasMember("Static-Pressure")) {
    const rapidjson::Value& PressureSources = Config["Static-Pressure"];
    for (auto &PressureSource : PressureSources.GetArray()) {
      StaticPressureKeys_.push_back(PressureSource.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKeys_.back())) {
        config_.StaticPressure.push_back(DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure source ")+StaticPressureKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure sources not specified in configuration."));
  }
  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Agl_m,"Altitude above ground, m",true,false);
}

void AglAltitude::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;
  // if less than init time, compute initial pressure altitude
  if (ElapsedTime < config_.InitTime) {
    // average static pressure sources
    data_.AvgStaticPressure = 0.0f;
    for (size_t i=0; i < config_.StaticPressure.size(); i++) {
      data_.AvgStaticPressure += (*config_.StaticPressure[i])/config_.StaticPressure.size();
    }
    // compute pressure altitude
    data_.PressAlt0 = data_.PressAlt0 + (AirData_.getPressureAltitude(data_.AvgStaticPressure)-data_.PressAlt0)/((float)NumberSamples_);
    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool AglAltitude::Initialized() {
  return Initialized_;
}

void AglAltitude::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute average static pressure
    data_.AvgStaticPressure = 0.0f;
    for (size_t i=0; i < config_.StaticPressure.size(); i++) {
      data_.AvgStaticPressure += (*config_.StaticPressure[i])/config_.StaticPressure.size();
    }
    // compute altitude above ground level
    data_.Agl_m = AirData_.getAGL(data_.AvgStaticPressure,data_.PressAlt0);
  }
}

void AglAltitude::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.StaticPressure.clear();
  config_.InitTime = 0.0f;
  data_.Mode = kStandby;
  data_.Agl_m = 0.0f;
  data_.PressAlt0 = 0.0f;
  data_.AvgStaticPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  StaticPressureKeys_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}

uint64_t AglAltitude::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

/* Pitot-Static System */
void PitotStatic::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get PitotStatic name
  std::string SysName;
  if (Config.HasMember("Output")) {
    SysName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  std::string OutputIasName;
  if (Config.HasMember("OutputIas")) {
    OutputIasName = Config["OutputIas"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputIas not specified in configuration."));
  }

  // get output names
  std::string OutputAglName;
  if (Config.HasMember("OutputAltitude")) {
    OutputAglName = Config["OutputAltitude"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputAltitude not specified in configuration."));
  }


  // get differential pressure source
  if (Config.HasMember("Differential-Pressure")) {
    DifferentialPressureKey_ = Config["Differential-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKey_)) {
      config_.DifferentialPressure = DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Differential-Pressure ")+DifferentialPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Differential-Pressure not specified in configuration."));
  }

  // get static pressure source
  if (Config.HasMember("Static-Pressure")) {
    StaticPressureKey_ = Config["Static-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKey_)) {
      config_.StaticPressure = DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure ")+StaticPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure not specified in configuration."));
  }

  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Initialization time not specified in configuration."));
  }

  // pointer to log run mode data
  ModeKey_ = SysName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);

  // pointer to log ias data
  OutputIasKey_ = SysName+"/"+OutputIasName;
  DefinitionTreePtr->InitMember(OutputIasKey_,&data_.Ias_ms,"Indicated airspeed, m/s",true,false);
  OutputAglKey_ = SysName+"/"+OutputAglName;
  DefinitionTreePtr->InitMember(OutputAglKey_,&data_.Agl_m,"Altitude above ground, m",true,false);
}

void PitotStatic::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }

  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;

  // if less than init time, compute bias
  if (ElapsedTime < config_.InitTime) {
    // compute differential pressure bias, using Welford's algorithm
    data_.DifferentialPressureBias += (*config_.DifferentialPressure - data_.DifferentialPressureBias) / (float)NumberSamples_;

    // compute static pressure bias, using Welford's algorithm
    data_.PressAlt0 += (AirData_.getPressureAltitude(*config_.StaticPressure) - data_.PressAlt0) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool PitotStatic::Initialized() {
  return Initialized_;
}

void PitotStatic::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute indicated airspeed
    data_.Ias_ms = AirData_.getIAS(*config_.DifferentialPressure - data_.DifferentialPressureBias);

    // compute altitude above ground level
    data_.Agl_m = AirData_.getAGL(*config_.StaticPressure, data_.PressAlt0);
  }
}

void PitotStatic::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.InitTime = 0.0f;

  data_.Mode = kStandby;

  data_.DifferentialPressureBias = 0.0f;
  data_.Ias_ms = 0.0f;

  data_.Agl_m = 0.0f;
  data_.PressAlt0 = 0.0f;

  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputIasKey_);
  DefinitionTreePtr->Erase(OutputAglKey_);

  DifferentialPressureKey_.clear();
  StaticPressureKey_.clear();

  ModeKey_.clear();
  OutputIasKey_.clear();
  OutputAglKey_.clear();
}

uint64_t PitotStatic::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}


/* 5-Hole Probe - Method 1 */
// Assume that all 5 pressure sensors are measuring relative to the static ring
void FiveHole::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get FiveHole name
  std::string SysName;
  if (Config.HasMember("Output")) {
    SysName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  // get Airspeed output name
  std::string OutputIasName;
  if (Config.HasMember("OutputIas")) {
    OutputIasName = Config["OutputIas"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputIas not specified in configuration."));
  }

  // get Altitude output name
  std::string OutputAglName;
  if (Config.HasMember("OutputAltitude")) {
    OutputAglName = Config["OutputAltitude"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputAltitude not specified in configuration."));
  }

  // get Alpha output name
  std::string OutputAlphaName;
  if (Config.HasMember("OutputAlpha")) {
    OutputAlphaName = Config["OutputAlpha"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputAlpha not specified in configuration."));
  }

  // get Beta output name
  std::string OutputBetaName;
  if (Config.HasMember("OutputBeta")) {
    OutputBetaName = Config["OutputBeta"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": OutputBeta not specified in configuration."));
  }


  // get Tip pressure source
  if (Config.HasMember("Tip-Pressure")) {
    TipPressureKey_ = Config["Tip-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(TipPressureKey_)) {
      config_.TipPressure = DefinitionTreePtr->GetValuePtr<float*>(TipPressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Tip-Pressure ")+TipPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Tip-Pressure not specified in configuration."));
  }

  // get static pressure source
  if (Config.HasMember("Static-Pressure")) {
    StaticPressureKey_ = Config["Static-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKey_)) {
      config_.StaticPressure = DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure ")+StaticPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure not specified in configuration."));
  }

  // get Alpha1 pressure source
  if (Config.HasMember("Alpha1-Pressure")) {
    Alpha1PressureKey_ = Config["Alpha1-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(Alpha1PressureKey_)) {
      config_.Alpha1Pressure = DefinitionTreePtr->GetValuePtr<float*>(Alpha1PressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha1-Pressure ")+Alpha1PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha1-Pressure not specified in configuration."));
  }

  // get Alpha2 pressure source
  if (Config.HasMember("Alpha2-Pressure")) {
    Alpha2PressureKey_ = Config["Alpha2-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(Alpha2PressureKey_)) {
      config_.Alpha2Pressure = DefinitionTreePtr->GetValuePtr<float*>(Alpha2PressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha2-Pressure ")+Alpha2PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha2-Pressure not specified in configuration."));
  }

  // get Beta1 pressure source
  if (Config.HasMember("Beta1-Pressure")) {
    Beta1PressureKey_ = Config["Beta1-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(Beta1PressureKey_)) {
      config_.Beta1Pressure = DefinitionTreePtr->GetValuePtr<float*>(Beta1PressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta1-Pressure ")+Beta1PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta1-Pressure not specified in configuration."));
  }

  // get Beta2 pressure source
  if (Config.HasMember("Beta2-Pressure")) {
    Beta2PressureKey_ = Config["Beta2-Pressure"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(Beta2PressureKey_)) {
      config_.Beta2Pressure = DefinitionTreePtr->GetValuePtr<float*>(Beta2PressureKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta2-Pressure ")+Beta2PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta2-Pressure not specified in configuration."));
  }


  // get Alpha Calibration constant
  if (Config.HasMember("Alpha-Calibration")) {
    config_.kAlpha = Config["Alpha-Calibration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha-Calibration not specified in configuration."));
  }

  // get Beta Calibration constant
  if (Config.HasMember("Beta-Calibration")) {
    config_.kBeta = Config["Beta-Calibration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta-Calibration not specified in configuration."));
  }

  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Initialization time not specified in configuration."));
  }

  // pointer to log run mode data
  ModeKey_ = SysName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);

  // pointer to log ias data
  OutputAglKey_ = SysName+"/"+OutputAglName;
  DefinitionTreePtr->InitMember(OutputAglKey_,&data_.Agl_m,"Altitude above ground, m",true,false);
  OutputIasKey_ = SysName+"/"+OutputIasName;
  DefinitionTreePtr->InitMember(OutputIasKey_,&data_.Ias_ms,"Indicated airspeed, m/s",true,false);
  OutputAlphaKey_ = SysName+"/"+OutputAlphaName;
  DefinitionTreePtr->InitMember(OutputAlphaKey_,&data_.Alpha_rad,"Angle of attack, rad",true,false);
  OutputBetaKey_ = SysName+"/"+OutputBetaName;
  DefinitionTreePtr->InitMember(OutputBetaKey_,&data_.Beta_rad,"Sideslip angle, rad",true,false);
}

void FiveHole::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }

  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;

  // if less than init time, compute bias
  if (ElapsedTime < config_.InitTime) {
    // compute pressure biases, using Welford's algorithm
    data_.PressAlt0 += (AirData_.getPressureAltitude(*config_.StaticPressure) - data_.PressAlt0) / (float)NumberSamples_;
    data_.TipPressureBias += (*config_.TipPressure - data_.TipPressureBias) / (float)NumberSamples_;
    data_.Alpha1PressureBias += (*config_.Alpha1Pressure - data_.Alpha1PressureBias) / (float)NumberSamples_;
    data_.Alpha2PressureBias += (*config_.Alpha2Pressure - data_.Alpha2PressureBias) / (float)NumberSamples_;
    data_.Beta1PressureBias += (*config_.Beta1Pressure - data_.Beta1PressureBias) / (float)NumberSamples_;
    data_.Beta2PressureBias += (*config_.Beta2Pressure - data_.Beta2PressureBias) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool FiveHole::Initialized() {
  return Initialized_;
}

void FiveHole::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute indicated airspeed
    data_.Ias_ms = AirData_.getIAS(*config_.TipPressure - data_.TipPressureBias);

    // compute altitude above ground level
    data_.Agl_m = AirData_.getAGL(*config_.StaticPressure, data_.PressAlt0);

    // compute Alpha and Beta
    // if the airspeed is low, the non-dimensionalizing pressures on the side port will be very small and result in huge angles
    if (data_.Ias_ms > 2.0f) {
      // compute alpha
      data_.Alpha_rad = AirData_.getAngle(*config_.TipPressure - data_.TipPressureBias,
        *config_.Alpha1Pressure - data_.Alpha1PressureBias,
        *config_.Alpha2Pressure - data_.Alpha2PressureBias,
        *config_.Beta1Pressure - data_.Beta1PressureBias,
        *config_.Beta2Pressure - data_.Beta2PressureBias,
        config_.kAlpha);

      // limit alpha to +/-45 deg
      if (data_.Alpha_rad > 0.7854f) {
        data_.Alpha_rad = 0.7854f;
      } else if (data_.Alpha_rad < -0.7854f) {
        data_.Alpha_rad = -0.7854f;
      }

      // compute beta
      data_.Beta_rad = AirData_.getAngle(*config_.TipPressure - data_.TipPressureBias,
        *config_.Beta1Pressure - data_.Beta1PressureBias,
        *config_.Beta2Pressure - data_.Beta2PressureBias,
        *config_.Alpha1Pressure - data_.Alpha1PressureBias,
        *config_.Alpha2Pressure - data_.Alpha2PressureBias,
        config_.kBeta);

      // limit beta to +/-45 deg
      if (data_.Beta_rad > 0.7854f) {
        data_.Beta_rad = 0.7854f;
      } else if (data_.Beta_rad < -0.7854f) {
        data_.Beta_rad = -0.7854f;
      }

    } else { // airspeed < theshold
      data_.Alpha_rad = 0.0f;
      data_.Beta_rad = 0.0f;
    }
  }
}

void FiveHole::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.InitTime = 0.0f;

  data_.Mode = kStandby;

  data_.TipPressureBias = 0.0f;
  data_.Ias_ms = 0.0f;

  data_.Agl_m = 0.0f;
  data_.PressAlt0 = 0.0f;

  data_.Alpha1PressureBias = 0.0f;
  data_.Alpha2PressureBias = 0.0f;
  data_.Alpha_rad = 0.0f;

  data_.Beta1PressureBias = 0.0f;
  data_.Beta2PressureBias = 0.0f;
  data_.Beta_rad = 0.0f;

  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputIasKey_);
  DefinitionTreePtr->Erase(OutputAglKey_);
  DefinitionTreePtr->Erase(OutputAlphaKey_);
  DefinitionTreePtr->Erase(OutputBetaKey_);

  TipPressureKey_.clear();
  StaticPressureKey_.clear();
  Alpha1PressureKey_.clear();
  Alpha2PressureKey_.clear();
  Beta1PressureKey_.clear();
  Beta2PressureKey_.clear();

  ModeKey_.clear();
  OutputIasKey_.clear();
  OutputAglKey_.clear();
  OutputAlphaKey_.clear();
  OutputBetaKey_.clear();
}

uint64_t FiveHole::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}
