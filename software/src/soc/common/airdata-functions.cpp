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

#include "airdata-functions.h"

void IndicatedAirspeed::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
      ElementPtr ele = deftree.getElement(DifferentialPressureKeys_.back());
      if (ele) {
        config_.DifferentialPressure.push_back(ele);
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
  data_.mode_node = deftree.initElement(ModeKey_, "Run mode", LOG_UINT8, LOG_NONE);
  data_.mode_node->setInt(kStandby);
  
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  data_.ias_ms_node = deftree.initElement(OutputKey_, "Indicated airspeed, m/s", LOG_FLOAT, LOG_NONE);
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
      data_.DifferentialPressureBias[i] = data_.DifferentialPressureBias[i] + (config_.DifferentialPressure[i]->getFloat()-data_.DifferentialPressureBias[i])/((float)NumberSamples_);
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
  data_.mode_node->setInt(mode);
  if (mode!=kStandby) {
    // compute average differential pressure
    data_.AvgDifferentialPressure = 0.0f;
    for (size_t i=0; i < config_.DifferentialPressure.size(); i++) {
      data_.AvgDifferentialPressure += (config_.DifferentialPressure[i]->getFloat()-data_.DifferentialPressureBias[i])/config_.DifferentialPressure.size();
    }
    // compute indicated airspeed
    data_.ias_ms_node->setFloat( AirData_.getIAS(data_.AvgDifferentialPressure) );
  }
}

void IndicatedAirspeed::Clear() {
  config_.DifferentialPressure.clear();
  config_.InitTime = 0.0f;
  data_.DifferentialPressureBias.clear();
  data_.mode_node->setFloat(kStandby);
  data_.ias_ms_node->setFloat(0.0f);
  data_.AvgDifferentialPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  deftree.Erase(ModeKey_);
  deftree.Erase(OutputKey_);
  DifferentialPressureKeys_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}

uint64_t IndicatedAirspeed::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void AglAltitude::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
      ElementPtr ele = deftree.getElement(StaticPressureKeys_.back());
      if ( ele ) {
        config_.StaticPressure.push_back(ele);
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
  data_.mode_node = deftree.initElement(ModeKey_, "Run mode", LOG_UINT8, LOG_NONE);
  data_.mode_node->setInt(kStandby);
  
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  data_.agl_m_node = deftree.initElement(OutputKey_, "Altitude above ground, m", LOG_FLOAT, LOG_NONE);
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
      data_.AvgStaticPressure += (config_.StaticPressure[i]->getFloat())/config_.StaticPressure.size();
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
  data_.mode_node->setInt(mode);
  if (mode!=kStandby) {
    // compute average static pressure
    data_.AvgStaticPressure = 0.0f;
    for (size_t i=0; i < config_.StaticPressure.size(); i++) {
      data_.AvgStaticPressure += (config_.StaticPressure[i]->getFloat())/config_.StaticPressure.size();
    }
    // compute altitude above ground level
    data_.agl_m_node->setFloat( AirData_.getAGL(data_.AvgStaticPressure,data_.PressAlt0) );
  }
}

void AglAltitude::Clear() {
  config_.StaticPressure.clear();
  config_.InitTime = 0.0f;
  data_.mode_node->setInt( kStandby );
  data_.agl_m_node->setFloat( 0.0f );
  data_.PressAlt0 = 0.0f;
  data_.AvgStaticPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  deftree.Erase(ModeKey_);
  deftree.Erase(OutputKey_);
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
void PitotStatic::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
    config_.diff_press_node = deftree.getElement(DifferentialPressureKey_);
    if ( !config_.diff_press_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Differential-Pressure ")+DifferentialPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Differential-Pressure not specified in configuration."));
  }

  // get static pressure source
  if (Config.HasMember("Static-Pressure")) {
    StaticPressureKey_ = Config["Static-Pressure"].GetString();
    config_.static_press_node = deftree.getElement(StaticPressureKey_);
    if ( ! config_.static_press_node ) {
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
  data_.mode_node = deftree.initElement(ModeKey_, "Run mode", LOG_UINT8, LOG_NONE);
  data_.mode_node->setInt(kStandby);

  // pointer to log ias data
  OutputIasKey_ = SysName+"/"+OutputIasName;
  data_.ias_ms_node = deftree.initElement(OutputIasKey_, "Indicated airspeed, m/s", LOG_FLOAT, LOG_NONE);
  OutputAglKey_ = SysName+"/"+OutputAglName;
  data_.agl_m_node = deftree.initElement(OutputAglKey_, "Altitude above ground, m", LOG_FLOAT, LOG_NONE);
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
    data_.DifferentialPressureBias += (config_.diff_press_node->getFloat() - data_.DifferentialPressureBias) / (float)NumberSamples_;

    // compute static pressure bias, using Welford's algorithm
    data_.PressAlt0 += (AirData_.getPressureAltitude(config_.static_press_node->getFloat()) - data_.PressAlt0) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool PitotStatic::Initialized() {
  return Initialized_;
}

void PitotStatic::Run(Mode mode) {
  data_.mode_node->setInt(mode);
  if (mode!=kStandby) {
    // compute indicated airspeed
    data_.ias_ms_node->setFloat( AirData_.getIAS(config_.diff_press_node->getFloat() - data_.DifferentialPressureBias) );

    // compute altitude above ground level
    data_.agl_m_node->setFloat( AirData_.getAGL(config_.static_press_node->getFloat(), data_.PressAlt0) );
  }
}

void PitotStatic::Clear() {
  config_.InitTime = 0.0f;

  data_.mode_node->setInt( kStandby );

  data_.DifferentialPressureBias = 0.0f;
  data_.ias_ms_node->setFloat(0.0f);

  data_.agl_m_node->setFloat(0.0f);
  data_.PressAlt0 = 0.0f;

  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  deftree.Erase(ModeKey_);
  deftree.Erase(OutputIasKey_);
  deftree.Erase(OutputAglKey_);

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
void FiveHole::Configure(const rapidjson::Value& Config,std::string RootPath) {
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
    config_.TipPressure_node = deftree.getElement(TipPressureKey_);
    if ( !config_.TipPressure_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Tip-Pressure ")+TipPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Tip-Pressure not specified in configuration."));
  }

  // get static pressure source
  if (Config.HasMember("Static-Pressure")) {
    StaticPressureKey_ = Config["Static-Pressure"].GetString();
    config_.StaticPressure_node =  deftree.getElement(StaticPressureKey_);
    if ( !config_.StaticPressure_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure ")+StaticPressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Static-Pressure not specified in configuration."));
  }

  // get Alpha1 pressure source
  if (Config.HasMember("Alpha1-Pressure")) {
    Alpha1PressureKey_ = Config["Alpha1-Pressure"].GetString();
    config_.Alpha1Pressure_node = deftree.getElement(Alpha1PressureKey_);
    if ( !config_.Alpha1Pressure_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha1-Pressure ")+Alpha1PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha1-Pressure not specified in configuration."));
  }

  // get Alpha2 pressure source
  if (Config.HasMember("Alpha2-Pressure")) {
    Alpha2PressureKey_ = Config["Alpha2-Pressure"].GetString();
    config_.Alpha2Pressure_node = deftree.getElement(Alpha2PressureKey_);
    if ( !config_.Alpha2Pressure_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha2-Pressure ")+Alpha2PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Alpha2-Pressure not specified in configuration."));
  }

  // get Beta1 pressure source
  if (Config.HasMember("Beta1-Pressure")) {
    Beta1PressureKey_ = Config["Beta1-Pressure"].GetString();
    config_.Beta1Pressure_node = deftree.getElement(Beta1PressureKey_);
    if ( !config_.Beta1Pressure_node ) {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta1-Pressure ")+Beta1PressureKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Beta1-Pressure not specified in configuration."));
  }

  // get Beta2 pressure source
  if (Config.HasMember("Beta2-Pressure")) {
    Beta2PressureKey_ = Config["Beta2-Pressure"].GetString();
    config_.Beta2Pressure_node = deftree.getElement(Beta2PressureKey_);
    if ( !config_.Beta2Pressure_node ) {
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
  data_.mode_node = deftree.initElement(ModeKey_, "Run mode", LOG_UINT8, LOG_NONE);
  data_.mode_node->setInt(kStandby);
  
  // pointer to log ias data
  OutputAglKey_ = SysName+"/"+OutputAglName;
  data_.agl_m_node = deftree.initElement(OutputAglKey_,"Altitude above ground, m",LOG_FLOAT, LOG_NONE);
  OutputIasKey_ = SysName+"/"+OutputIasName;
  data_.ias_ms_node = deftree.initElement(OutputIasKey_,"Indicated airspeed, m/s",LOG_FLOAT, LOG_NONE);
  OutputAlphaKey_ = SysName+"/"+OutputAlphaName;
  data_.Alpha_rad_node = deftree.initElement(OutputAlphaKey_,"Angle of attack, rad",LOG_FLOAT, LOG_NONE);
  OutputBetaKey_ = SysName+"/"+OutputBetaName;
  data_.Beta_rad_node = deftree.initElement(OutputBetaKey_,"Sideslip angle, rad",LOG_FLOAT, LOG_NONE);
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
    data_.PressAlt0 += (AirData_.getPressureAltitude(config_.StaticPressure_node->getFloat()) - data_.PressAlt0) / (float)NumberSamples_;
    data_.TipPressureBias += (config_.TipPressure_node->getFloat() - data_.TipPressureBias) / (float)NumberSamples_;
    data_.Alpha1PressureBias += (config_.Alpha1Pressure_node->getFloat() - data_.Alpha1PressureBias) / (float)NumberSamples_;
    data_.Alpha2PressureBias += (config_.Alpha2Pressure_node->getFloat() - data_.Alpha2PressureBias) / (float)NumberSamples_;
    data_.Beta1PressureBias += (config_.Beta1Pressure_node->getFloat() - data_.Beta1PressureBias) / (float)NumberSamples_;
    data_.Beta2PressureBias += (config_.Beta2Pressure_node->getFloat() - data_.Beta2PressureBias) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool FiveHole::Initialized() {
  return Initialized_;
}

void FiveHole::Run(Mode mode) {
  data_.mode_node->setInt(mode);
  if (mode!=kStandby) {
    // compute indicated airspeed
    data_.ias_ms_node->setFloat( AirData_.getIAS(config_.TipPressure_node->getFloat() - data_.TipPressureBias) );

    // compute altitude above ground level
    data_.agl_m_node->setFloat( AirData_.getAGL(config_.StaticPressure_node->getFloat(), data_.PressAlt0) );

    // compute Alpha and Beta
    // if the airspeed is low, the non-dimensionalizing pressures on the side port will be very small and result in huge angles
    if (data_.ias_ms_node->getFloat() > 2.0f) {
      // compute alpha
      data_.Alpha_rad_node->setFloat( AirData_.getAngle(config_.TipPressure_node->getFloat() - data_.TipPressureBias, config_.Alpha1Pressure_node->getFloat() - data_.Alpha1PressureBias, config_.Alpha2Pressure_node->getFloat() - data_.Alpha2PressureBias, config_.Beta1Pressure_node->getFloat() - data_.Beta1PressureBias, config_.Beta2Pressure_node->getFloat() - data_.Beta2PressureBias, config_.kAlpha) );

      // limit alpha to +/-45 deg
      if (data_.Alpha_rad_node->getFloat() > 0.7854f) {
        data_.Alpha_rad_node->setFloat( 0.7854f );
      } else if (data_.Alpha_rad_node->getFloat() < -0.7854f) {
        data_.Alpha_rad_node->setFloat( -0.7854f );
      }

      // compute beta
      data_.Beta_rad_node->setFloat( AirData_.getAngle(config_.TipPressure_node->getFloat() - data_.TipPressureBias, config_.Beta1Pressure_node->getFloat() - data_.Beta1PressureBias, config_.Beta2Pressure_node->getFloat() - data_.Beta2PressureBias, config_.Alpha1Pressure_node->getFloat() - data_.Alpha1PressureBias, config_.Alpha2Pressure_node->getFloat() - data_.Alpha2PressureBias, config_.kBeta) );

      // limit beta to +/-45 deg
      if (data_.Beta_rad_node->getFloat() > 0.7854f) {
        data_.Beta_rad_node->setFloat( 0.7854f );
      } else if (data_.Beta_rad_node->getFloat() < -0.7854f) {
        data_.Beta_rad_node->setFloat( -0.7854f );
      }

    } else { // airspeed < theshold
      data_.Alpha_rad_node->setFloat( 0.0f );
      data_.Beta_rad_node->setFloat( 0.0f );
    }
  }
}

void FiveHole::Clear() {
  config_.InitTime = 0.0f;

  data_.mode_node->setInt(kStandby);

  data_.TipPressureBias = 0.0f;
  data_.ias_ms_node->setFloat( 0.0f );

  data_.agl_m_node->setFloat( 0.0f );
  data_.PressAlt0 = 0.0f;

  data_.Alpha1PressureBias = 0.0f;
  data_.Alpha2PressureBias = 0.0f;
  data_.Alpha_rad_node->setFloat( 0.0f );

  data_.Beta1PressureBias = 0.0f;
  data_.Beta2PressureBias = 0.0f;
  data_.Beta_rad_node->setFloat( 0.0f );

  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  deftree.Erase(ModeKey_);
  deftree.Erase(OutputIasKey_);
  deftree.Erase(OutputAglKey_);
  deftree.Erase(OutputAlphaKey_);
  deftree.Erase(OutputBetaKey_);

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
