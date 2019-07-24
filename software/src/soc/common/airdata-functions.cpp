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

#include "millis.h"
#include "airdata-functions.h"

void IndicatedAirspeed::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Differential-Pressure", &DiffPress_node_, &DiffPressKeys_);
  LoadOutput(Config, SystemName, "Output", &IAS_node_);

  // Resize bias vector
  DiffPressBias_.resize(DiffPressKeys_.size());

  // Initialization time
  TimeInit_s_ = 0.0f;
  LoadVal(Config, "Initialization-Time", &TimeInit_s_);
}

void IndicatedAirspeed::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float TimeElapsed_s = ((float)(micros() - T0_us_)) / 1e6;
  // if less than init time, compute bias
  if (TimeElapsed_s < TimeInit_s_) {
    for (size_t i=0; i < DiffPress_node_.size(); i++) {
      DiffPressBias_[i] = DiffPressBias_[i] + (DiffPress_node_[i]->getFloat() - DiffPressBias_[i]) / ((float)NumberSamples_);
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

  if (mode!=kStandby) {
    // compute average differential pressure
    DiffPressMean_ = 0.0f;
    for (size_t i=0; i < DiffPress_node_.size(); i++) {
      DiffPressMean_ += (DiffPress_node_[i]->getFloat() - DiffPressBias_[i]) / DiffPress_node_.size();
    }
    // compute indicated airspeed
    IAS_node_->setFloat( AirData_.getIAS(DiffPressMean_) );
  }
}

void IndicatedAirspeed::Clear() {
  DiffPress_node_.clear();
  TimeInit_s_ = 0.0f;
  DiffPressBias_.clear();
  IAS_node_->setFloat(0.0f);
  DiffPressMean_ = 0.0f;
  TimeLatch_ = false;
  Initialized_ = false;
  T0_us_ = 0;
  NumberSamples_ = 1;
  DiffPressKeys_.clear();
}

void AglAltitude::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Static-Pressure", &StaticPress_node_, &StaticPressKeys_);
  LoadOutput(Config, SystemName, "Output", &AltAgl_node_);

  // Initialization time
  LoadVal(Config, "Initialization-Time", &TimeInit_s_);
}

void AglAltitude::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float TimeElapsed_s = ((float)(micros() - T0_us_)) / 1e6;
  // if less than init time, compute initial pressure altitude
  if (TimeElapsed_s < TimeInit_s_) {
    // average static pressure sources
    StaticPressMean_ = 0.0f;
    for (size_t i=0; i < StaticPress_node_.size(); i++) {
      StaticPressMean_ += (StaticPress_node_[i]->getFloat()) / StaticPress_node_.size();
    }
    // compute pressure altitude
    PressAltBias_ = PressAltBias_ + (AirData_.getPressureAltitude(StaticPressMean_) - PressAltBias_) / ((float)NumberSamples_);
    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool AglAltitude::Initialized() {
  return Initialized_;
}

void AglAltitude::Run(Mode mode) {

  if (mode!=kStandby) {
    // compute average static pressure
    StaticPressMean_ = 0.0f;
    for (size_t i=0; i < StaticPress_node_.size(); i++) {
      StaticPressMean_ += (StaticPress_node_[i]->getFloat()) / StaticPress_node_.size();
    }
    // compute altitude above ground level
    AltAgl_node_->setFloat( AirData_.getAGL(StaticPressMean_, PressAltBias_) );
  }
}

void AglAltitude::Clear() {
  StaticPress_node_.clear();
  TimeInit_s_ = 0.0f;
  AltAgl_node_->setFloat( 0.0f );
  PressAltBias_ = 0.0f;
  StaticPressMean_ = 0.0f;
  TimeLatch_ = false;
  Initialized_ = false;
  T0_us_ = 0;
  NumberSamples_ = 1;
  StaticPressKeys_.clear();
}

/* Pitot-Static System */
void PitotStatic::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Static-Pressure", &StaticPress_node_, &StaticPressKey_);
  LoadInput(Config, SystemName, "Differential-Pressure", &DiffPress_node_, &DiffPressKey_);

  LoadOutput(Config, SystemName, "OutputAltitude", &AltAgl_node_);
  LoadOutput(Config, SystemName, "OutputIas", &IAS_node_);

  // Initialization time
  LoadVal(Config, "Initialization-Time", &TimeInit_s_);
}

void PitotStatic::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }

  // compute the elapsed time
  float TimeElapsed_s = ((float)(micros() - T0_us_)) / 1e6;

  // if less than init time, compute bias
  if (TimeElapsed_s < TimeInit_s_) {
    // compute differential pressure bias, using Welford's algorithm
    DiffPressBias_ += (DiffPress_node_->getFloat() - DiffPressBias_) / (float)NumberSamples_;

    // compute static pressure bias, using Welford's algorithm
    PressAltBias_ += (AirData_.getPressureAltitude(StaticPress_node_->getFloat()) - PressAltBias_) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool PitotStatic::Initialized() {
  return Initialized_;
}

void PitotStatic::Run(Mode mode) {

  if (mode!=kStandby) {
    // compute indicated airspeed
    IAS_node_->setFloat( AirData_.getIAS(DiffPress_node_->getFloat() - DiffPressBias_) );

    // compute altitude above ground level
    AltAgl_node_->setFloat( AirData_.getAGL(StaticPress_node_->getFloat(), PressAltBias_) );
  }
}

void PitotStatic::Clear() {
  TimeInit_s_ = 0.0f;

  DiffPressBias_ = 0.0f;
  IAS_node_->setFloat(0.0f);

  AltAgl_node_->setFloat(0.0f);
  PressAltBias_ = 0.0f;

  TimeLatch_ = false;
  Initialized_ = false;
  T0_us_ = 0;
  NumberSamples_ = 1;

  DiffPressKey_.clear();
  StaticPressKey_.clear();
}

/* 5-Hole Probe - Method 1 */
// Assume that all 5 pressure sensors are measuring relative to the static ring
void FiveHole1::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Static-Pressure", &StaticPress_node_, &StaticPressKey_);
  LoadInput(Config, SystemName, "Tip-Pressure", &TipPress_node_, &TipPressKey_);
  LoadInput(Config, SystemName, "Alpha1-Pressure", &Alpha1Press_node_, &Alpha1PressKey_);
  LoadInput(Config, SystemName, "Alpha2-Pressure", &Alpha2Press_node_, &Alpha2PressKey_);
  LoadInput(Config, SystemName, "Beta1-Pressure", &Beta1Press_node_, &Beta1PressKey_);
  LoadInput(Config, SystemName, "Beta2-Pressure", &Beta2Press_node_, &Beta2PressKey_);

  LoadOutput(Config, SystemName, "OutputAltitude", &AltAgl_node_);
  LoadOutput(Config, SystemName, "OutputIas", &IAS_node_);
  LoadOutput(Config, SystemName, "OutputAlpha", &Alpha_rad_node_);
  LoadOutput(Config, SystemName, "OutputBeta", &Beta_rad_node_);

  // Alpha and Beta Calibration constants
  LoadVal(Config, "Alpha-Calibration", &kAlpha_);
  LoadVal(Config, "Beta-Calibration", &kBeta_);

  // Initialization time
  LoadVal(Config, "Initialization-Time", &TimeInit_s_);
}

void FiveHole1::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }

  // compute the elapsed time
  float TimeElapsed_s = ((float)(micros() - T0_us_)) / 1e6;

  // if less than init time, compute bias
  if (TimeElapsed_s < TimeInit_s_) {
    // compute pressure biases, using Welford's algorithm
    PressAltBias_ += (AirData_.getPressureAltitude(StaticPress_node_->getFloat()) - PressAltBias_) / (float)NumberSamples_;
    TipPressBias_ += (TipPress_node_->getFloat() - TipPressBias_) / (float)NumberSamples_;
    Alpha1PressBias_ += (Alpha1Press_node_->getFloat() - Alpha1PressBias_) / (float)NumberSamples_;
    Alpha2PressBias_ += (Alpha2Press_node_->getFloat() - Alpha2PressBias_) / (float)NumberSamples_;
    Beta1PressBias_ += (Beta1Press_node_->getFloat() - Beta1PressBias_) / (float)NumberSamples_;
    Beta2PressBias_ += (Beta2Press_node_->getFloat() - Beta2PressBias_) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool FiveHole1::Initialized() {
  return Initialized_;
}

void FiveHole1::Run(Mode mode) {

  if (mode!=kStandby) {
    // compute indicated airspeed
    IAS_node_->setFloat( AirData_.getIAS(TipPress_node_->getFloat() - TipPressBias_) );

    // compute altitude above ground level
    AltAgl_node_->setFloat( AirData_.getAGL(StaticPress_node_->getFloat(), PressAltBias_) );

    // compute Alpha and Beta
    // if the airspeed is low, the non-dimensionalizing pressures on the side port will be very small and result in huge angles
    if (IAS_node_->getFloat() > 2.0f) {
      // compute alpha
      Alpha_rad_node_->setFloat( AirData_.getAngle1(TipPress_node_->getFloat() - TipPressBias_, Alpha1Press_node_->getFloat() - Alpha1PressBias_, Alpha2Press_node_->getFloat() - Alpha2PressBias_, Beta1Press_node_->getFloat() - Beta1PressBias_, Beta2Press_node_->getFloat() - Beta2PressBias_, kAlpha_) );

      // limit alpha to +/-45 deg
      if (Alpha_rad_node_->getFloat() > 0.7854f) {
        Alpha_rad_node_->setFloat( 0.7854f );
      } else if (Alpha_rad_node_->getFloat() < -0.7854f) {
        Alpha_rad_node_->setFloat( -0.7854f );
      }

      // compute beta
      Beta_rad_node_->setFloat( AirData_.getAngle1(TipPress_node_->getFloat() - TipPressBias_, Beta1Press_node_->getFloat() - Beta1PressBias_, Beta2Press_node_->getFloat() - Beta2PressBias_, Alpha1Press_node_->getFloat() - Alpha1PressBias_, Alpha2Press_node_->getFloat() - Alpha2PressBias_, kBeta_) );

      // limit beta to +/-45 deg
      if (Beta_rad_node_->getFloat() > 0.7854f) {
        Beta_rad_node_->setFloat( 0.7854f );
      } else if (Beta_rad_node_->getFloat() < -0.7854f) {
        Beta_rad_node_->setFloat( -0.7854f );
      }

    } else { // airspeed < theshold
      Alpha_rad_node_->setFloat( 0.0f );
      Beta_rad_node_->setFloat( 0.0f );
    }
  }
}

void FiveHole1::Clear() {
  TimeInit_s_ = 0.0f;

  TipPressBias_ = 0.0f;
  IAS_node_->setFloat( 0.0f );

  AltAgl_node_->setFloat( 0.0f );
  PressAltBias_ = 0.0f;

  Alpha1PressBias_ = 0.0f;
  Alpha2PressBias_ = 0.0f;
  Alpha_rad_node_->setFloat( 0.0f );

  Beta1PressBias_ = 0.0f;
  Beta2PressBias_ = 0.0f;
  Beta_rad_node_->setFloat( 0.0f );

  TimeLatch_ = false;
  Initialized_ = false;
  T0_us_ = 0;
  NumberSamples_ = 1;

  TipPressKey_.clear();
  StaticPressKey_.clear();
  Alpha1PressKey_.clear();
  Alpha2PressKey_.clear();
  Beta1PressKey_.clear();
  Beta2PressKey_.clear();
}


/* 5-Hole Probe - Method 2 */
// Tip pressure sensor measures relative to the static ring
// Alpha pressure sensor measures Alpha2 - Alpha1
// Beta pressure sensor measures Beta2 - Beta1
void FiveHole2::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Static-Pressure", &StaticPress_node_, &StaticPressKey_);
  LoadInput(Config, SystemName, "Tip-Pressure", &TipPress_node_, &TipPressKey_);
  LoadInput(Config, SystemName, "Alpha-Pressure", &AlphaPress_node_, &AlphaPressKey_);
  LoadInput(Config, SystemName, "Beta-Pressure", &BetaPress_node_, &BetaPressKey_);

  LoadOutput(Config, SystemName, "OutputAltitude", &AltAgl_node_);
  LoadOutput(Config, SystemName, "OutputIas", &IAS_node_);
  LoadOutput(Config, SystemName, "OutputAlpha", &Alpha_rad_node_);
  LoadOutput(Config, SystemName, "OutputBeta", &Beta_rad_node_);

  // Alpha and Beta Calibration constants
  LoadVal(Config, "Alpha-Calibration", &kAlpha_);
  LoadVal(Config, "Beta-Calibration", &kBeta_);

  // Initialization time
  LoadVal(Config, "Initialization-Time", &TimeInit_s_);
}

void FiveHole2::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }

  // compute the elapsed time
  float TimeElapsed_s = ((float)(micros() - T0_us_)) / 1e6;

  // if less than init time, compute bias
  if (TimeElapsed_s < TimeInit_s_) {
    // compute pressure biases, using Welford's algorithm
    PressAltBias_ += (AirData_.getPressureAltitude(StaticPress_node_->getFloat()) - PressAltBias_) / (float)NumberSamples_;
    TipPressBias_ += (TipPress_node_->getFloat() - TipPressBias_) / (float)NumberSamples_;
    AlphaPressBias_ += (AlphaPress_node_->getFloat() - AlphaPressBias_) / (float)NumberSamples_;
    BetaPressBias_ += (BetaPress_node_->getFloat() - BetaPressBias_) / (float)NumberSamples_;

    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool FiveHole2::Initialized() {
  return Initialized_;
}

void FiveHole2::Run(Mode mode) {

  if (mode!=kStandby) {
    // compute indicated airspeed
    IAS_node_->setFloat( AirData_.getIAS(TipPress_node_->getFloat() - TipPressBias_) );

    // compute altitude above ground level
    AltAgl_node_->setFloat( AirData_.getAGL(StaticPress_node_->getFloat(), PressAltBias_) );

    // compute Alpha and Beta
    // if the airspeed is low, the non-dimensionalizing pressures on the side port will be very small and result in huge angles
    if (IAS_node_->getFloat() > 2.0f) {
      // compute alpha
      Alpha_rad_node_->setFloat( AirData_.getAngle2(TipPress_node_->getFloat() - TipPressBias_, AlphaPress_node_->getFloat() - AlphaPressBias_, kAlpha_) );

      // limit alpha to +/-45 deg
      if (Alpha_rad_node_->getFloat() > 0.7854f) {
        Alpha_rad_node_->setFloat( 0.7854f );
      } else if (Alpha_rad_node_->getFloat() < -0.7854f) {
        Alpha_rad_node_->setFloat( -0.7854f );
      }

      // compute beta
      Beta_rad_node_->setFloat( AirData_.getAngle2(TipPress_node_->getFloat() - TipPressBias_, BetaPress_node_->getFloat() - BetaPressBias_, kBeta_) );

      // limit beta to +/-45 deg
      if (Beta_rad_node_->getFloat() > 0.7854f) {
        Beta_rad_node_->setFloat( 0.7854f );
      } else if (Beta_rad_node_->getFloat() < -0.7854f) {
        Beta_rad_node_->setFloat( -0.7854f );
      }

    } else { // airspeed < theshold
      Alpha_rad_node_->setFloat( 0.0f );
      Beta_rad_node_->setFloat( 0.0f );
    }
  }
}

void FiveHole2::Clear() {
  TimeInit_s_ = 0.0f;

  TipPressBias_ = 0.0f;
  IAS_node_->setFloat( 0.0f );

  AltAgl_node_->setFloat( 0.0f );
  PressAltBias_ = 0.0f;

  AlphaPressBias_ = 0.0f;
  Alpha_rad_node_->setFloat( 0.0f );

  BetaPressBias_ = 0.0f;
  Beta_rad_node_->setFloat( 0.0f );

  TimeLatch_ = false;
  Initialized_ = false;
  T0_us_ = 0;
  NumberSamples_ = 1;

  TipPressKey_.clear();
  StaticPressKey_.clear();
  AlphaPressKey_.clear();
  BetaPressKey_.clear();
}
