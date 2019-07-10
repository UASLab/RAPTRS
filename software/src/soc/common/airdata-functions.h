/*
airdata-functions.hxx
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

#pragma once

#include "AirData.h"
#include "configuration.h"
#include "generic-function.h"
#include <sys/time.h>

/*
Indicated Airspeed Class - Computes indicated airspeed from differential pressure.
Example JSON configuration:
{
  "Type": "IAS",
  "Output": "OutputName",
  "Differential-Pressure": [X]
  "Initialization-Time": X
}
Where:
   * Output gives a convenient name for the block (i.e. Ias_ms).
   * Differential pressure is a vector of all differential pressure sources.
     Data from all sources will be averaged and used.
   * Initialization time is the amount of time used during initialization for
     bias estimation.
Pressures are expected to be in Pa and airspeed is in m/s
*/
class IndicatedAirspeed: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    std::vector<ElementPtr> DiffPress_node_;
    float TimeInit_s_ = 0.0f;

    std::vector<float> DiffPressBias_;
    float DiffPressMean_ = 0.0f;
    ElementPtr IAS_node_;

    std::vector<std::string> DiffPressKeys_;
    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    AirData AirData_;
};

/*
AGL Altitude Class - Computes altitude above ground level from static pressure
Example JSON configuration:
{
  "Type": "AGL",
  "Output": "OutputName",
  "Static-Pressure": [X]
  "Initialization-Time": X
}
Where:
   * Output gives a convenient name for the block (i.e. Agl_m).
   * Static pressure is a vector of all static pressure sources.
     Data from all sources will be averaged and used.
   * Initialization time is the amount of time used during initialization for
     bias estimation.
Pressures are expected to be in Pa and altitude is in m
*/
class AglAltitude: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    std::vector<ElementPtr> StaticPress_node_;
    float TimeInit_s_ = 0.0f;

    float PressAltBias_;
    float StaticPressMean_ = 0.0f;
    ElementPtr AltAgl_node_;

    std::vector<std::string> StaticPressKeys_;
    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    AirData AirData_;
};


/*
PitotStatic Class - Computes indicated airspeed and altitude above ground level from pitot-static pressures.
Example JSON configuration:
{
  "Type": "PitotStatic",
  "Output": "OutputName",
  "OutputIas": "OutputIasName",
  "OutputAlt": "OutputAglName",
  "Differential-Pressure": X,
  "Static-Pressure": X,
  "Initialization-Time": X
}
Where:
   * Output gives a convenient name for the block (i.e. PitotStatic).
   * Differential pressure is differential pressure source, etc.
   * Initialization time is the amount of time used during initialization for
     bias estimation.
Pressures are expected to be in Pa and airspeed is in m/s
*/
class PitotStatic: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    float TimeInit_s_ = 0.0f;
    ElementPtr StaticPress_node_;
    ElementPtr DiffPress_node_;

    float DiffPressBias_ = 0.0f;
    ElementPtr IAS_node_;

    float PressAltBias_ = 0.0f;
    ElementPtr AltAgl_node_;

    std::string DiffPressKey_;
    std::string StaticPressKey_;

    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    AirData AirData_;
};

/*
5Hole1 Class - Computes indicated airspeed and altitude above ground level from pitot-static pressures.
Example JSON configuration:
{
  "Type": "FiveHole1",
  "Output": "OutputName",
  "OutputIas": "OutputIasName",
  "OutputAlt": "OutputAglName",
  "OutputAlpha": "OutputAlphaName",
  "OutputBeta": "OutputBetaName",
  "Static-Pressure": X,
  "Tip-Pressure": X,
  "Alpha1-Pressure": X,
  "Alpha2-Pressure": X,
  "Beta1-Pressure": X,
  "Beta2-Pressure": X,
  "Initialization-Time": X
}
Where:
   * Output gives a convenient name for the block (i.e. 5Hole).
   * Tip pressure is tip pressure source, etc.
   * Initialization time is the amount of time used during initialization for
     bias estimation.
Pressures are expected to be in Pa and airspeed is in m/s
*/
class FiveHole1: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    ElementPtr StaticPress_node_;
    ElementPtr TipPress_node_;
    ElementPtr Alpha1Press_node_;
    ElementPtr Alpha2Press_node_;
    ElementPtr Beta1Press_node_;
    ElementPtr Beta2Press_node_;
    float TimeInit_s_ = 0.0f;
    float kAlpha_ = 0.0f;
    float kBeta_ = 0.0f;

    float PressAltBias_ = 0.0f;
    ElementPtr AltAgl_node_;

    float TipPressBias_ = 0.0f;
    ElementPtr IAS_node_;

    float Alpha1PressBias_ = 0.0f;
    float Alpha2PressBias_ = 0.0f;
    ElementPtr Alpha_rad_node_;

    float Beta1PressBias_ = 0.0f;
    float Beta2PressBias_ = 0.0f;
    ElementPtr Beta_rad_node_;

    std::string TipPressKey_, StaticPressKey_, Alpha1PressKey_, Alpha2PressKey_, Beta1PressKey_, Beta2PressKey_;
    std::string AlphaCalKey_, BetaCalKey_;

    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    AirData AirData_;
};

/*
5Hole2 Class - Computes indicated airspeed and altitude above ground level from pitot-static pressures.
Example JSON configuration:
{
  "Type": "FiveHole2",
  "Output": "OutputName",
  "OutputIas": "OutputIasName",
  "OutputAlt": "OutputAglName",
  "OutputAlpha": "OutputAlphaName",
  "OutputBeta": "OutputBetaName",
  "Static-Pressure": X,
  "Tip-Pressure": X,
  "Alpha-Pressure": X,
  "Beta-Pressure": X,
  "Initialization-Time": X
}
Where:
   * Output gives a convenient name for the block (i.e. 5Hole).
   * Tip pressure is tip pressure source, etc.
   * Initialization time is the amount of time used during initialization for
     bias estimation.
Pressures are expected to be in Pa and airspeed is in m/s
*/
class FiveHole2: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemName);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    ElementPtr StaticPress_node_;
    ElementPtr TipPress_node_;
    ElementPtr AlphaPress_node_;
    ElementPtr BetaPress_node_;
    float TimeInit_s_ = 0.0f;
    float kAlpha_ = 0.0f;
    float kBeta_ = 0.0f;

    float PressAltBias_ = 0.0f;
    ElementPtr AltAgl_node_;

    float TipPressBias_ = 0.0f;
    ElementPtr IAS_node_;

    float AlphaPressBias_ = 0.0f;
    ElementPtr Alpha_rad_node_;

    float BetaPressBias_ = 0.0f;
    ElementPtr Beta_rad_node_;

    std::string TipPressKey_, StaticPressKey_, AlphaPressKey_, BetaPressKey_;
    std::string AlphaCalKey_, BetaCalKey_;

    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    AirData AirData_;
};
