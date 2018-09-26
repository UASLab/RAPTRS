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

#ifndef AIRDATA_FUNCTIONS_HXX_
#define AIRDATA_FUNCTIONS_HXX_

#include "AirData.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree.hxx"
#include "generic-function.hxx"
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
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      std::vector<float*> DifferentialPressure;
      float InitTime = 0.0f;
    };
    struct Data {
      std::vector<float> DifferentialPressureBias;
      float AvgDifferentialPressure = 0.0f;
      uint8_t Mode = kStandby;
      float Ias_ms = 0.0f;
    };
    Config config_;
    Data data_;
    std::vector<std::string> DifferentialPressureKeys_;
    std::string ModeKey_,OutputKey_;
    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    uint64_t micros();
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
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      std::vector<float*> StaticPressure;
      float InitTime = 0.0f;
    };
    struct Data {
      float PressAlt0 = 0.0f;
      float AvgStaticPressure = 0.0f;
      uint8_t Mode = kStandby;
      float Agl_m = 0.0f;
    };
    Config config_;
    Data data_;
    std::vector<std::string> StaticPressureKeys_;
    std::string ModeKey_,OutputKey_;
    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    uint64_t micros();
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
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float* StaticPressure;
      float* DifferentialPressure;
      float InitTime = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;

      float DifferentialPressureBias = 0.0f;
      float Ias_ms = 0.0f;

      float PressAlt0 = 0.0f;
      float Agl_m = 0.0f;
    };

    Config config_;
    Data data_;
    std::string DifferentialPressureKey_;
    std::string StaticPressureKey_;
    std::string ModeKey_,OutputIasKey_,OutputAglKey_;

    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    uint64_t micros();
    AirData AirData_;
};

/*
PitotStatic Class - Computes indicated airspeed and altitude above ground level from pitot-static pressures.
Example JSON configuration:
{
  "Type": "FiveHole",
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
class FiveHole: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float* StaticPressure;
      float* TipPressure;
      float* Alpha1Pressure;
      float* Alpha2Pressure;
      float* Beta1Pressure;
      float* Beta2Pressure;
      float InitTime = 0.0f;
      float kAlpha = 0.0f;
      float kBeta = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;

      float PressAlt0 = 0.0f;
      float Agl_m = 0.0f;

      float TipPressureBias = 0.0f;
      float Ias_ms = 0.0f;

      float Alpha1PressureBias = 0.0f;
      float Alpha2PressureBias = 0.0f;
      float Alpha_rad = 0.0f;

      float Beta1PressureBias = 0.0f;
      float Beta2PressureBias = 0.0f;
      float Beta_rad = 0.0f;
    };

    Config config_;
    Data data_;
    std::string ModeKey_;
    std::string TipPressureKey_, StaticPressureKey_, Alpha1PressureKey_, Alpha2PressureKey_, Beta1PressureKey_, Beta2PressureKey_;
    std::string AlphaCalKey_, BetaCalKey_;
    std::string OutputIasKey_, OutputAglKey_, OutputAlphaKey_, OutputBetaKey_;

    bool TimeLatch_ = false;
    bool Initialized_ = false;
    uint64_t T0_us_ = 0;
    size_t NumberSamples_ = 1;
    uint64_t micros();
    AirData AirData_;
};


#endif
