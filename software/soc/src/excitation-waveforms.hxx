/*
excitation-waveforms.hxx
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

#ifndef EXCITATION_WAVEFORMS_HXX_
#define EXCITATION_WAVEFORMS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <Eigen/Dense>
#include "definition-tree.hxx"
#include "generic-function.hxx"

/* Excitation related functions. Each function describes its JSON
configuration below. See generic-function.hxx for more information
on the methods and modes. */

/*
Pulse Class - Adds a pulse to the signal for the specified duration
Example JSON configuration:
{
  "Type": "Pulse"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X,
  "Scale-Factor": X
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the pulse in seconds
   * Amplitude is the pulse amplitude
   * Scale factor is optional and provides another option to scale the output signal
*/

class Pulse: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    float Amplitude = 0.0f;
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

/*
Doublet Class - Adds a doublet to the signal for the specified duration
Example JSON configuration:
{
  "Type": "Doublet"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 2 times the duration
   * Amplitude is the doublet amplitude
*/

class Doublet: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    float Amplitude = 0.0f;
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

/*
Doublet121 Class - Adds a 1-2-1 doublet to the signal for the specified duration
Example JSON configuration:
{
  "Type": "Doublet121"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 4 times the duration
   * Amplitude is the doublet amplitude
*/

class Doublet121: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    float Amplitude = 0.0f;
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

/*
Doublet3211 Class - Adds a 3-2-1-1 doublet to the signal for the specified duration
Example JSON configuration:
{
  "Type": "Doublet3211"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": X
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 7 times the duration
   * Amplitude is the doublet amplitude
*/

class Doublet3211: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    float Amplitude = 0.0f;
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

/*
Linear Chirp Class - Adds a linear chirp to the signal for the specified duration
Example JSON configuration:
{
  "Type": "LinearChirp"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": [start,end],
  "Frequency": [start,end]
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the chirp
   * Amplitude is an array specifying the starting and ending chirp amplitude
   * Frequency is an array specifying the starting and ending frequency in rad/sec
*/

class LinearChirp: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    float Amplitude[2] = {0.0f};
    float Frequency[2] = {0.0f};
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

/*
MultiSine Class - Adds a multisine to the signal for the specified duration
Example JSON configuration:
{
  "Type": "MultiSine"
  "Signal": "SignalName",
  "Time": "Time",
  "Start-Time": X,
  "Duration": X,
  "Amplitude": [X],
  "Frequency": [X],
  "Phase": [X]
  }
}
Where:
   * Signal gives the name of the input and output
   * Time is the full path to the current system time in us
   * Start time is when the excitation will start after engage in seconds
   * Duration is the duration time of the chirp
   * Amplitude, frequency, and phase are vectors specifying the amplitude,
     frequency, and phase of the multisine. They should all be the same length.
*/

class MultiSine: public GenericFunction {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
  void Initialize();
  bool Initialized();
  void Clear(DefinitionTree *DefinitionTreePtr);
private:
  struct Config {
    uint64_t *Time_us;
    float *Signal;
    Eigen::Array<float,Eigen::Dynamic,1> Amplitude;
    Eigen::Array<float,Eigen::Dynamic,1> Frequency;
    Eigen::Array<float,Eigen::Dynamic,1> Phase;
    float StartTime_s = 0.0f;
    float Duration_s = 0.0f;
    float Scale = 1.0f;
  };
  struct Data {
    uint8_t Mode = kStandby;
    float Excitation = 0.0f;
  };
  Config config_;
  Data data_;
  uint64_t Time0_us = 0;
  float ExciteTime_s = 0;
  bool TimeLatch = false;
};

#endif
