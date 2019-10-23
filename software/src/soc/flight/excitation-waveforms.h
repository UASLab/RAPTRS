/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Brian Taylor
*/

#pragma once

#include <math.h>
#include "configuration.h"

// Base Class
class Waveform {
  public:
    virtual void Configure(const rapidjson::Value& Config) {};
    virtual void Run(float tExc_s, float *Excite) {};
    virtual void Clear() {};
};


/*
Pulse Class - Adds a pulse to the signal for the specified duration
Example JSON configuration:
{"Type": "Pulse", "Duration": X}

Where:
   * Duration is the duration time of the pulse in seconds
*/

class Pulse: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
};

/*
Doublet Class - Adds a doublet to the signal for the specified duration
Example JSON configuration:
{"Type": "Doublet",  "Duration": X}

Where:
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 2 times the duration
*/

class Doublet: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
};

/*
Doublet121 Class - Adds a 1-2-1 doublet to the signal for the specified duration
Example JSON configuration:
{"Type": "Doublet121",  "Duration": X}

Where:
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 4 times the duration
*/

class Doublet121: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
};

/*
Doublet3211 Class - Adds a 3-2-1-1 doublet to the signal for the specified duration
Example JSON configuration:
{"Type": "Doublet3211",  "Duration": X}

Where:
   * Duration is the duration time of one pulse in seconds, the total doublet
     time will be 7 times the duration
*/

class Doublet3211: public Waveform {
public:
  void Configure(const rapidjson::Value& Config);
  void Run(float tExc_s, float *Excite);
  void Clear();
private:
  float tDur_s_ = 0.0f;
};

/*
Linear Chirp Class - Adds a linear chirp to the signal for the specified duration
Example JSON configuration:
{
  "Type": "LinearChirp", "Duration": X,
  "AmpStart": X, "AmpEnd": X,
  "FreqStart": X, "FreqEnd": X
}
Where:
   * Duration is the duration time of the chirp
   * Amplitude is starting and ending chirp Amplitude
   * Frequency is starting and ending Frequency in rad/sec
*/

class LinearChirp: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;

    float Amp0_ = 1.0f;
    float Amp1_ = 1.0f;
    float Freq0_rps_ = 0.0f;
    float Freq1_rps_ = 0.0f;

    float AmpK_, FreqK_rad_;
};

/*
Log Chirp Class - Adds a log chirp to the signal for the specified duration
Example JSON configuration:
{
  "Type": "LogChirp", "Duration": X,
  "AmpStart": X, "AmpEnd": X,
  "FreqStart": X, "FreqEnd": X
  }
}
Where:
   * Duration is the duration time of the chirp
   * Amplitude is an array specifying the starting and ending chirp Amplitude
   * Frequency is an array specifying the starting and ending Frequency in rad/sec
*/

class LogChirp: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;

    float Amp0_ = 1.0f;
    float Amp1_ = 1.0f;
    float Freq0_rps_ = 0.0f;
    float Freq1_rps_ = 0.0f;

    float AmpK_, FreqK_rad_, FreqLogK_;
};

/*
1-Cos Class - Adds a 1-cosine signal for the specified duration
Example JSON configuration:
{
  "Type": "1-Cos"
  "Duration": X,
  "Pause": X,
  }
}
Where:
   * Duration is the duration time of the 1-cos wave
   * Pause is the pause time at the peak of the 1-cos wave
*/

class Pulse_1_Cos: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
    float tPause_s_ = 0.0f;
    float Freq_rps_ = 0.0f;
};

/*
MultiSine Class - Adds a multisine to the signal for the specified duration
Example JSON configuration:
{
  "Type": "MultiSine"
  "Duration": X,
  "Amplitude": [X],
  "Frequency": [X],
  "Phase": [X]
  }
}
Where:
   * Duration is the duration time of the chirp
   * Amplitude, Frequency, and phase are vectors specifying the Amplitude,
     Frequency, and phase of the multisine. They should all be the same length.
*/

class MultiSine: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
    Eigen::ArrayXf Amp_;
    Eigen::ArrayXf Freq_rps_;
    Eigen::ArrayXf Phase_rad_;
};

/*
Sampled Class - Adds a Pre-defined time history to the signal
Example JSON configuration:
{
  "Type": "Sampled"
  "Duration": X,
  "dt": X,
  "Sample": [X]
  }
}
Where:
   * Duration is the duration time of the chirp
   * Sample is a vector of each element of the time history
*/

class Sampled: public Waveform {
  public:
    void Configure(const rapidjson::Value& Config);
    void Run(float tExc_s, float *Excite);
    void Clear();
  private:
    float tDur_s_ = 0.0f;
    float dt_s_ = 0.0f;
    Eigen::ArrayXf Sample_;
};
