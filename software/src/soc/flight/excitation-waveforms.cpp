/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Chris Regan, Brian Taylor
*/

#include "excitation-waveforms.h"

void Pulse::Configure(const rapidjson::Value& Config) {
   LoadVal(Config, "Duration", &tDur_s_, true);
}

void Pulse::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    *Excite = 1.0;
  }
}

void Pulse::Clear() {
  tDur_s_ = 0.0f;
}

void Doublet::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
}

void Doublet::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    *Excite = 1.0f;
  } else if (tExc_s < 2.0f * tDur_s_) {
    *Excite = -1.0f;
  }
}

void Doublet::Clear() {
  tDur_s_ = 0.0f;
}

void Doublet121::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
}

void Doublet121::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    *Excite = 1.0f;
  } else if (tExc_s < 3.0f*tDur_s_) {
    *Excite = -1.0f;
  } else if (tExc_s < 4.0f*tDur_s_) {
    *Excite = 1.0f;
  }
}

void Doublet121::Clear() {
  tDur_s_ = 0.0f;
}

void Doublet3211::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
}

void Doublet3211::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < (3.0f*tDur_s_)) {
    *Excite = 1.0f;
  } else if (tExc_s < (5.0f*tDur_s_)) {
    *Excite = -1.0f;
  } else if (tExc_s < (6.0f*tDur_s_)) {
    *Excite = 1.0f;
  } else if (tExc_s < (7.0f*tDur_s_)) {
    *Excite = -1.0f;
  }
}

void Doublet3211::Clear() {
  tDur_s_ = 0.0f;
}

void LinearChirp::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);

  Amp0_ = 1.0f;
  Amp1_ = Amp0_;
  LoadVal(Config, "AmpStart", &Amp0_);
  LoadVal(Config, "AmpEnd", &Amp1_);

  LoadVal(Config, "FreqStart", &Freq0_rps_, true);
  Freq1_rps_ = Freq0_rps_;
  LoadVal(Config, "FreqEnd", &Freq1_rps_);

  // Constant for linear varying Frequency
  FreqK_rad_ = (Freq1_rps_ - Freq0_rps_) / tDur_s_;

  // Constant for linear varying Amplitude
  AmpK_ = (Amp1_ - Amp0_) / tDur_s_;
}

void LinearChirp::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
      // linear varying instantanious Frequency
      float Freq_rps = Freq0_rps_ + FreqK_rad_ * tExc_s;

      // linear varying Amplitude
      float Amp_nd = Amp0_ + AmpK_ * tExc_s;

      // chirp Equation, note the factor of 2.0 is correct!
      *Excite = Amp_nd * sinf((Freq_rps / 2.0f) * tExc_s);
    }
}

void LinearChirp::Clear() {
  tDur_s_ = 0.0f;
  Freq0_rps_ = 0.0f;
  Freq1_rps_ = 0.0f;
  FreqK_rad_ = 0.0f;
  Amp0_ = 1.0f;
  Amp1_ = 1.0f;
  AmpK_ = 0.0f;
}

void LogChirp::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);

  Amp0_ = 1.0f;
  Amp1_ = Amp0_;
  LoadVal(Config, "AmpStart", &Amp0_);
  LoadVal(Config, "AmpEnd", &Amp1_);

  LoadVal(Config, "FreqStart", &Freq0_rps_, true);
  Freq1_rps_ = Freq0_rps_;
  LoadVal(Config, "FreqEnd", &Freq1_rps_);

  // Constants for log varying Frequency
  FreqK_rad_ = pow(Freq1_rps_ / Freq0_rps_, (1/tDur_s_));
  FreqLogK_ = log(FreqK_rad_);

  // Constants for linear varying Amplitude
  AmpK_ = (Amp1_ - Amp0_) / tDur_s_;
}

void LogChirp::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    // log varying instantaneous Frequency
    float Freq_rps = Freq0_rps_ * (pow(FreqK_rad_, tExc_s) - 1) / (tExc_s * FreqLogK_);

    // linear varying Amplitude
    float Amp_nd = Amp0_ + AmpK_ * tExc_s;

    // chirp Equation
    *Excite = Amp_nd * sinf(Freq_rps * tExc_s);
  }
}

void LogChirp::Clear() {
  tDur_s_ = 0.0f;
  Freq0_rps_ = 0.0f;
  Freq1_rps_ = 0.0f;
  FreqK_rad_ = 0.0f;
  FreqLogK_ = 0.0f;
  Amp0_ = 1.0f;
  Amp1_ = 1.0f;
  AmpK_ = 0.0f;
}

// 1-Cos Excitation
void Pulse_1_Cos::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
  LoadVal(Config, "Pause", &tPause_s_, true);

  Freq_rps_ = (2 * M_PI) / tDur_s_;
}

void Pulse_1_Cos::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < (tDur_s_ + tPause_s_)) {
    if (tExc_s < (0.5 * tDur_s_)) {
      *Excite =  0.5 * (1.0 - cosf(Freq_rps_ * tExc_s));
    } else if (tExc_s < (0.5 * tDur_s_ + tPause_s_)) {
      *Excite = 1.0f;
    } else {
      *Excite = 0.5 * (1.0 - cosf(Freq_rps_ * (tExc_s - tPause_s_)));
    }
  }
}

void Pulse_1_Cos::Clear() {
  tDur_s_ = 0.0f;
  tPause_s_ = 0.0f;
  Freq_rps_ = 0.0f;
}

void MultiSine::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
  LoadVal(Config, "Amplitude", &Amp_, true);
  LoadVal(Config, "Frequency", &Freq_rps_, true);
  LoadVal(Config, "Phase", &Phase_rad_, true);
}

void MultiSine::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    // Compute the Waveform: sum(Amp .* sin(Freq * t + phase))
    *Excite = (Amp_ * (Freq_rps_ * tExc_s + Phase_rad_).sin()).sum();
  }
}

void MultiSine::Clear() {
  Amp_.resize(0,1);
  Freq_rps_.resize(0,1);
  Phase_rad_.resize(0,1);
  tDur_s_ = 0.0f;
}



void Sampled::Configure(const rapidjson::Value& Config) {
  LoadVal(Config, "Duration", &tDur_s_, true);
  LoadVal(Config, "dt", &dt_s_, true);
  LoadVal(Config, "Sample", &Sample_, true);
}

void Sampled::Run(float tExc_s, float *Excite) {
  *Excite = 0.0f;

  if (tExc_s < tDur_s_) {
    size_t iSamp = (size_t) (tExc_s / dt_s_);
    *Excite = Sample_[iSamp];
  }
}

void Sampled::Clear() {
  tDur_s_ = 0.0f;
  dt_s_ = 0.0f;
  Sample_.resize(0,1);
}
