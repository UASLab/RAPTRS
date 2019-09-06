/*
control-functions.cc
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

#include "control-functions.h"

/* PID2 class methods, see control-functions.hxx for more information */
void PID2Class::Configure(const rapidjson::Value& Config, std::string SystemPath) {
  float Kp = 1.0;
  float Ki = 0.0;
  float Kd = 0.0;
  float Tf = 0.0;
  float b = 1.0;
  float c = 1.0;
  float Min = std::numeric_limits<float>::lowest();
  float Max = std::numeric_limits<float>::max();

  LoadInput(Config, SystemPath, "Reference", &reference_node, &ReferenceKey_);
  LoadInput(Config, SystemPath, "Feedback", &feedback_node, &FeedbackKey_);
  LoadOutput(Config, SystemPath, "Output", &output_node);

  std::string OutputKey = Config["Output"].GetString();

  ff_node = deftree.initElement(SystemPath + "/" + OutputKey + "FF", ": System feedforward component", LOG_FLOAT, LOG_NONE);
  fb_node = deftree.initElement(SystemPath + "/" + OutputKey + "FB", ": System feedback component", LOG_FLOAT, LOG_NONE);

  LoadVal(Config, "dt", &dt_);
  if (dt_ > 0.0) {
    UseFixedTimeSample = true;
  } else {
    LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
  }

  LoadVal(Config, "Kp", &Kp);
  LoadVal(Config, "Ki", &Ki);
  LoadVal(Config, "Kd", &Kd);
  if (Kd != 0.0){
    LoadVal(Config, "Tf", &Tf);
  }

  LoadVal(Config, "b", &b);
  LoadVal(Config, "c", &c);
  LoadVal(Config, "Min", &Min);
  LoadVal(Config, "Max", &Max);

  // configure PID2 Class
  PID2Class_.Configure(Kp, Ki, Kd, Tf, b, c, Min, Max);
}

void PID2Class::Initialize() {}
bool PID2Class::Initialized() {return true;}

void PID2Class::Run(Mode mode) {
  // sample time
  float dt = 0.0f;
  if(!UseFixedTimeSample) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // Run
  float y = 0.0f;
  float ff = 0.0f;
  float fb = 0.0f;

  PID2Class_.Run(mode, reference_node->getFloat(), feedback_node->getFloat(), dt, &y, &ff, &fb);

  output_node->setFloat(y);
  ff_node->setFloat(ff);
  fb_node->setFloat(fb);
}

void PID2Class::Clear() {
  UseFixedTimeSample = false;
  output_node->setFloat(0.0f);
  ff_node->setFloat(0.0f);
  fb_node->setFloat(0.0f);
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  TimeKey_.clear();
  PID2Class_.Clear();
}

/* PID class methods, see control-functions.hxx for more information */
void PIDClass::Configure(const rapidjson::Value& Config,std::string SystemPath) {
  float Kp = 1.0;
  float Ki = 0.0;
  float Kd = 0.0;
  float Tf = 0.0;
  float Min = std::numeric_limits<float>::lowest();
  float Max = std::numeric_limits<float>::max();

  LoadInput(Config, SystemPath, "Input", &input_node, &InputKey_);
  LoadOutput(Config, SystemPath, "Output", &output_node);

  std::string OutputKey = Config["Output"].GetString();

  ff_node = deftree.initElement(SystemPath + "/" + OutputKey + "FF", ": System feedforward component", LOG_FLOAT, LOG_NONE);
  fb_node = deftree.initElement(SystemPath + "/" + OutputKey + "FB", ": System feedback component", LOG_FLOAT, LOG_NONE);

  LoadVal(Config, "dt", &dt_);
  if (dt_ > 0.0) {
    UseFixedTimeSample = true;
  } else {
    LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
  }

  LoadVal(Config, "Kp", &Kp);
  LoadVal(Config, "Ki", &Ki);
  LoadVal(Config, "Kd", &Kd);
  if (Kd != 0.0){
    LoadVal(Config, "Tf", &Tf);
  }

  LoadVal(Config, "Min", &Min);
  LoadVal(Config, "Max", &Max);

  // configure using PID2 algorithm Class
  PID2Class_.Configure(Kp, Ki, Kd, Tf, 1.0, 1.0, Min, Max);
}

void PIDClass::Initialize() {}
bool PIDClass::Initialized() {return true;}

void PIDClass::Run(Mode mode) {
  // sample time
  float dt = 0.0f;
  if(!UseFixedTimeSample) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // Run
  float y = 0.0f;
  float ff = 0.0f;
  float fb = 0.0f;

  PID2Class_.Run(mode, input_node->getFloat(), 0.0, dt, &y, &ff, &fb);

  output_node->setFloat(y);
  ff_node->setFloat(ff);
  fb_node->setFloat(fb);
}

void PIDClass::Clear() {
  UseFixedTimeSample = false;
  output_node->setFloat(0.0f);
  ff_node->setFloat(0.0f);
  fb_node->setFloat(0.0f);
  InputKey_.clear();
  TimeKey_.clear();
  PID2Class_.Clear();
}


/* SS class methods, see control-functions.hxx for more information */
void SSClass::Configure(const rapidjson::Value& Config, std::string SystemPath) {
  // I/O signals
  LoadInput(Config, SystemPath, "Inputs", &u_node, &InputKeys_);
  LoadOutput(Config, SystemPath, "Outputs", &y_node);

  // Matrices
  LoadVal(Config, "A", &A);
  LoadVal(Config, "B", &B);
  LoadVal(Config, "C", &C);
  LoadVal(Config, "D", &D);

  // resize input vector
  int numU = C.rows();
  u.resize(numU);
  u.setZero(numU);

  // resize state vector
  int numX = A.rows();
  x.resize(numX);
  x.setZero(numX);

  // Resize output vector
  int numY = C.rows();
  y.resize(numY);
  y.setZero(numY);

  // Sample time (required)
  LoadVal(Config, "dt", &dt);
  if (dt > 0.0) {
    UseFixedTimeSample = true;
  } else {
    LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
  }

  // Limits
  Min.resize(numY);
  Min.setConstant(numY, std::numeric_limits<float>::lowest());

  LoadVal(Config, "Min", &Min);

  Max.resize(numY);
  Max.setConstant(numY, std::numeric_limits<float>::max());

  LoadVal(Config, "Max", &Max);

  // configure SS Class
  SSClass_.Configure(A, B, C, D, dt, Min, Max);
}

void SSClass::Initialize() {}
bool SSClass::Initialized() {return true;}

void SSClass::Run(Mode mode) {
  float dt_curr = 0.0f;

  // sample time computation
  float dt = 0;
  if (UseFixedTimeSample == false) {
    dt_curr = *TimeSource - timePrev;
    timePrev = *TimeSource;
    if (dt_curr > 2*dt) {dt_curr = dt;} // Catch large dt
    if (dt_curr <= 0) {dt_curr = dt;} // Catch negative and zero dt
  } else {
    dt_curr = dt;
  }

  // inputs to Eigen3 vector
  for (size_t i=0; i < u_node.size(); i++) {
    u(i) = u_node[i]->getFloat();
  }

  // Call Algorithm
  SSClass_.Run(mode, u, dt, &y);

  // outputs to nodes
  for (size_t i=0; i < y_node.size(); i++) {
    y_node[i]->setFloat(y(i));
  }
}

void SSClass::Clear() {
  UseFixedTimeSample = false;
  A.resize(0,0);
  B.resize(0,0);
  C.resize(0,0);
  D.resize(0,0);
  InputKeys_.clear();
  TimeKey_.clear();
  SSClass_.Clear();
}

/* Tecs class methods, see control-functions.hxx for more information */
void TecsClass::Configure(const rapidjson::Value& Config, std::string SystemPath) {
  // Inputs
  std::string RefSpeedKey, FeedbackSpeedKey;
  LoadInput(Config, SystemPath, "RefSpeed", &ref_vel_node, &RefSpeedKey);
  LoadInput(Config, SystemPath, "FeedbackSpeed", &vel_node, &FeedbackSpeedKey);

  std::string RefAltitudeKey, FeedbackAltitudeKey;
  LoadInput(Config, SystemPath, "RefAltitude", &ref_agl_node, &RefAltitudeKey);
  LoadInput(Config, SystemPath, "FeedbackAltitude", &agl_node, &FeedbackAltitudeKey);

  // Outputs
  LoadOutput(Config, SystemPath, "OutputTotal", &error_total_node);
  LoadOutput(Config, SystemPath, "OutputDiff", &error_diff_node);

  // Parameters
  LoadVal(Config, "mass_kg", &mass_kg, true);
  LoadVal(Config, "weight_bal", &weight_bal, true);
  LoadVal(Config, "min_mps", &min_mps, true);
  LoadVal(Config, "max_mps", &max_mps, true);
}

void TecsClass::Initialize() {
  // sanity checks
  if ( mass_kg < 0.01 ) {
    mass_kg = 2.5;
  }
  if ( weight_bal < 0.0 ) {
    weight_bal = 0.0;
  } else if ( weight_bal > 2.0 ) {
    weight_bal = 2.0;
  }
  initFlag = true;
}
bool TecsClass::Initialized() {return initFlag;}

void TecsClass::Run(Mode mode) {
  const float g = 9.807f;     // acceleration due to gravity, m/s/s

  if ( initFlag == false ) {
    Initialize();
  }

  // Feedback energy
  float energy_pot = mass_kg * g * (agl_node->getFloat());
  float energy_kin = 0.5 * mass_kg * (vel_node->getFloat()) * (vel_node->getFloat());

  // Reference energy
  float target_pot = mass_kg * g * (ref_agl_node->getFloat());
  float target_kin = 0.5 * mass_kg * (ref_vel_node->getFloat()) * (ref_vel_node->getFloat());

  // Energy error
  float error_pot = target_pot - energy_pot;
  float error_kin = target_kin - energy_kin;

  // Compute min & max kinetic energy allowed (based on configured
  // operational speed range)
  float min_kinetic = 0.5 * mass_kg * min_mps * min_mps;
  float max_kinetic = 0.5 * mass_kg * max_mps * max_mps;

  // Set min & max kinetic energy errors allowed (prevents us from
  // exceeding allowed kinetic energy range)
  float min_error = min_kinetic - energy_kin;
  float max_error = max_kinetic - energy_kin;

  // if min_error > 0: we are underspeed
  // if max_error < 0: we are overspeed

  // total energy error and (weighted) energy balance
  float error_total = error_pot + error_kin;
  float error_diff =  (2.0 - weight_bal) * error_kin - weight_bal * error_pot;

  // clamp error_diff to kinetic error range.  This prevents tecs from
  // requesting a pitch attitude that would over/under-speed the
  // aircraft.
  if ( error_diff < min_error ) { error_diff = min_error; }
  if ( error_diff > max_error ) { error_diff = max_error; }

  // clamp max total error to avoid an overspeed condition in a climb
  // if max pitch angle is saturated.
  if ( error_total > max_error ) { error_total = max_error; }

  error_total_node->setFloat(error_total);
  error_diff_node->setFloat(error_diff);
}

void TecsClass::Clear() {
}
