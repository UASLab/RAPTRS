/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#include "control-functions.h"

/* PID2 class methods, see control-functions.hxx for more information */
void PID2ClassExcite::Configure(const rapidjson::Value& Config, std::string SystemPath) {
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
    if (Tf < 0) {
      Tf = 0;
    }
  }

  LoadVal(Config, "b", &b);
  LoadVal(Config, "c", &c);
  LoadVal(Config, "Min", &Min);
  LoadVal(Config, "Max", &Max);

  // configure PID2 Class
  PID2ClassExcite_.Configure(Kp, Ki, Kd, Tf, b, c, Min, Max);
}

void PID2ClassExcite::Initialize() {}
bool PID2ClassExcite::Initialized() {return true;}

void PID2ClassExcite::Run(Mode mode) {
  // sample time
  float dt = 0.0f;
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // Run
  float y = 0.0f;
  float ff = 0.0f;
  float fb = 0.0f;

  PID2ClassExcite_.Run(mode, reference_node->getFloat(), feedback_node->getFloat(), dt, &y, &ff, &fb);

  output_node->setFloat(y);
  ff_node->setFloat(ff);
  fb_node->setFloat(fb);
}

void PID2ClassExcite::Clear() {
  UseFixedTimeSample = false;
  output_node->setFloat(0.0f);
  ff_node->setFloat(0.0f);
  fb_node->setFloat(0.0f);
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  TimeKey_.clear();
  PID2ClassExcite_.Clear();
}

//
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
    if (Tf < 0) {
      Tf = 0;
    }
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
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // Run
  float y = 0.0f;

  PID2Class_.Run(mode, reference_node->getFloat(), feedback_node->getFloat(), dt, &y);

  output_node->setFloat(y);
}

void PID2Class::Clear() {
  UseFixedTimeSample = false;
  output_node->setFloat(0.0f);
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
    if (Tf < 0) {
      Tf = 0;
    }
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
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // Run
  float y = 0.0f;

  PID2Class_.Run(mode, input_node->getFloat(), 0.0, dt, &y);

  output_node->setFloat(y);
}

void PIDClass::Clear() {
  UseFixedTimeSample = false;
  output_node->setFloat(0.0f);
  InputKey_.clear();
  TimeKey_.clear();
  PID2Class_.Clear();
}


/* SS class methods, see control-functions.hxx for more information */
void SSClass::Configure(const rapidjson::Value& Config, std::string SystemPath) {
  // I/O signals
  LoadInput(Config, SystemPath, "Inputs", &u_node, &InputKeys_);
  LoadOutput(Config, SystemPath, "Outputs", &y_node);

  // std::string OutputKey = Config["Output"].GetString();
  int LatchInit = 0;
  LoadVal(Config, "LatchInit", &LatchInit);
  bool LatchInitBool = (bool) LatchInit;

  // Matrices
  LoadVal(Config, "A", &A, true);
  LoadVal(Config, "B", &B, true);
  LoadVal(Config, "C", &C, true);
  LoadVal(Config, "D", &D, true);

  // resize input vector
  int numU = D.cols();
  u_node.resize(numU);
  u.resize(numU);
  u.setZero(numU);

  // Resize output vector
  int numY = D.rows();
  // y_node.resize(numY);
  y.resize(numY);
  y.setZero(numY);

  // Sample time (required)
  LoadVal(Config, "dt", &dt_);
  if (dt_ > 0.0) {
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
  SSClass_.Configure(A, B, C, D, dt_, Min, Max, LatchInitBool);
}

void SSClass::Initialize() {}
bool SSClass::Initialized() {return true;}

void SSClass::Run(Mode mode) {
  // sample time computation
  float dt = 0;
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // inputs to Eigen3 vector
  for (size_t i=0; i < u_node.size(); i++) {
    u(i) = u_node[i]->getFloat();
  }

  // Call Algorithm
  SSClass_.Run(mode, u, dt, &y); // Call SS System

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

//
// void FDIPEClass::Configure(const rapidjson::Value& Config, std::string SystemPath) {
//   // I/O Signals
//   LoadInput(Config, SystemPath, "Inputs", &u_node, &InputKeys_);
//   LoadInput(Config, SystemPath, "pRef", &p_ref_node, &RollKey_);
//
//   p_exp_node = deftree.initElement(SystemPath + "/" + "p_exp", ": Expected roll rate computed for a nominal system", LOG_FLOAT, LOG_NONE);
//   residual_raw_node = deftree.initElement(SystemPath + "/" + "residual_raw", ": Residual is difference between measured and expected roll rates", LOG_FLOAT, LOG_NONE);
//
//   LoadOutput(Config, SystemPath, "Output", &residual_filt_node);
//
//   std::string OutputKey = Config["Output"].GetString();
//
//   // Sample time (required)
//   LoadVal(Config, "dt", &dt_);
//   if (dt_ > 0.0) {
//     UseFixedTimeSample = true;
//   } else {
//     LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
//   }
//
//   LoadVal(Config, "SS_A", &A, true);
//   LoadVal(Config, "SS_B", &B, true);
//   LoadVal(Config, "SS_C", &C, true);
//   LoadVal(Config, "SS_D", &D, true);
//
//   // resize input vector
//   int numU = D.cols();
//   u_node.resize(numU);
//   u.resize(numU);
//   u.setZero(numU);
//
//   // Resize output vector
//   int numY = D.rows();
//   y.resize(numY);
//   y.setZero(numY);
//
//   // SS Limits
//   Min.resize(numY);
//   Min.setConstant(numY, std::numeric_limits<float>::lowest());
//
//   LoadVal(Config, "SS_Min", &Min);
//
//   Max.resize(numY);
//   Max.setConstant(numY, std::numeric_limits<float>::max());
//
//   LoadVal(Config, "SS_Max", &Max);
//
//   // configure using SS algorithm Class
//   SSClass_.Configure(A, B, C, D, dt_, Min, Max);
//
//   // Residual filter
//   LoadVal(Config, "Filt_Num", &num, true);
//   LoadVal(Config, "Filt_Den", &den, true);
//
//   // configure using Filter algorithm Class
//   Filter_.Configure(num, den, dt_);
// }
//
// void FDIPEClass::Initialize() {}
// bool FDIPEClass::Initialized() {return true;}
//
// void FDIPEClass::Run(Mode mode) {
//   // sample time computation
//   float dt = 0;
//   if(UseFixedTimeSample == false) {
//     dt = time_node->getFloat();
//   } else {
//     dt = dt_;
//   }
//
//   float p_ref = p_ref_node->getFloat();
//
//   // inputs to SS vector
//   for (size_t i=0; i < u_node.size(); i++) {
//     u(i) = u_node[i]->getFloat();
//   }
//
//   // Run
//   SSClass_.Run(mode, u, dt, &y); // Call SS System
//   float p_exp = y(0);
//   float residual_raw = fabs(p_ref - p_exp); // residual is difference between measured and expected roll rates
//   float residual_filt = 0.0f;
//   Filter_.Run(mode, residual_raw, dt, &residual_filt); // filter the residual to reduce susceptibility to noise
//
//   // outputs to nodes
//   p_exp_node->setFloat(p_exp);
//   residual_raw_node->setFloat(residual_raw);
//   residual_filt_node->setFloat(residual_filt);
// }
//
// void FDIPEClass::Clear() {
//   UseFixedTimeSample = false;
//   A.resize(0,0);
//   B.resize(0,0);
//   C.resize(0,0);
//   D.resize(0,0);
//   InputKeys_.clear();
//   RollKey_.clear();
//   TimeKey_.clear();
//   SSClass_.Clear();
//   Filter_.Clear();
// }

void FDIPCAClass::Configure(const rapidjson::Value& Config, std::string SystemPath) {
  // I/O Signals
  LoadInput(Config, SystemPath, "Inputs", &u_node, &InputKeys_);

  LoadOutput(Config, SystemPath, "Tsq", &Tsq_node);
  LoadOutput(Config, SystemPath, "Qsq", &Qsq_node);

  // std::string TsqKeys = Config["Tsq"].GetString();
  // std::string QsqKeys = Config["Qsq"].GetString();

  // resize input vector
  int numU = u_node.size();
  u.resize(numU);
  u.setZero(numU);

  Tsq_raw_node = deftree.initElement(SystemPath + "/" + "Tsq_raw", "", LOG_FLOAT, LOG_NONE);
  Qsq_raw_node = deftree.initElement(SystemPath + "/" + "Qsq_raw", "", LOG_FLOAT, LOG_NONE);

  // Sample time (required)
  LoadVal(Config, "dt", &dt_);
  if (dt_ > 0.0) {
    UseFixedTimeSample = true;
  } else {
    LoadInput(Config, SystemPath, "Time-Source", &time_node, &TimeKey_);
  }

  LoadVal(Config, "invSig", &invSig, true);
  LoadVal(Config, "Upc", &Upc, true);
  LoadVal(Config, "Ures", &Ures, true);

  // Residual filter
  LoadVal(Config, "Num", &num, true);
  LoadVal(Config, "Den", &den, true);

  T_inner_ = Upc * invSig * Upc.transpose();
  Q_inner_ = Ures * Ures.transpose();

  Tsq_filter_.Configure(num, den, dt_);
  Qsq_filter_.Configure(num, den, dt_);
}

void FDIPCAClass::Initialize() {}
bool FDIPCAClass::Initialized() {return true;}

void FDIPCAClass::Run(Mode mode) {
  // sample time
  float dt = 0.0f;
  if(UseFixedTimeSample == false) {
    dt = time_node->getFloat();
  } else {
    dt = dt_;
  }

  // inputs to vector
  for (size_t i=0; i < u_node.size(); i++) {
    u(i) = u_node[i]->getFloat();
  }

  // Run
  Tsq_raw = u.transpose() * T_inner_ * u;
  Qsq_raw = u.transpose() * Q_inner_ * u;

  // filter
  Tsq_filter_.Run(mode, Tsq_raw, dt, &Tsq);
  Qsq_filter_.Run(mode, Qsq_raw, dt, &Qsq);

  // outputs to nodes
  Tsq_raw_node->setFloat(Tsq_raw);
  Qsq_raw_node->setFloat(Qsq_raw);

  Tsq_node->setFloat(Tsq);
  Qsq_node->setFloat(Qsq);
}

void FDIPCAClass::Clear() {
  UseFixedTimeSample = false;
  invSig.resize(0,0);

  InputKeys_.clear();
  TsqKey_.clear();
  QsqKey_.clear();
  TimeKey_.clear();
}
