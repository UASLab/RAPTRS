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
void PID2Class::Configure(const rapidjson::Value& Config,std::string RootPath) {
  float Kp = 1;
  float Ki = 0;
  float Kd = 0;
  float Tf = 0;
  float b = 1;
  float c = 1;
  float yMin = 0;
  float yMax = 0;
  std::string OutputName;
  std::string SystemName;

  if (Config.HasMember("Output")) {
    OutputName = Config["Output"].GetString();
    SystemName = RootPath;

    // pointer to log command data
    data_.output_node = deftree.initElement(RootPath + "/" + OutputName, "Control law output", LOG_FLOAT, LOG_NONE);
    data_.ff_node = deftree.initElement(RootPath + "/" + OutputName + "FF", "Control law output, feedforward component", LOG_FLOAT, LOG_NONE);
    data_.fb_node = deftree.initElement(RootPath + "/" + OutputName + "FB", "Control law output, feedback component", LOG_FLOAT, LOG_NONE);
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    config_.reference_node = deftree.getElement(ReferenceKey_, true);
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference not specified in configuration."));
  }

  if (Config.HasMember("Feedback")) {
    FeedbackKey_ = Config["Feedback"].GetString();
    config_.feedback_node = deftree.getElement(FeedbackKey_, true);
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Feedback not specified in configuration."));
  }

  if (Config.HasMember("dt")) {
    if (Config["dt"].IsString()) {
      dtKey_ = Config["dt"].GetString();
      config_.dt_node = deftree.getElement(dtKey_);
      if ( !config_.dt_node ) {
        throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time ")+dtKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseFixedTimeSample = true;
      config_.dt = Config["dt"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time not specified in configuration."));
  }

  if (Config.HasMember("Kp")) {
    Kp = Config["Kp"].GetFloat();
  }
  if (Config.HasMember("Ki")) {
    Ki = Config["Ki"].GetFloat();
  }
  if (Config.HasMember("Kd")) {
    Kd = Config["Kd"].GetFloat();
    if (Config.HasMember("Tf")) {
      Tf = Config["Tf"].GetFloat();
    }
  }

  if (Config.HasMember("b")) {
    b = Config["b"].GetFloat();
  }
  if (Config.HasMember("c")) {
    c = Config["c"].GetFloat();
  }

  if (Config.HasMember("Min")) {
    yMin = Config["Min"].GetFloat();
  }
  if (Config.HasMember("Max")) {
    yMax = Config["Max"].GetFloat();
  }

  // configure PID2 Class
  PID2Class_.Configure(Kp,Ki,Kd,Tf,b,c,yMin,yMax);
}

void PID2Class::Initialize() {}
bool PID2Class::Initialized() {return true;}

void PID2Class::Run(Mode mode) {
  // sample time
  if(!config_.UseFixedTimeSample) {
    config_.dt = config_.dt_node->getFloat();
  }

  // Run
  float y = 0.0f;
  float ff = 0.0f;
  float fb = 0.0f;
  PID2Class_.Run(mode,
                 config_.reference_node->getFloat(),
                 config_.feedback_node->getFloat(),
                 config_.dt, &y, &ff, &fb);
  data_.output_node->setFloat(y);
  data_.ff_node->setFloat(ff);
  data_.fb_node->setFloat(fb);
}

void PID2Class::Clear() {
  config_.UseFixedTimeSample = false;
  data_.mode_node->setInt(kStandby);
  data_.output_node->setFloat(0.0f);
  data_.ff_node->setFloat(0.0f);
  data_.fb_node->setFloat(0.0f);
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  PID2Class_.Clear();
}

/* PID class methods, see control-functions.hxx for more information */
void PIDClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
  float Kp = 1;
  float Ki = 0;
  float Kd = 0;
  float Tf = 0;
  float yMin = 0;
  float yMax = 0;
  std::string OutputName;
  std::string SystemName;

  if (Config.HasMember("Output")) {
    OutputName = Config["Output"].GetString();
    SystemName = RootPath;

    // pointer to log command data
    data_.output_node = deftree.initElement(RootPath + "/" + OutputName, "Control law output", LOG_FLOAT, LOG_NONE);
    data_.ff_node = deftree.initElement(RootPath + "/" + OutputName + "FF", "Control law output, feedforward component", LOG_FLOAT, LOG_NONE);
    data_.fb_node = deftree.initElement(RootPath + "/" + OutputName + "FB", "Control law output, feedback component", LOG_FLOAT, LOG_NONE);
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    config_.reference_node = deftree.getElement(ReferenceKey_, true);
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference not specified in configuration."));
  }


  // configure using PID2 algorithm Class
  PID2Class_.Configure(Kp,Ki,Kd,Tf,1.0,1.0,yMin,yMax);
}

void PIDClass::Initialize() {}
bool PIDClass::Initialized() {return true;}

void PIDClass::Run(Mode mode) {
  // sample time
  if(!config_.UseFixedTimeSample) {
    config_.dt = config_.dt_node->getFloat();
  }

  // Run
  float y = 0.0f;
  float ff = 0.0f;
  float fb = 0.0f;
  PID2Class_.Run(mode, config_.reference_node->getFloat(), 0.0, config_.dt, &y, &ff, &fb);
  data_.output_node->setFloat(y);
  data_.ff_node->setFloat(ff);
  data_.fb_node->setFloat(fb);
}

void PIDClass::Clear() {
  config_.UseFixedTimeSample = false;
  data_.output_node->setFloat(0.0f);
  data_.ff_node->setFloat(0.0f);
  data_.fb_node->setFloat(0.0f);
  ReferenceKey_.clear();
  PID2Class_.Clear();
}


/* SS class methods, see control-functions.hxx for more information */
void SSClass::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string OutputName;
  std::string SystemName;

  // grab sytem Name
  if (Config.HasMember("Name")) {
    SystemName = Config["Name"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Name not specified in configuration."));
  }

  // grab inputs
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      ElementPtr ele = deftree.getElement(InputKeys_.back());
      if (ele) {
        config_.Inputs.push_back(ele);
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Inputs not specified in configuration."));
  }

  // grab outputs
  if (Config.HasMember("Outputs")) {
    // resize output matrix
    data_.y.resize(Config["Outputs"].Size());
    data_.y_node.resize(Config["Outputs"].Size());
    for (size_t i=0; i < Config["Outputs"].Size(); i++) {
      const rapidjson::Value& y = Config["Outputs"][i];
      OutputName = y.GetString();

      // pointer to log output
      data_.y_node[i] = deftree.initElement(RootPath + SystemName + "/" + OutputName, "SS output", LOG_FLOAT, LOG_NONE);
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Outputs not specified in configuration."));
  }

  // grab A
  if (Config.HasMember("A")) {
    // resize A matrix
    config_.A.resize(Config["A"].Size(),Config["A"][0].Size());
    for (size_t m=0; m < Config["A"].Size(); m++) {
      for (size_t n=0; n < Config["A"][m].Size(); n++) {
        config_.A(m,n) = Config["A"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": A not specified in configuration."));
  }

  // resize state vector
  config_.x.resize(config_.A.rows());

  // grab B
  if (Config.HasMember("B")) {
    // resize B matrix
    config_.B.resize(Config["B"].Size(), Config["B"][0].Size());
    for (size_t m=0; m < Config["B"].Size(); m++) {
      for (size_t n=0; n < Config["B"][m].Size(); n++) {
        config_.B(m,n) = Config["B"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": B not specified in configuration."));
  }

  // resize input vector
  config_.u.resize(config_.B.cols());
  config_.u.setZero(config_.B.cols());

  // grab C
  if (Config.HasMember("C")) {
    // resize C matrix
    config_.C.resize(Config["C"].Size(),Config["C"][0].Size());
    for (size_t m=0; m < Config["C"].Size(); m++) {
      for (size_t n=0; n < Config["C"][m].Size(); n++) {
        config_.C(m,n) = Config["C"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": C not specified in configuration."));
  }

  // Resize output vector
  data_.y.resize(config_.C.rows());
  config_.yMin.resize(config_.C.rows());
  config_.yMax.resize(config_.C.rows());

  // grab D
  if (Config.HasMember("D")) {
    // resize D matrix
    config_.D.resize(Config["D"].Size(),Config["D"][0].Size());
    for (size_t m=0; m < Config["D"].Size(); m++) {
      for (size_t n=0; n < Config["D"][m].Size(); n++) {
        config_.D(m,n) = Config["D"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": D not specified in configuration."));
  }

  // grab stample time (required)
  if (Config.HasMember("dt")) {
    config_.dt = Config["dt"].GetFloat();
    config_.UseFixedTimeSample = true;
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": dt not specified in configuration."));
  }

  // grab time source input (optional)
  if (Config.HasMember("Time-Source")) {
    TimeSourceKey_ = Config["Time-Source"].GetString();
    config_.time_source_node = deftree.getElement(TimeSourceKey_);
    config_.UseFixedTimeSample = false;
    if ( !config_.time_source_node ) {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Time-Source ")+TimeSourceKey_+std::string(" not found in global data."));
    }
  }

  // grab limits
  if (Config.HasMember("Min")) {
    config_.yMin.resize(Config["Min"].Size());
    for (size_t i=0; i < Config["Min"].Size(); i++) {
      config_.yMin(i) = Config["Min"][i].GetFloat();
    }
  }

  if (Config.HasMember("Max")) {
    config_.yMax.resize(Config["Max"].Size());
    for (size_t i=0; i < Config["Max"].Size(); i++) {
      config_.yMax(i) = Config["Max"][i].GetFloat();
    }
  }

  // configure SS Class
  SSClass_.Configure(config_.A, config_.B, config_.C, config_.D, config_.dt, config_.yMin, config_.yMax);
}

void SSClass::Initialize() {}
bool SSClass::Initialized() {return true;}

void SSClass::Run(Mode mode) {
  // sample time computation
  float dt = 0;
  if (config_.UseFixedTimeSample == false) {
    dt = *config_.TimeSource - config_.timePrev;
    config_.timePrev = *config_.TimeSource;
    if (dt > 2*config_.dt) {dt = config_.dt;} // Catch large dt
    if (dt <= 0) {dt = config_.dt;} // Catch negative and zero dt
  } else {
    dt = config_.dt;
  }

  // inputs to Eigen3 vector
  for (size_t i=0; i < config_.Inputs.size(); i++) {
    config_.u(i) = config_.Inputs[i]->getFloat();
  }

  // Call Algorithm
  SSClass_.Run(mode, config_.u, dt, &data_.y);
  for (size_t i=0; i < data_.y_node.size(); i++) {
    data_.y_node[i]->setFloat(data_.y(i));
  }
}

void SSClass::Clear() {
  config_.UseFixedTimeSample = false;
  config_.A.resize(0,0);
  config_.B.resize(0,0);
  config_.C.resize(0,0);
  config_.D.resize(0,0);
  InputKeys_.clear();
  TimeSourceKey_.clear();
  SSClass_.Clear();
}

/* Tecs class methods, see control-functions.hxx for more information */
void TecsClass::Configure(const rapidjson::Value& Config,std::string RootPath) {

  std::string SystemName, OutputName;

  // grab sytem Name
  if (Config.HasMember("Name")) {
    SystemName = Config["Name"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Name not specified in configuration."));
  }

  if (Config.HasMember("mass_kg")) {
    mass_kg = Config["mass_kg"].GetFloat();
  }

  if (Config.HasMember("weight_bal")) {
    weight_bal = Config["weight_bal"].GetFloat();
  }

  if (Config.HasMember("max_mps")) {
    max_mps = Config["max_mps"].GetFloat();
  }

  if (Config.HasMember("min_mps")) {
    min_mps = Config["min_mps"].GetFloat();
  }

  if (Config.HasMember("RefSpeed")) {
    string RefSpeedKey = Config["RefSpeed"].GetString();
    ref_vel_node = deftree.getElement(RefSpeedKey);
    if ( !ref_vel_node ){
      throw std::runtime_error(std::string("ERROR")+std::string(": RefSpeed ")+RefSpeedKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": RefSpeed not specified in configuration."));
  }

  if (Config.HasMember("RefAltitude")) {
    string RefAltitudeKey = Config["RefAltitude"].GetString();
    ref_agl_node = deftree.getElement(RefAltitudeKey);
    if ( !ref_agl_node ) {
      throw std::runtime_error(std::string("ERROR")+std::string(": RefAltitude ")+RefAltitudeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": RefAltitude not specified in configuration."));
  }

  if (Config.HasMember("FeedbackSpeed")) {
    string FeedbackSpeedKey = Config["FeedbackSpeed"].GetString();
    vel_node = deftree.getElement(FeedbackSpeedKey);
    if ( !vel_node ) {
      throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackSpeed ")+FeedbackSpeedKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackSpeed not specified in configuration."));
  }

  if (Config.HasMember("FeedbackAltitude")) {
    string FeedbackAltitudeKey = Config["FeedbackAltitude"].GetString();
    agl_node = deftree.getElement(FeedbackAltitudeKey);
    if ( !agl_node ) {
      throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackAltitude ")+FeedbackAltitudeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackAltitude not specified in configuration."));
  }

  if (Config.HasMember("OutputTotal")) {
    OutputName = Config["OutputTotal"].GetString();

    // pointer to log output
    error_total_node = deftree.initElement(RootPath + "/" + SystemName + "/" + OutputName, "Tecs Total Energy Error", LOG_FLOAT, LOG_NONE);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": OutputTotal not specified in configuration."));
  }

  if (Config.HasMember("OutputDiff")) {
    OutputName = Config["OutputDiff"].GetString();

    // pointer to log output
    error_diff_node = deftree.initElement(RootPath + "/" + SystemName + "/" + OutputName, "Tecs Diff Energy Error ", LOG_FLOAT, LOG_NONE);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": OutputDiff not specified in configuration."));
  }

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
