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

#include "control-functions.hxx"

/* PID2 class methods, see control-functions.hxx for more information */
void PID2Class::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  float Kp = 1;
  float Ki = 0;
  float Kd = 0;
  float b = 1;
  float c = 1;
  float Tf = 0;
  float UpperLimit = 0;
  float LowerLimit = 0;
  bool SaturateOutput = false;
  std::string OutputName;
  std::string SystemName;

  if (Config.HasMember("Output")) {
    OutputName = Config["Output"].GetString();
    SystemName = RootPath;

    // pointer to log run mode data
    DefinitionTreePtr->InitMember(RootPath + "/Mode", &data_.Mode, "Run mode", true, false);

    // pointer to log command data
    DefinitionTreePtr->InitMember(RootPath + "/" + OutputName, &data_.Output, "Control law output", true, false);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_)) {
      config_.Reference = DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference ")+ReferenceKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference not specified in configuration."));
  }

  if (Config.HasMember("Feedback")) {
    FeedbackKey_ = Config["Feedback"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_)) {
      config_.Feedback = DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Feedback ")+FeedbackKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Feedback not specified in configuration."));
  }

  if (Config.HasMember("Gains")) {
    const rapidjson::Value& Gains = Config["Gains"];
    if (Gains.HasMember("Proportional")) {
      Kp = Gains["Proportional"].GetFloat();
    }
    if (Gains.HasMember("Derivative")) {
      Kd = Gains["Derivative"].GetFloat();

      if (Config.HasMember("Time-Constant")) {
        Tf = Config["Time-Constant"].GetFloat();
      }
    }
    if (Gains.HasMember("Integral")) {
      Ki = Gains["Integral"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Gains not specified in configuration."));
  }

  if (Config.HasMember("Sample-Time")) {
    if (Config["Sample-Time"].IsString()) {
      SampleTimeKey_ = Config["Sample-Time"].GetString();
      if (DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_)) {
        config_.dt = DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_);
      } else {
        throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time ")+SampleTimeKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseFixedTimeSample = true;
      config_.SampleTime = Config["Sample-Time"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time not specified in configuration."));
  }

  if (Config.HasMember("Setpoint-Weights")) {
    const rapidjson::Value& Weights = Config["Setpoint-Weights"];
    if (Weights.HasMember("Proportional")) {
      b = Weights["Proportional"].GetFloat();
    }
    if (Weights.HasMember("Derivative")) {
      c = Weights["Derivative"].GetFloat();
    }
  }

  if (Config.HasMember("Limits")) {
    SaturateOutput = true;
    // pointer to log saturation data
    DefinitionTreePtr->InitMember(SystemName + "/Saturated", &data_.Saturated, "Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", true, false);

    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      UpperLimit = Config["Limits"]["Upper"].GetFloat();
      LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // configure PID2 Class
  PID2Class_.Configure(Kp,Ki,Kd,b,c,Tf,SaturateOutput,UpperLimit,LowerLimit);
}

void PID2Class::Initialize() {}
bool PID2Class::Initialized() {return true;}

void PID2Class::Run(Mode mode) {
  // mode
  data_.Mode = (uint8_t) mode;

  // sample time
  if(!config_.UseFixedTimeSample) {
    config_.SampleTime = *config_.dt;
  }

  // Run
  PID2Class_.Run(mode,*config_.Reference,*config_.Feedback,config_.SampleTime,&data_.Output,&data_.Saturated);
}

void PID2Class::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseFixedTimeSample = false;
  data_.Mode = kStandby;
  data_.Saturated = 0;
  data_.Output = 0.0f;
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  PID2Class_.Clear();
}

/* PID class methods, see control-functions.hxx for more information */
void PIDClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  float Kp = 1;
  float Ki = 0;
  float Kd = 0;
  float Tf = 0;
  float UpperLimit = 0;
  float LowerLimit = 0;
  bool SaturateOutput = false;
  std::string OutputName;
  std::string SystemName;

  if (Config.HasMember("Output")) {
    OutputName = Config["Output"].GetString();
    SystemName = RootPath;

    // pointer to log run mode data
    DefinitionTreePtr->InitMember(RootPath + "/Mode", &data_.Mode, "Run mode", true, false);

    // pointer to log command data
    DefinitionTreePtr->InitMember(RootPath + "/" + OutputName, &data_.Output, "Control law output", true, false);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_)) {
      config_.Reference = DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference ")+ReferenceKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Reference not specified in configuration."));
  }

  if (Config.HasMember("Gains")) {
    const rapidjson::Value& Gains = Config["Gains"];
    if (Gains.HasMember("Proportional")) {
      Kp = Gains["Proportional"].GetFloat();
    }
    if (Gains.HasMember("Derivative")) {
      Kd = Gains["Derivative"].GetFloat();

      if (Config.HasMember("Time-Constant")) {
        Tf = Config["Time-Constant"].GetFloat();
      }
    }
    if (Gains.HasMember("Integral")) {
      Ki = Gains["Integral"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Gains not specified in configuration."));
  }

  if (Config.HasMember("Sample-Time")) {
    if (Config["Sample-Time"].IsString()) {
      SampleTimeKey_ = Config["Sample-Time"].GetString();
      if (DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_)) {
        config_.dt = DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_);
      } else {
        throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time ")+SampleTimeKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseFixedTimeSample = true;
      config_.SampleTime = Config["Sample-Time"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Sample time not specified in configuration."));
  }

  if (Config.HasMember("Limits")) {
    SaturateOutput = true;
    // pointer to log saturation data
    DefinitionTreePtr->InitMember(SystemName + "/Saturated", &data_.Saturated, "Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", true, false);

    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      UpperLimit = Config["Limits"]["Upper"].GetFloat();
      LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // configure using PID2 algorithm Class
  PID2Class_.Configure(Kp,Ki,Kd,1.0,1.0,Tf,SaturateOutput,UpperLimit,LowerLimit);
}

void PIDClass::Initialize() {}
bool PIDClass::Initialized() {return true;}

void PIDClass::Run(Mode mode) {
  // mode
  data_.Mode = (uint8_t) mode;

  // sample time
  if(!config_.UseFixedTimeSample) {
    config_.SampleTime = *config_.dt;
  }

  // Run
  PID2Class_.Run(mode,*config_.Reference, 0.0, config_.SampleTime, &data_.Output, &data_.Saturated);
}

void PIDClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseFixedTimeSample = false;
  data_.Mode = kStandby;
  data_.Saturated = 0;
  data_.Output = 0.0f;
  ReferenceKey_.clear();
  PID2Class_.Clear();
}


/* SS class methods, see control-functions.hxx for more information */
void SSClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  float mass_kg;
  float weight_bal;
  float max_mps;
  float min_mps;
  bool SatFlag = false;
  std::string OutputName;
  std::string SystemName;


  // grab sytem Name
  if (Config.HasMember("Name")) {
    SystemName = Config["Name"].GetString();

    // pointer to log run mode data
    DefinitionTreePtr->InitMember(RootPath + SystemName + "/Mode", &data_.Mode, "Run mode", true, false);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Name not specified in configuration."));
  }

  // grab inputs
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back())) {
        config_.Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back()));
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
    data_.ySat.resize(Config["Outputs"].Size());
    for (size_t i=0; i < Config["Outputs"].Size(); i++) {
      const rapidjson::Value& Output = Config["Outputs"][i];
      OutputName = Output.GetString();

      // pointer to log output
      DefinitionTreePtr->InitMember(RootPath + SystemName + "/" + OutputName, &data_.y(i), "SS output", true, false);

      // pointer to log saturation data
      DefinitionTreePtr->InitMember(RootPath + SystemName + "/Saturated" + "/" + OutputName, &data_.ySat(i), "Output saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit", true, false);

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
    if (DefinitionTreePtr->GetValuePtr<float*>(TimeSourceKey_)) {
      config_.TimeSource = DefinitionTreePtr->GetValuePtr<float*>(TimeSourceKey_);
      config_.UseFixedTimeSample = false;
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Time-Source ")+TimeSourceKey_+std::string(" not found in global data."));
    }
  }

  // grab limits
  if (Config.HasMember("Limits")) {
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      SatFlag = true;

      // resize limit vectors
      config_.yMin.resize(Config["Limits"]["Lower"].Size());
      config_.yMax.resize(Config["Limits"]["Upper"].Size());
      for (size_t i=0; i < Config["Limits"]["Lower"].Size(); i++) {
        config_.yMin(i) = Config["Limits"]["Lower"][i].GetFloat();
      }
      for (size_t i=0; i < Config["Limits"]["Upper"].Size(); i++) {
        config_.yMax(i) = Config["Limits"]["Upper"][i].GetFloat();
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+SystemName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // configure SS Class
  data_.y.resize(config_.C.rows());
  data_.ySat.resize(config_.C.rows());
  SSClass_.Configure(config_.A, config_.B, config_.C, config_.D, config_.dt, SatFlag, config_.yMax, config_.yMin);
}

void SSClass::Initialize() {}
bool SSClass::Initialized() {return true;}

void SSClass::Run(Mode mode) {
  // mode
  data_.Mode = (uint8_t) mode;

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
    config_.u(i) = *config_.Inputs[i];
  }

  // Call Algorithm
  SSClass_.Run(mode, config_.u, dt, &data_.y, &data_.ySat);
}

void SSClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseFixedTimeSample = false;
  data_.Mode = kStandby;
  config_.A.resize(0,0);
  config_.B.resize(0,0);
  config_.C.resize(0,0);
  config_.D.resize(0,0);
  InputKeys_.clear();
  TimeSourceKey_.clear();
  SSClass_.Clear();
}

/* Tecs class methods, see control-functions.hxx for more information */
void TecsClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {

  std::string SystemName, OutputName;

  // grab sytem Name
  if (Config.HasMember("Name")) {
    SystemName = Config["Name"].GetString();

    // pointer to log run mode data
    DefinitionTreePtr->InitMember(RootPath + "/" + SystemName + "/Mode", &mode, "Run mode", true, false);

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
    std::string RefSpeedKey = Config["RefSpeed"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(RefSpeedKey)) {
      ref_vel_mps = DefinitionTreePtr->GetValuePtr<float*>(RefSpeedKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+std::string(": RefSpeed ")+RefSpeedKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": RefSpeed not specified in configuration."));
  }

  if (Config.HasMember("RefAltitude")) {
    std::string RefAltitudeKey = Config["RefAltitude"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(RefAltitudeKey)) {
      ref_agl_m = DefinitionTreePtr->GetValuePtr<float*>(RefAltitudeKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+std::string(": RefAltitude ")+RefAltitudeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": RefAltitude not specified in configuration."));
  }

  if (Config.HasMember("FeedbackSpeed")) {
    std::string FeedbackSpeedKey = Config["FeedbackSpeed"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(FeedbackSpeedKey)) {
      vel_mps = DefinitionTreePtr->GetValuePtr<float*>(FeedbackSpeedKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackSpeed ")+FeedbackSpeedKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackSpeed not specified in configuration."));
  }

  if (Config.HasMember("FeedbackAltitude")) {
    std::string FeedbackAltitudeKey = Config["FeedbackAltitude"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(FeedbackAltitudeKey)) {
      agl_m = DefinitionTreePtr->GetValuePtr<float*>(FeedbackAltitudeKey);
    } else {
      throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackAltitude ")+FeedbackAltitudeKey+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": FeedbackAltitude not specified in configuration."));
  }

  if (Config.HasMember("OutputTotal")) {
    OutputName = Config["OutputTotal"].GetString();

    // pointer to log output
    DefinitionTreePtr->InitMember(RootPath + "/" + SystemName + "/" + OutputName, &error_total, "Tecs Total Energy Error", true, false);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": OutputTotal not specified in configuration."));
  }

  if (Config.HasMember("OutputDiff")) {
    OutputName = Config["OutputDiff"].GetString();

    // pointer to log output
    DefinitionTreePtr->InitMember(RootPath + "/" + SystemName + "/" + OutputName, &error_diff, "Tecs Diff Energy Error ", true, false);

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
  float energy_pot = mass_kg * g * (*agl_m);
  float energy_kin = 0.5 * mass_kg * (*vel_mps) * (*vel_mps);

  // Reference energy
  float target_pot = mass_kg * g * (*ref_agl_m);
  float target_kin = 0.5 * mass_kg * (*ref_vel_mps) * (*ref_vel_mps);

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
  error_total = error_pot + error_kin;
  error_diff =  (2.0 - weight_bal) * error_kin - weight_bal * error_pot;

  // clamp error_diff to kinetic error range.  This prevents tecs from
  // requesting a pitch attitude that would over/under-speed the
  // aircraft.
  if ( error_diff < min_error ) { error_diff = min_error; }
  if ( error_diff > max_error ) { error_diff = max_error; }

  // clamp max total error to avoid an overspeed condition in a climb
  // if max pitch angle is saturated.
  if ( error_total > max_error ) { error_total = max_error; }
}

void TecsClass::Clear(DefinitionTree *DefinitionTreePtr) {
}
