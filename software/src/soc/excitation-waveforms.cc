/*
excitation-waveforms.cc
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

#include "excitation-waveforms.hxx"

void Pulse::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Pulse::Initialize() {}
bool Pulse::Initialized() {return true;}

void Pulse::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us-Time0_us)/1e6 - config_.StartTime_s;

    // pulse logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the pulse to the signal
      data_.Excitation = config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }

  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Pulse::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Doublet::Initialize() {}
bool Doublet::Initialized() {return true;}

void Doublet::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (ExciteTime_s < 2.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }

  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet121::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Doublet121::Initialize() {}
bool Doublet121::Initialized() {return true;}

void Doublet121::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic, 1-2-1
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (ExciteTime_s < 3.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else if (ExciteTime_s < 4.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }

  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet121::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet3211::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Doublet3211::Initialize() {}
bool Doublet3211::Initialized() {return true;}

void Doublet3211::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic, 3-2-1-1
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < (3.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (ExciteTime_s < (5.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else if (ExciteTime_s < (6.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (ExciteTime_s < (7.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }

  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet3211::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void LinearChirp::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    if (Config["Amplitude"].Size() != 2) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude size incorrect; should be two [min,max]"));
    } else {
      config_.Amplitude[0] = Config["Amplitude"][0].GetFloat();
      config_.Amplitude[1] = Config["Amplitude"][1].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
  if (Config.HasMember("Frequency")) {
    if (Config["Frequency"].Size() != 2) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency size incorrect; should be two [min,max]"));
    } else {
      config_.Frequency[0] = Config["Frequency"][0].GetFloat();
      config_.Frequency[1] = Config["Frequency"][1].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency not specified in configuration."));
  }
}

void LinearChirp::Initialize() {}
bool LinearChirp::Initialized() {return true;}

void LinearChirp::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us-Time0_us)/1e6 - config_.StartTime_s;


    // chirp logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < config_.Duration_s) {
      // linear varying instantanious frequency
      float freq_rps = config_.Frequency[0]+(config_.Frequency[1]-config_.Frequency[0])*ExciteTime_s / (2.0f*config_.Duration_s);
      // linear varying amplitude
      float amp_nd = config_.Amplitude[0]+(config_.Amplitude[1]-config_.Amplitude[0])*ExciteTime_s / (config_.Duration_s);
      // chirp Equation
      data_.Excitation = amp_nd*sinf(freq_rps*ExciteTime_s);
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }

  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void LinearChirp::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude[0] = 0.0f;
  config_.Amplitude[1] = 0.0f;
  config_.Frequency[0] = 0.0f;
  config_.Frequency[1] = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void MultiSine::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName,&data_.Excitation,"Excitation system output",true,false);

    if (DefinitionTreePtr->GetValuePtr<float*>(SignalName)) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(SignalName);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
      config_.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time ")+Config["Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude.resize(Config["Amplitude"].Size(),1);
    for (size_t i=0; i < Config["Amplitude"].Size(); i++) {
      config_.Amplitude(i,0) = Config["Amplitude"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
  if (Config.HasMember("Frequency")) {
    config_.Frequency.resize(Config["Frequency"].Size(),1);
    for (size_t i=0; i < Config["Frequency"].Size(); i++) {
      config_.Frequency(i,0) = Config["Frequency"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency not specified in configuration."));
  }
  if (Config.HasMember("Phase")) {
    config_.Phase.resize(Config["Phase"].Size(),1);
    for (size_t i=0; i < Config["Phase"].Size(); i++) {
      config_.Phase(i,0) = Config["Phase"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Phase not specified in configuration."));
  }
  if ((Config["Amplitude"].Size() != Config["Frequency"].Size())||(Config["Amplitude"].Size() != Config["Phase"].Size())) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude, frequency, and phase arrays are not the same length."));
  }
}

void MultiSine::Initialize() {}
bool MultiSine::Initialized() {return true;}

void MultiSine::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = *config_.Time_us;
      TimeLatch = true;
    }
    ExciteTime_s = (float)(*config_.Time_us - Time0_us)/1e6 - config_.StartTime_s;

    // multisine logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.Excitation = 0;
    } else if (ExciteTime_s < config_.Duration_s) {
      // Scale the waveform to preserve unity
      float scale = sqrtf(0.5f/((float)config_.Amplitude.size()));
      // Compute the Waveform - scale * sum(amp .* cos(freq * t + phase))
      data_.Excitation=scale*(config_.Amplitude*(config_.Frequency*ExciteTime_s + config_.Phase).cos()).sum();
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Excitation = data_.Excitation * config_.Scale;
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void MultiSine::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.Scale = 1.0f;
  config_.Amplitude.resize(0,1);
  config_.Frequency.resize(0,1);
  config_.Phase.resize(0,1);
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.Excitation = 0.0f;
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}
