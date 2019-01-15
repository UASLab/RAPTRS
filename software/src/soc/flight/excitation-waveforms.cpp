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

#include "excitation-waveforms.h"

void Pulse::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // pulse logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the pulse to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat(data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat(config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void Pulse::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else if (ExciteTime_s < 2.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(-1*config_.Amplitude);
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0);
  }

  data_.excitation_node->setFloat( data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat() );
}

void Doublet::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet121::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic, 1-2-1
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else if (ExciteTime_s < 3.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(-1*config_.Amplitude);
    } else if (ExciteTime_s < 4.0f*config_.Duration_s) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat( data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void Doublet121::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Doublet3211::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // doublet logic, 3-2-1-1
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < (3.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else if (ExciteTime_s < (5.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(-1*config_.Amplitude);
    } else if (ExciteTime_s < (6.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(config_.Amplitude);
    } else if (ExciteTime_s < (7.0f*config_.Duration_s)) {
      // add the doublet to the signal
      data_.excitation_node->setFloat(-1*config_.Amplitude);
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat(data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat(config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void Doublet3211::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void LinearChirp::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }

  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
  if (config_.StartTime_s <= 0.0) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time must be greater than 0."));
  }

  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (config_.Duration_s <= 0.0) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration must be greater than 0."));
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
  if ((config_.Frequency[0] <= 0.0) || (config_.Frequency[1] <= 0.0)) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequencies must be greater than 0."));
  }

  // Constant for linear varying frequency
  config_.FreqK = (config_.Frequency[1] - config_.Frequency[0]) / config_.Duration_s;

  // Constant for linear varying amplitude
  config_.AmpK = (config_.Amplitude[1] - config_.Amplitude[0]) / config_.Duration_s;
}

void LinearChirp::Initialize() {}
bool LinearChirp::Initialized() {return true;}

void LinearChirp::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;


    // chirp logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // linear varying instantanious frequency
      float freq_rps = config_.Frequency[0] + config_.FreqK * ExciteTime_s;
      // linear varying amplitude
      float amp_nd = config_.Amplitude[0] + config_.AmpK * ExciteTime_s;
      // chirp Equation, note the factor of 2.0 is correct!
      data_.excitation_node->setFloat(amp_nd * sinf((freq_rps / 2.0f) * ExciteTime_s));
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat(data_.excitation_node->getFloat() * config_.Scale);
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void LinearChirp::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude[0] = 0.0f;
  config_.Amplitude[1] = 0.0f;
  config_.Frequency[0] = 0.0f;
  config_.Frequency[1] = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  config_.FreqK = 0.0f;
  config_.AmpK = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void LogChirp::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }

  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
  if (config_.StartTime_s <= 0.0) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time must be greater than 0."));
  }

  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (config_.Duration_s <= 0.0) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration must be greater than 0."));
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
  if ((config_.Frequency[0] <= 0.0) || (config_.Frequency[1] <= 0.0)) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequencies must be greater than 0."));
  }

  // Constants for log varying frequency
  config_.FreqK = pow(config_.Frequency[1] / config_.Frequency[0], (1/config_.Duration_s));
  config_.FreqLogK = log(config_.FreqK);

  // Constants for linear varying amplitude
  config_.AmpK = (config_.Amplitude[1] - config_.Amplitude[0]) / config_.Duration_s;
}

void LogChirp::Initialize() {}
bool LogChirp::Initialized() {return true;}

void LogChirp::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // chirp logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // log varying instantaneous frequency
      float freq_rps = config_.Frequency[0] * (pow(config_.FreqK, ExciteTime_s) - 1) / (ExciteTime_s * config_.FreqLogK);
      // linear varying amplitude
      float amp_nd = config_.Amplitude[0] + config_.AmpK * ExciteTime_s;
      // chirp Equation
      data_.excitation_node->setFloat(amp_nd * sinf(freq_rps*ExciteTime_s));
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat(data_.excitation_node->getFloat() * config_.Scale);
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void LogChirp::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude[0] = 0.0f;
  config_.Amplitude[1] = 0.0f;
  config_.Frequency[0] = 0.0f;
  config_.Frequency[1] = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  config_.FreqK = 0.0f;
  config_.FreqLogK = 0.0f;
  config_.AmpK = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void Pulse_1_Cos::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
    config_.Frequency = (2 * M_PI) / config_.Duration_s;
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Pause")) {
    config_.Pause_s = Config["Pause"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Pause not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Pulse_1_Cos::Initialize() {}
bool Pulse_1_Cos::Initialized() {return true;}

void Pulse_1_Cos::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong()-Time0_us)/1e6 - config_.StartTime_s;

    // chirp logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < (config_.Duration_s + config_.Pause_s)) {
      // 1-Cos excitation_node->setFloat(
      if (ExciteTime_s < (0.5*config_.Duration_s)) {
        data_.excitation_node->setFloat( config_.Amplitude * 0.5 * (1 - cosf(config_.Frequency * ExciteTime_s)) );
      } else if (ExciteTime_s < (0.5*config_.Duration_s + config_.Pause_s)) {
        data_.excitation_node->setFloat(config_.Amplitude);
      } else {
        data_.excitation_node->setFloat( config_.Amplitude * 0.5 * (1 - cosf(config_.Frequency * (ExciteTime_s - config_.Pause_s))) );
      }
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }

  data_.excitation_node->setFloat( data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void Pulse_1_Cos::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude = 0.0f;
  config_.Frequency = 0.0f;
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  config_.Pause_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat( 0.0f );
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}

void MultiSine::Configure(const rapidjson::Value& Config,std::string RootPath) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Scale-Factor")) {
    config_.Scale = Config["Scale-Factor"].GetFloat();
  }
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));

    // pointer to log excitation data
    data_.excitation_node = deftree.initElement(OutputName, "Excitation system output", LOG_FLOAT, LOG_NONE);

    config_.signal_node = deftree.getElement(SignalName);
    if ( !config_.signal_node ) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal ")+SignalName+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    config_.time_node = deftree.getElement(Config["Time"].GetString());
    if ( !config_.time_node ) {
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
      Time0_us = config_.time_node->getLong();
      TimeLatch = true;
    }
    ExciteTime_s = (float)(config_.time_node->getLong() - Time0_us)/1e6 - config_.StartTime_s;

    // multisine logic
    if (ExciteTime_s < 0){
      // do nothing
      data_.excitation_node->setFloat(0.0);
    } else if (ExciteTime_s < config_.Duration_s) {
      // Scale the waveform to preserve unity
      float scale = sqrtf(0.5f/((float)config_.Amplitude.size()));
      // Compute the Waveform - scale * sum(amp .* cos(freq * t + phase))
      data_.excitation_node->setFloat( scale*(config_.Amplitude*(config_.Frequency*ExciteTime_s + config_.Phase).cos()).sum() );
    } else {
      // do nothing
      data_.excitation_node->setFloat(0.0);
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.excitation_node->setFloat(0.0);
  }
  data_.excitation_node->setFloat( data_.excitation_node->getFloat() * config_.Scale );
  data_.Mode = (uint8_t)mode;
  config_.signal_node->setFloat( config_.signal_node->getFloat() + data_.excitation_node->getFloat());
}

void MultiSine::Clear() {
  config_.Scale = 1.0f;
  config_.Amplitude.resize(0,1);
  config_.Frequency.resize(0,1);
  config_.Phase.resize(0,1);
  config_.StartTime_s = 0.0f;
  config_.Duration_s = 0.0f;
  data_.Mode = kStandby;
  data_.excitation_node->setFloat(0.0f);
  Time0_us = 0;
  ExciteTime_s = 0;
  TimeLatch = false;
}
