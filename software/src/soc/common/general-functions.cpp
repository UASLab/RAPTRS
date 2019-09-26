/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "general-functions.h"
#include <stdio.h>
#include <iostream>

/* Constant class methods, see general-functions.h for more information */
void ConstantClass::Configure(const rapidjson::Value& Config,std::string SystemName) {

  LoadOutput(Config, SystemName, "Output", &Output_node_);
  LoadVal(Config, "Constant", &Val_);
}

void ConstantClass::Initialize() {}
bool ConstantClass::Initialized() {return true;}

void ConstantClass::Run(Mode mode) {
  Output_node_->setFloat(Val_);
}

void ConstantClass::Clear() {
  Val_ = 0.0f;
  Output_node_->setFloat(0.0f);
}

/* Gain class methods, see general-functions.h for more information */
void GainClass::Configure(const rapidjson::Value& Config,std::string SystemName) {
  LoadInput(Config, SystemName, "Input", &Input_node_, &InputKey_);
  LoadOutput(Config, SystemName, "Output", &Output_node_);
  LoadVal(Config, "Gain", &Gain_);
  LoadVal(Config, "Min", &Min_);
  LoadVal(Config, "Max", &Max_);
}

void GainClass::Initialize() {}
bool GainClass::Initialized() {return true;}

void GainClass::Run(Mode mode) {
  float Val = Input_node_->getFloat()*Gain_;

  // saturate command
  if (Val <= Min_) {
    Val = Min_;
  } else if (Val >= Max_) {
    Val = Max_;
  }

  Output_node_->setFloat( Val );
}

void GainClass::Clear() {
  Gain_ = 1.0f;
  Min_ = 0.0f;
  Max_ = 0.0f;
  Output_node_->setFloat(0.0f);
  InputKey_.clear();
}

/* Sum class methods, see general-functions.h for more information */
void SumClass::Configure(const rapidjson::Value& Config,std::string SystemName) {

  LoadInput(Config, SystemName, "Inputs", &Input_nodes_, &InputKeys_);
  LoadOutput(Config, SystemName, "Output", &Output_node_);
  LoadVal(Config, "Min", &Min_);
  LoadVal(Config, "Max", &Max_);
}

void SumClass::Initialize() {}
bool SumClass::Initialized() {return true;}

void SumClass::Run(Mode mode) {
  float Val = 0.0;
  for (size_t i=0; i < Input_nodes_.size(); i++) {
    Val += Input_nodes_[i]->getFloat();
  }

  // saturate command
  if (Val <= Min_) {
    Val = Min_;
  } else if (Val >= Max_) {
    Val = Max_;
  }

  Output_node_->setFloat(Val);
}

void SumClass::Clear() {
  Input_nodes_.clear();
  Min_ = 0.0f;
  Max_ = 0.0f;
  Output_node_->setFloat(0.0f);
  InputKeys_.clear();
}

/* Product class methods, see general-functions.h for more information */
void ProductClass::Configure(const rapidjson::Value& Config,std::string SystemName) {

  LoadInput(Config, SystemName, "Inputs", &Input_nodes_, &InputKeys_);
  LoadOutput(Config, SystemName, "Output", &Output_node_);
  LoadVal(Config, "Min", &Min_);
  LoadVal(Config, "Max", &Max_);
}

void ProductClass::Initialize() {}
bool ProductClass::Initialized() {return true;}

void ProductClass::Run(Mode mode) {
  float Val = 1.0;
  for (size_t i=0; i < Input_nodes_.size(); i++) {
    Val *= Input_nodes_[i]->getFloat();
  }

  // saturate command
  if (Val <= Min_) {
    Val = Min_;
  } else if (Val >= Max_) {
    Val = Max_;
  }

  Output_node_->setFloat(Val);
}

void ProductClass::Clear() {
  Input_nodes_.clear();
  Min_ = 0.0f;
  Max_ = 0.0f;
  Output_node_->setFloat(0.0f);
  InputKeys_.clear();
}


/* Delay class methods, see general-functions.h for more information */
void DelayClass::Configure(const rapidjson::Value& Config, std::string SystemName) {

  LoadInput(Config, SystemName, "Input", &Input_node_, &InputKey_);
  LoadOutput(Config, SystemName, "Output", &Output_node_);
  LoadVal(Config, "Delay_frames", &DelayFrames_);
}

void DelayClass::Initialize() {
  DelayBuffer_.clear();
}
bool DelayClass::Initialized() { return true; }

void DelayClass::Run(Mode mode) {
  DelayBuffer_.push_back( Input_node_->getFloat() );
  float Val = 0.0;
  while ( DelayBuffer_.size() > 0 && DelayBuffer_.size() > (unsigned int)DelayFrames_ ) {
    Val = DelayBuffer_.front();
    DelayBuffer_.pop_front();
  }
  Output_node_->setFloat(Val);
}

void DelayClass::Clear() {
  DelayFrames_ = 0;
  DelayBuffer_.clear();
  Output_node_->setFloat(0.0f);
  InputKey_.clear();
}


/* Latch class methods, see general-functions.h for more information */
void LatchClass::Configure(const rapidjson::Value& Config,std::string SystemName) {
  LoadInput(Config, SystemName, "Input", &Input_node_, &InputKey_);
  LoadOutput(Config, SystemName, "Output", &Output_node_);
}

void LatchClass::Initialize() {}
bool LatchClass::Initialized() {return true;}

void LatchClass::Run(Mode mode) {
  switch(mode) {
    case GenericFunction::Mode::kEngage: {
      if (InitLatch_ == false) {
        InitLatch_ = true;
        Output_node_->setFloat(Input_node_->getFloat());
      }
      break;
    }
    default: {
      InitLatch_ = false;
    }
  }
}

void LatchClass::Clear() {
  Output_node_->setFloat(0.0f);
  InputKey_.clear();
}
