/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#pragma once

#include "configuration.h"

/*
Generic Function Class
This is the base function for the Bolder Flight Systems
Flight Control System components. Airdata, filters,
allocators, and control laws are derived from this class.

The following methods are defined:
   * Configure: uses input JSON object to configure the function,
     root path is given to the higher level manager and group. i.e.
     if this is a control law function within the baseline group,
     the root path would be "/Control/Baseline". Finally a pointer
     to the global definition tree is given to grab pointers to inputs
     and register output variables.
   * Initialize: initializes the function prior to running.
   * Initialized: provides feedback on whether the function has completed
     initialization. Should be latched, so once initialization completed
     the first time, it stays initialized.
   * Run: runs the function with the given mode:
        * kEngage: the function is engaged and outputs are being used
        * kHold: the function is engaged, outputs are being used, but integrators
          should not be active (i.e. the aircraft is on the ground before launch)
        * kArm: the function is running, but not output. The function,
          should be ready to engage at any time and computing states for a
          transient free transition.
        * kStandby: the function is not running.
   * Clear: clears all data, states, and resources. Makes the function ready to
     be configured again.
*/

class GenericFunction {
  public:
    enum Mode {
      kStandby,
      kArm,
      kHold,
      kEngage
    };
    virtual void Configure(const rapidjson::Value& Config,std::string RootPath) {};
    virtual void Initialize() {};
    virtual bool Initialized() { return true; };
    virtual void Run(Mode mode) {};
    virtual void Clear() {};
};
