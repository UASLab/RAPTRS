/*
generic-function.hxx
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

#ifndef GENERIC_FUNCTION_HXX_
#define GENERIC_FUNCTION_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree.hxx"

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
    virtual void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    virtual void Initialize();
    virtual bool Initialized();
    virtual void Run(Mode mode);
    virtual void Clear(DefinitionTree *DefinitionTreePtr);
};

#endif
