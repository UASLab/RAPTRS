/*
filter-functions.cc
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

#include "filter-functions.h"

void GeneralFilter::Configure(const rapidjson::Value& Config,std::string SystemName) {
  // I/O signals
  LoadInput(Config, SystemName, "Input", &u_node, &InputKey_);
  LoadOutput(Config, SystemName, "Output", &y_node);

  // Coefficient vectors
  LoadVal(Config, "num", &num, true);
  LoadVal(Config, "den", &den, true);

  // Configure filter
  filter_.Configure(num, den);
}

void GeneralFilter::Initialize() {}

bool GeneralFilter::Initialized() {
  return true;
}

void GeneralFilter::Run(Mode mode) {
  y_node->setFloat( filter_.Run(u_node->getFloat()) );
}

void GeneralFilter::Clear() {
  filter_.Clear();
  y_node->setFloat(0.0f);
  InputKey_.clear();
}
