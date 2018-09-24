/*
filter-algorithms.hxx
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

#ifndef FILTER_ALGORITHMS_HXX_
#define FILTER_ALGORITHMS_HXX_

#include <algorithm>
#include <vector>

/*
General Filter - Implements a general discrete time filter using the
general filter difference equation. Matches the MATLAB filter function.
*/
class __GeneralFilter {
  public:
    void Configure(std::vector<float> b,std::vector<float> a);
    float Run(float input);
    void Clear();
  private:
    struct Config {
      std::vector<float> b;
      std::vector<float> a;
    };
    struct Data {
      float Output = 0.0f;
    };
    struct States {
      std::vector<float> x;
      std::vector<float> y;
    };
    Config config_;
    Data data_;
    States states_;
};

#endif
