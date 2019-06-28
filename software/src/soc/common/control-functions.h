/*
control-functions.hxx
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

#ifndef CONTROL_FUNCTIONS_HXX_
#define CONTROL_FUNCTIONS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"

#include "configuration.h"
#include "generic-function.h"
#include "control-algorithms.h"

/* Control related functions. Each function describes its JSON
configuration below. See generic-function.hxx for more information
on the methods and modes. */

/*
PID2 Class - PID2 control law
Example JSON configuration:
{
  "Type": "PID2",
  "Reference": "ReferenceName",
  "Feedback": "FeedbackName",
  "Output": "OutputName",
  "dt": "dt" or X,
  "Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf,
  "b": b, "c": c
  "Max": X, "Min": X
}
Where:
   * Reference is the full path name of the reference signal.
   * Feedback is the full path name of the feedback signal.
   * Output gives a convenient name for the block (i.e. cmdPitch).
   * dt is either: the full path name of the sample time signal in seconds,
     or a fixed value sample time in seconds.
   * Tf is the time constant for the derivative filter.
     If a time constant is not specified, then no filtering is used.
   * Gains specifies the proportional, derivative, and integral gains.
   * Setpoint weights (b and c) optionally specifies the setpoint
     weights (proportional and derivative) used in the controller.
   * Limits (Max and Min) are optional.
Data types for all input and output values are float.
*/

class PID2Class: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      float dt;
      bool UseFixedTimeSample = false;
      ElementPtr reference_node;
      ElementPtr feedback_node;
      ElementPtr time_node;
      float Min = std::numeric_limits<float>::lowest();
      float Max = std::numeric_limits<float>::max();
    };
    struct Data {
      ElementPtr output_node;
      ElementPtr ff_node;
      ElementPtr fb_node;
    };
    __PID2Class PID2Class_;
    Config config_;
    Data data_;
    std::string OutputKey_, ReferenceKey_, FeedbackKey_, TimeKey_;
};

/*
PID Class - PID control law
Example JSON configuration:
{
  "Type": "PID",
  "Reference": "ReferenceName",
  "Output": "OutputName",
  "dt": "dt" or X,
  "Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf,
  "Max": X, "Min": X
}
Where:
   * Reference is the full path name of the reference signal.
   * Output gives a convenient name for the block (i.e. cmdPitch).
   * dt is either: the full path name of the sample time signal in seconds,
     or a fixed value sample time in seconds.
   * Tf is the time constant for the derivative filter.
     If a time constant is not specified, then no filtering is used.
   * Gains specifies the proportional, derivative, and integral gains.
   * Limits (Max and Min) are optional.
Data types for all input and output values are float.
*/

class PIDClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
        float dt;
        bool UseFixedTimeSample = false;
        ElementPtr reference_node;
        ElementPtr dt_node;
        float Min = std::numeric_limits<float>::lowest();
        float Max = std::numeric_limits<float>::max();
    };
    struct Data {
        ElementPtr output_node;
        ElementPtr ff_node;
        ElementPtr fb_node;
    };
    __PID2Class PID2Class_;
    Config config_;
    Data data_;
    std::string ReferenceKey_,dtKey_;
};

/*
SS Class - State Space
Example JSON configuration:
{
  "Type": "SS",
  "Inputs": ["InputNames"],
  "Outputs": ["OutputNames"],
  "dt": "dt" or X,
  "A": [[X]],
  "B": [[X]],
  "C": [[X]],
  "D": [[X]],
  "Max": [X],
  "Min": [X]
}
Where:
   * Inputs is the full path name of the input signals.
   * Outputs is the full path name of the output signals.
   * dt is either: the full path name of the sample time signal in seconds,
     or a fixed value sample time in seconds.
   * Limits (Max and Min) are optional.

Data types for all input and output values are float.

The implemented algorithm uses a discrete state space model, with variable dt.
The A and B matrices supplied are the continuous form, a simple zero-order hold is used to compute the discrete form.
xDot = A*x + B*u;
y = C*x + D*u;
  where:  Ad = (Ac*dt + I);
          Bd = B*dt;
Thus, x[k+1] = Ad*x + Bd*u;
*/

class SSClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      std::vector<ElementPtr> Inputs;
      Eigen::VectorXf u;
      Eigen::VectorXf x;
      Eigen::MatrixXf A;
      Eigen::MatrixXf B;
      Eigen::MatrixXf C;
      Eigen::MatrixXf D;
      Eigen::VectorXf Min;
      Eigen::VectorXf Max;
      float dt = 0;
      float* TimeSource = 0;
      float timePrev = 0;
      bool UseFixedTimeSample = false;
        ElementPtr time_source_node;
    };
    struct Data {
        Eigen::VectorXf y;
        vector<ElementPtr>y_node;
    };
    __SSClass SSClass_;
    Config config_;
    Data data_;
    std::vector<std::string> InputKeys_;
    std::string TimeSourceKey_;
};

/*
Tecs Class - Total Energy Control System
Example JSON configuration:
{
  "Type": "Tecs",
  "RefSpeed": "RefSpeed",
  "RefAltitude": "RefAltitude",
  "FeedbackSpeed": "FeedbackSpeed",
  "FeedbackAltitude": "FeedbackAltitude",
  "OutputTotal": "OutputTotal",
  "OutputDiff": "OutputDiff",
  "mass_kg": x,
  "weight_bal": x,
  "vMax_mps": x,
  "vMin_mps": x
}
Where:
   * mass_kg is the total aircraft weight in kg
   * weight_bal is a value = [0.0 - 2.0] with 1.0 being a good starting point.
     0.0 = elevator controls speed only, 2.0 = elevator controls altitude only
   * min_mps: the system will not command a pitch angle that causes the
     airspeed to drop below min_mps, even with zero throttle.
   * max_mps: the system will not command a combination of pitch and throttle
     that will cause the airspeed to exceed this value
   * In either case it is possible to momentarily bust these limits, but the
     system will always be driving the airspeed back within the specified limits

Data types for all input and output values are float.

*/

class TecsClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    //float *ref_vel_mps;
    //float *ref_agl_m;
    //float *vel_mps;
    //float *agl_m;
    ElementPtr ref_vel_node;
    ElementPtr ref_agl_node;
    ElementPtr vel_node;
    ElementPtr agl_node;
    //float error_total;
    //float error_diff;
    ElementPtr error_total_node;
    ElementPtr error_diff_node;

    bool initFlag = false;
    float mass_kg = 0.0;
    float weight_bal = 1.0;
    float min_mps = 0.0;
    float max_mps;
    //uint8_t mode = kStandby;
    int8_t error_totalSat = 0;
    int8_t error_diffSat = 0;
};
#endif
