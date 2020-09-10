/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#pragma once

#include "configuration.h"
#include "generic-function.h"
#include "control-algorithms.h"
#include "filter-functions.h"

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
  "Min": X, "Max": X
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
    void Configure(const rapidjson::Value& Config, std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    float dt_ = 0.0f;
    bool UseFixedTimeSample = false;
    ElementPtr reference_node;
    ElementPtr feedback_node;
    ElementPtr time_node;
    float Min = std::numeric_limits<float>::lowest();
    float Max = std::numeric_limits<float>::max();

    ElementPtr output_node;

    __PID2Class PID2Class_;

    std::string OutputKey_, ReferenceKey_, FeedbackKey_, TimeKey_;
};

class PID2ClassExcite: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config, std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    float dt_ = 0.0f;
    bool UseFixedTimeSample = false;
    ElementPtr reference_node;
    ElementPtr feedback_node;
    ElementPtr time_node;
    float Min = std::numeric_limits<float>::lowest();
    float Max = std::numeric_limits<float>::max();

    ElementPtr output_node;
    ElementPtr ff_node;
    ElementPtr fb_node;

    __PID2ClassExcite PID2ClassExcite_;

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
  "Min": X, "Max": X
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
    void Configure(const rapidjson::Value& Config, std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    float dt_ = 0.0f;
    bool UseFixedTimeSample = false;
    float Min = std::numeric_limits<float>::lowest();
    float Max = std::numeric_limits<float>::max();

    ElementPtr input_node;
    ElementPtr time_node;
    ElementPtr output_node;

    __PID2Class PID2Class_;

    std::string OutputKey_, InputKey_, TimeKey_;
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
  "Min": [X], "Max": [X]
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
    void Configure(const rapidjson::Value& Config,std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:

    std::vector<ElementPtr> u_node;
    Eigen::VectorXf u;
    Eigen::VectorXf x;
    Eigen::VectorXf y;
    Eigen::MatrixXf A, B, C, D;
    Eigen::VectorXf Min, Max;

    float dt = 0;
    float* TimeSource = 0;
    float timePrev = 0;

    bool UseFixedTimeSample = false;
    ElementPtr time_node;

    std::vector<ElementPtr> y_node;

    __SSClass SSClass_;

    std::vector<std::string> InputKeys_, OutputKeys_;
    std::string TimeKey_;
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
    void Configure(const rapidjson::Value& Config,std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    ElementPtr ref_vel_node;
    ElementPtr ref_agl_node;
    ElementPtr vel_node;
    ElementPtr agl_node;
    ElementPtr error_total_node;
    ElementPtr error_diff_node;

    bool initFlag = false;
    float mass_kg = 0.0;
    float weight_bal = 1.0;
    float min_mps = 0.0;
    float max_mps;
    int8_t error_totalSat = 0;
    int8_t error_diffSat = 0;
};

class FDIPEClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:

    std::vector<ElementPtr> u_node;
    Eigen::VectorXf u;
    Eigen::VectorXf y;
    Eigen::MatrixXf A, B, C, D;
    Eigen::VectorXf Min, Max;

    float dt = 0;
    float* TimeSource = 0;
    float timePrev = 0;

    bool UseFixedTimeSample = false;
    ElementPtr time_node;

    __GeneralFilter Filter_;
    __SSClass SSClass_;

    std::vector<float> num;
    std::vector<float> den;

    ElementPtr p_exp_node;
    ElementPtr p_ref_node;
    ElementPtr residual_raw_node;
    ElementPtr residual_filt_node;

    std::string RollKey_;
    std::vector<std::string> InputKeys_, OutputKeys_;
    std::string TimeKey_;
};


class FDIROClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:

    std::vector<ElementPtr> u_node;
    Eigen::VectorXf u;
    Eigen::VectorXf y;
    Eigen::MatrixXf A, B, C, D;
    Eigen::VectorXf Min, Max;

    float dt = 0;
    float* TimeSource = 0;
    float timePrev = 0;

    bool UseFixedTimeSample = false;
    ElementPtr time_node;

    __SSClass SSClass_;

    ElementPtr residual_node;

    std::vector<std::string> InputKeys_, OutputKeys_;
    std::string TimeKey_;
};


class FDIPCAClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string SystemPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:

    std::vector<ElementPtr> u_node;
    Eigen::VectorXf u;
    Eigen::MatrixXf invSig, Upc, Ures;
    Eigen::MatrixXf T_inner_, Q_inner_;
    Eigen::VectorXf Tsq, Qsq;

    float dt = 0;
    float* TimeSource = 0;
    float timePrev = 0;

    bool UseFixedTimeSample = false;
    ElementPtr time_node;

    std::vector<ElementPtr> Tsq_node;
    std::vector<ElementPtr> Qsq_node;

    std::vector<std::string> InputKeys_, TsqKeys_, QsqKeys_;
    std::string TimeKey_;
};
