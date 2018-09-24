
#ifndef CONTROL_ALGORITHMS_HXX_
#define CONTROL_ALGORITHMS_HXX_

#include "generic-function.hxx"
#include <Eigen/Dense>

class __PID2Class {
  public:
    void Configure(float Kp, float Ki, float Kd, float Tf, float b, float c, bool SatFlag, float OutMax, float OutMin);
    void Run(GenericFunction::Mode mode, float Reference, float Feedback, float dt, float *Output, int8_t *Saturated);
    void Clear();
  private:
    uint8_t mode_ = GenericFunction::Mode::kStandby;

    float Kp_, Ki_, Kd_, Tf_, b_, c_, Output_, OutMax_, OutMin_;
    bool SatFlag_;
    bool initLatch_ = false;

    float ProportionalError_, DerivativeError_, IntegralError_, DerivativeErrorState_;
    float PreviousDerivativeError_, IntegralErrorState_;

    int8_t Saturated_;

    void InitializeState(float Command);
    void ComputeDerivative(float dt);
    void UpdateState(float dt);
    void CalculateCommand();
    void Reset();
};

class __SSClass {
  public:
    void Configure(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, float dt, bool satFlag, Eigen::VectorXf yMax, Eigen::VectorXf yMin);
    void Run(GenericFunction::Mode mode, Eigen::VectorXf u, float dt, Eigen::VectorXf *y, Eigen::VectorXi *ySat_);
    void Clear();
  private:
    uint8_t mode_ = GenericFunction::Mode::kStandby;

    Eigen::MatrixXf A_, B_, C_, D_;
    Eigen::VectorXf x_;
    Eigen::VectorXf y_, yMin_, yMax_;
    Eigen::VectorXi ySat_;

    bool SatFlag_;
    bool initLatch_ = false;

    Eigen::MatrixXf CA_inv_, CB_;

    void InitializeState(Eigen::VectorXf u, float dt);
    void UpdateState(Eigen::VectorXf u, float dt);
    void OutputEquation(Eigen::VectorXf u, float dt);
    void Reset();
};

#endif
