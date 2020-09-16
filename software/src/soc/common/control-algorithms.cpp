/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#include "control-algorithms.h"

void __PID2Class::Configure(float Kp, float Ki, float Kd, float Tf, float b, float c, float Min, float Max) {
  Clear();  // Set Defaults

  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  Tf_ = Tf;
  b_ = b;
  c_ = c;
  Min_ = Min;
  Max_ = Max;
}

void __PID2Class::Run(GenericFunction::Mode mode, float Reference, float Feedback, float dt, float *y) {

  // error for proportional
  ProportionalError_ = b_ * Reference - Feedback;

  // error for integral
  IntegralError_ = Reference - Feedback;

  // error for derivative
  DerivativeError_ = c_ * Reference - Feedback;

  mode_ = mode;
  switch(mode_) {
    case GenericFunction::Mode::kStandby: {
      initLatch_ = false;
      break;
    }
    case GenericFunction::Mode::kArm: {
      Reset();
      ComputeDerivative(dt);
      InitializeState(0.0f);
      initLatch_ = true;
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kHold: {
      ComputeDerivative(dt);
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kEngage: {
      ComputeDerivative(dt);
      if (initLatch_ == false) {
        InitializeState(0.0f);
        initLatch_ = true;
      }
      UpdateState(dt);
      CalculateCommand();
      break;
    }
  }
  *y = y_;
}

void __PID2Class::InitializeState(float y) {
  // Protect for Ki == 0
  if (Ki_ != 0.0f) {
    // IntegralErrorState_ = (y - (Kp_*ProportionalError_ + Kd_*DerivativeErrorState_)) / Ki_;
    IntegralErrorState_ = (y - (Kp_*ProportionalError_)) / Ki_; // Ignore the derivative term to reduce noise on limits
  } else {
    IntegralErrorState_ = 0.0f;
  }
}

void __PID2Class::ComputeDerivative(float dt) {

  float ErrorChange = DerivativeError_ - PreviousDerivativeError_;

  if (Tf_ != 0) {
    DerivativeErrorState_ = (1 / Tf_) * ErrorChange + (1 - dt/Tf_) * PreviousDerivativeErrorState_;
  } else {
    DerivativeErrorState_ = 0.0f;
  }

  PreviousDerivativeError_ = DerivativeError_;
  PreviousDerivativeErrorState_ = DerivativeErrorState_;
}

void __PID2Class::UpdateState(float dt) {
  // Protect for unlimited windup when Ki == 0
  if (Ki_ != 0.0f) {
    IntegralErrorState_ += (dt * IntegralError_);
  } else {
    IntegralErrorState_ = 0.0;
  }
}

void __PID2Class::CalculateCommand() {
  y_ = Kp_*ProportionalError_ + Ki_*IntegralErrorState_ + Kd_*DerivativeErrorState_;

  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (y_ <= Min_) {
    y_ = Min_;
    // Re-compute the integrator state
    InitializeState(y_);
  } else if (y_ >= Max_) {
    y_ = Max_;
    // Re-compute the integrator state
    InitializeState(y_);
  }
}

void __PID2Class::Reset() {
  // Reset internal values
  ProportionalError_ = 0.0f;
  DerivativeError_ = 0.0f;
  IntegralError_ = 0.0f;
  DerivativeErrorState_ = 0.0f;

  // Reset States
  PreviousDerivativeError_ = 0.0f;
  IntegralErrorState_ = 0.0f;

  // reset mode
  mode_ = GenericFunction::Mode::kStandby;
  initLatch_ = false;
}

void __PID2Class::Clear() {
  Reset();

  Kp_ = 1.0f;
  Ki_ = 0.0f;
  Kd_ = 0.0f;
  Tf_ = 0.0f;
  b_ = 1.0f;
  c_ = 1.0f;
  Min_ = 0.0f;
  Max_ = 0.0f;

  y_ = 0.0f;
}

// PID2 with Excitation exposure
void __PID2ClassExcite::Configure(float Kp, float Ki, float Kd, float Tf, float b, float c, float Min, float Max) {
  Clear();  // Set Defaults

  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  Tf_ = Tf;
  b_ = b;
  c_ = c;
  Min_ = Min;
  Max_ = Max;
}

void __PID2ClassExcite::Run(GenericFunction::Mode mode, float Reference, float Feedback, float dt, float *y, float *ff, float *fb) {

  // error for proportional
  ffProportionalError_ = b_ * Reference;
  fbProportionalError_ = Feedback;

  // error for integral
  ffIntegralError_ = Reference;
  fbIntegralError_ = Feedback;

  // error for derivative
  ffDerivativeError_ = c_ * Reference;
  fbDerivativeError_ = Feedback;

  mode_ = mode;
  switch(mode_) {
    case GenericFunction::Mode::kStandby: {
      initLatch_ = false;
      break;
    }
    case GenericFunction::Mode::kArm: {
      Reset();
      ComputeDerivative(dt);
      InitializeState(0.0f, 0.0f);
      initLatch_ = true;
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kHold: {
      ComputeDerivative(dt);
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kEngage: {
      ComputeDerivative(dt);
      if (initLatch_ == false) {
        InitializeState(0.0f, 0.0f);
        initLatch_ = true;
      }
      UpdateState(dt);
      CalculateCommand();
      break;
    }
  }
  *y = y_;
  *ff = ff_;
  *fb = fb_;
}

void __PID2ClassExcite::InitializeState(float ff, float fb) {
  // Protect for Ki == 0
  if (Ki_ != 0.0f) {
    // IntegralErrorState_ = (y - (Kp_*ProportionalError_ + Kd_*DerivativeErrorState_)) / Ki_;
    // IntegralErrorState_ = (y - (Kp_*ProportionalError_)) / Ki_; // Ignore the derivative term to reduce noise on limits
    ffIntegralErrorState_ = (ff - (Kp_*ffProportionalError_)) / Ki_;
    fbIntegralErrorState_ = (fb - (Kp_*fbProportionalError_)) / Ki_;
  } else {
    ffIntegralErrorState_ = 0.0f;
    fbIntegralErrorState_ = 0.0f;
  }
}

void __PID2ClassExcite::ComputeDerivative(float dt) {

  float ffErrorChange = ffDerivativeError_ - ffPreviousDerivativeError_;
  float fbErrorChange = fbDerivativeError_ - fbPreviousDerivativeError_;

  if (Tf_ != 0) {
    ffDerivativeErrorState_ = (1 / Tf_) * ffErrorChange + (1 - dt/Tf_) * ffPreviousDerivativeErrorState_;
    fbDerivativeErrorState_ = (1 / Tf_) * fbErrorChange + (1 - dt/Tf_) * fbPreviousDerivativeErrorState_;
  } else {
    ffDerivativeErrorState_ = 0.0f;
    fbDerivativeErrorState_ = 0.0f;
  }

  ffPreviousDerivativeError_ = ffDerivativeError_;
  ffPreviousDerivativeErrorState_ = ffDerivativeErrorState_;

  fbPreviousDerivativeError_ = fbDerivativeError_;
  fbPreviousDerivativeErrorState_ = fbDerivativeErrorState_;
}

void __PID2ClassExcite::UpdateState(float dt) {
  // Protect for unlimited windup when Ki == 0
  if (Ki_ != 0.0f) {
    ffIntegralErrorState_ += (dt*ffIntegralError_);
    fbIntegralErrorState_ += (dt*fbIntegralError_);
  } else {
    ffIntegralErrorState_ = 0.0;
    fbIntegralErrorState_ = 0.0;
  }
}

void __PID2ClassExcite::CalculateCommand() {
  ff_ = Kp_*ffProportionalError_ + Ki_*ffIntegralErrorState_ + Kd_*ffDerivativeErrorState_;
  fb_ = Kp_*fbProportionalError_ + Ki_*fbIntegralErrorState_ + Kd_*fbDerivativeErrorState_;

  y_ = ff_ - fb_;

  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (y_ <= Min_) {
    y_ = Min_;
    ff_ = y_ + fb_;
    // Re-compute the integrator state
    // InitializeState(y_);
    InitializeState(ff_, fb_);
  } else if (y_ >= Max_) {
    y_ = Max_;
    ff_ = y_ + fb_;
    // Re-compute the integrator state
    // InitializeState(y_);
    InitializeState(ff_, fb_);
  }
}

void __PID2ClassExcite::Reset() {
  // Reset internal values
  ffProportionalError_ = 0.0f;
  fbProportionalError_ = 0.0f;
  ffDerivativeError_ = 0.0f;
  fbDerivativeError_ = 0.0f;
  ffIntegralError_ = 0.0f;
  fbIntegralError_ = 0.0f;
  ffDerivativeErrorState_ = 0.0f;
  fbDerivativeErrorState_ = 0.0f;

  // Reset States
  ffPreviousDerivativeError_ = 0.0f;
  fbPreviousDerivativeError_ = 0.0f;
  ffIntegralErrorState_ = 0.0f;
  fbIntegralErrorState_ = 0.0f;

  // reset mode
  mode_ = GenericFunction::Mode::kStandby;
  initLatch_ = false;
}

void __PID2ClassExcite::Clear() {
  Reset();

  Kp_ = 1.0f;
  Ki_ = 0.0f;
  Kd_ = 0.0f;
  Tf_ = 0.0f;
  b_ = 1.0f;
  c_ = 1.0f;
  Min_ = 0.0f;
  Max_ = 0.0f;

  y_ = 0.0f;
}


/* State Space */
void __SSClass::Configure(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, float dt, Eigen::VectorXf Min, Eigen::VectorXf Max, uint8_t LatchInit) {
  Clear(); // Clear and set to defaults

  A_ = A;
  B_ = B;
  C_ = C;
  D_ = D;
  Min_ = Min;
  Max_ = Max;
  LatchInit_ = LatchInit;

  uint8_t numU = D_.cols();
  uint8_t numX = A_.rows();
  uint8_t numY = D_.rows();

  uInit_.resize(numU);

  x_.resize(numX);
  y_.resize(numY);
  Max_.resize(numY);
  Min_.resize(numY);

  Reset(); // Initialize states and output

  // Compute the Inverse of C
  // Pseduo-Inverse using singular value decomposition
  Eigen::MatrixXf Ix = Eigen::MatrixXf::Identity(numX, numX);
  Eigen::MatrixXf Iy = Eigen::MatrixXf::Identity(numY, numY);
  CA_inv_ = (C_ * (A_*dt + Ix)).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Iy); // Jacobi SVD

  // Pre-compute C*B
  CB_ = C_*B_;
  CB_.resize(numY, numU);
}

void __SSClass::Run(GenericFunction::Mode mode, Eigen::VectorXf u, float dt, Eigen::VectorXf *y) {
  mode_ = mode;

  switch(mode_) {
    case GenericFunction::Mode::kStandby: {
      initLatch_ = false;
      break;
    }
    case GenericFunction::Mode::kArm: {
      Reset();
      InitializeState(u, *y, dt);
      initLatch_ = true;
      UpdateState(u, dt);
      OutputEquation(u, dt);
      break;
    }
    case GenericFunction::Mode::kHold: {
      OutputEquation(u, dt);
      break;
    }
    case GenericFunction::Mode::kEngage: {
      if (initLatch_ == false) {
        InitializeState(u, *y, dt);
        initLatch_ = true;
      }
      UpdateState(u, dt);
      OutputEquation(u, dt);
      break;
    }
  }
  *y = y_;
}

void __SSClass::GetInputInit(Eigen::VectorXf *uInit) {
  *uInit = uInit_;
}

void __SSClass::InitializeState(Eigen::VectorXf u, Eigen::VectorXf y, float dt) {
  // Initialize the input biases
  if (LatchInit_ == true) {
    uInit_ = u;
  }

  x_.Zero(A_.rows()); // Reset to Zero
  // x_ = CA_inv_ * (y - (CB_*dt + D_) * u);
}

void __SSClass::UpdateState(Eigen::VectorXf u, float dt) {
  uint8_t numX = A_.rows();
  Eigen::MatrixXf Ix = Eigen::MatrixXf::Identity(numX, numX);
  // x_ = A_ * x_ + B_ * u; // A,B,C,D defined as Discrete already
  x_ = (A_ * dt + Ix) * x_ + B_ * (u - uInit_) * dt; // A,B,B,D defined as Continuous
}

void __SSClass::OutputEquation(Eigen::VectorXf u, float dt) {
  y_ = C_*x_ + D_*(u - uInit_);

  // saturate output
  for (int i=0; i < y_.size(); i++) {
    if (y_(i) <= Min_(i)) {
      y_(i) = Min_(i);
    } else if (y_(i) >= Max_(i)) {
      y_(i) = Max_(i);
    }
  }

  //InitializeState(u, y_, dt); // Re-initialize the states with saturated outputs
}

void __SSClass::Reset() {
  mode_ = GenericFunction::Mode::kStandby;
  initLatch_ = false;
  uInit_.Zero(D_.cols());

  x_.Zero(A_.rows()); // Reset to Zero
  y_.Zero(C_.cols()); // Reset to Zero
}

void __SSClass::Clear() {
  A_.resize(0,0);
  B_.resize(0,0);
  C_.resize(0,0);
  D_.resize(0,0);

  uInit_.resize(0);
  Max_.resize(0);
  Min_.resize(0);

  CA_inv_.resize(0,0);
  CB_.resize(0,0);

  Reset();
}
