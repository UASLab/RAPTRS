
#include "control-algorithms.h"

void __PID2Class::Configure(float Kp, float Ki, float Kd, float b, float c, float Tf, bool SatFlag, float OutMax, float OutMin) {
  Clear();  // Set Defaults

  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  Tf_ = Tf;
  b_ = b;
  c_ = c;
  SatFlag_ = SatFlag;
  OutMax_ = OutMax;
  OutMin_ = OutMin;
}

void __PID2Class::Run(GenericFunction::Mode mode,float Reference,float Feedback,float dt, float *Output, float *ff, float *fb, int8_t *Saturated) {

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
  *Output = Output_;
  *ff = ff_;
  *fb = fb_;
  *Saturated = Saturated_;
}

void __PID2Class::InitializeState(float ff, float fb) {
  // Protect for Ki == 0
  if (Ki_ != 0.0f) {
    // IntegralErrorState_ = (Output - (Kp_*ProportionalError_ + Kd_*DerivativeErrorState_)) / Ki_;
    // IntegralErrorState_ = (Output - (Kp_*ProportionalError_)) / Ki_; // Ignore the derivative term to reduce noise on limits
    ffIntegralErrorState_ = (ff - (Kp_*ffProportionalError_)) / Ki_;
    fbIntegralErrorState_ = (fb - (Kp_*fbProportionalError_)) / Ki_;
  } else {
    ffIntegralErrorState_ = 0.0f;
    fbIntegralErrorState_ = 0.0f;
  }
}

void __PID2Class::ComputeDerivative(float dt) {

  float ffErrorChange = ffDerivativeError_ - ffPreviousDerivativeError_;
  float fbErrorChange = fbDerivativeError_ - fbPreviousDerivativeError_;

  if ((Tf_ + dt) != 0) {
    ffDerivativeErrorState_ = Tf_ * (ffPreviousDerivativeErrorState_ + ffErrorChange) / (Tf_ + dt);
    fbDerivativeErrorState_ = Tf_ * (fbPreviousDerivativeErrorState_ + fbErrorChange) / (Tf_ + dt);
  } else {
    ffDerivativeErrorState_ = 0.0f;
    fbDerivativeErrorState_ = 0.0f;
  }

  ffPreviousDerivativeError_ = ffDerivativeError_;
  ffPreviousDerivativeErrorState_ = ffDerivativeErrorState_;

  fbPreviousDerivativeError_ = fbDerivativeError_;
  fbPreviousDerivativeErrorState_ = fbDerivativeErrorState_;
}

void __PID2Class::UpdateState(float dt) {
  // Protect for unlimited windup when Ki == 0
  if (Ki_ != 0.0f) {
    ffIntegralErrorState_ += (dt*ffIntegralError_);
    fbIntegralErrorState_ += (dt*fbIntegralError_);
  } else {
    ffIntegralErrorState_ = 0.0;
    fbIntegralErrorState_ = 0.0;
  }
}

void __PID2Class::CalculateCommand() {
  ff_ = Kp_*ffProportionalError_ + Ki_*ffIntegralErrorState_ + Kd_*ffDerivativeErrorState_;
  fb_ = Kp_*fbProportionalError_ + Ki_*fbIntegralErrorState_ + Kd_*fbDerivativeErrorState_;

  Output_ = ff_ - fb_;

  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (SatFlag_) {
    if (Output_ <= OutMin_) {
      Output_ = OutMin_;
      ff_ = Output_ + fb_;
      Saturated_ = -1;
      // Re-compute the integrator state
      // InitializeState(Output_);
      InitializeState(ff_, fb_);
    } else if (Output_ >= OutMax_) {
      Output_ = OutMax_;
      ff_ = Output_ + fb_;
      Saturated_ = 1;
      // Re-compute the integrator state
      // InitializeState(Output_);
      InitializeState(ff_, fb_);
    } else {
      Saturated_ = 0;
    }
  }
}

void __PID2Class::Reset() {
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

void __PID2Class::Clear() {
  Reset();

  Kp_ = 1.0f;
  Ki_ = 0.0f;
  Kd_ = 0.0f;
  Tf_ = 0.0f;
  b_ = 1.0f;
  c_ = 1.0f;
  SatFlag_ = false;
  OutMax_ = 0.0f;
  OutMin_ = 0.0f;

  Saturated_ = 0;
  Output_ = 0.0f;
}


/* State Space */
void __SSClass::Configure(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, float dt, bool SatFlag, Eigen::VectorXf yMax, Eigen::VectorXf yMin) {
  Clear(); // Clear and set to defaults

  A_ = A;
  B_ = B;
  C_ = C;
  D_ = D;
  SatFlag_ = SatFlag;
  yMax_ = yMax;
  yMin_ = yMin;

  uint8_t numU = B_.cols();
  uint8_t numX = A_.rows();
  uint8_t numY = C_.rows();

  y_.resize(numY);
  yMax_.resize(numY);
  yMin_.resize(numY);
  ySat_.resize(numY);

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

void __SSClass::Run(GenericFunction::Mode mode, Eigen::VectorXf u, float dt, Eigen::VectorXf *y, Eigen::VectorXi *ySat) {
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
  *ySat = ySat_;
}

void __SSClass::InitializeState(Eigen::VectorXf u, Eigen::VectorXf y, float dt) {
  x_ = CA_inv_ * (y - (CB_*dt + D_) * u);
}

void __SSClass::UpdateState(Eigen::VectorXf u, float dt) {

  uint8_t numX = A_.rows();
  Eigen::MatrixXf Ix = Eigen::MatrixXf::Identity(numX, numX);
  x_ = (A_*dt + Ix)*x_ + B_*u*dt;
}

void __SSClass::OutputEquation(Eigen::VectorXf u, float dt) {
  y_ = C_*x_ + D_*u;

  // saturate output
  if (SatFlag_ == true){
    for (int i=0; i < y_.size(); i++) {
      if (y_(i) <= yMin_(i)) {
        y_(i) = yMin_(i);
        ySat_(i) = -1;
      } else if (y_(i) >= yMax_(i)) {
        y_(i) = yMax_(i);
        ySat_(i) = 1;
      } else {
        ySat_(i) = 0;
      }
    }

    if (ySat_.cwiseAbs().any()) { // check if any of the outputs are saturated
      InitializeState(u, y_, dt); // Re-initialize the states with saturated outputs
    }
  }
}

void __SSClass::Reset() {
  x_.Zero(A_.rows()); // Reset to Zero
  y_.Zero(C_.cols()); // Reset to Zero
  ySat_.Zero(C_.cols()); // Reset to Zero

  mode_ = GenericFunction::Mode::kStandby;
  initLatch_ = false;
}

void __SSClass::Clear() {
  A_.resize(0,0);
  B_.resize(0,0);
  C_.resize(0,0);
  D_.resize(0,0);

  yMin_.resize(0);
  yMax_.resize(0);

  SatFlag_ = false;

  CA_inv_.resize(0,0);
  CB_.resize(0,0);

  Reset();
}
