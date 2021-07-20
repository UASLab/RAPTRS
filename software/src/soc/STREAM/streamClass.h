//
//  streamClass.h
//
//  Code generation for function 'streamClass'
//


#ifndef STREAMCLASS_H
#define STREAMCLASS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct struct_T
{
  double Clp;
  double CLq;
  double Cmq;
  double CYr;
  double Cnr;
  double Cnp;
  double Clr;
};

struct b_struct_T
{
  double Upper;
  double Lower;
};

struct c_struct_T
{
  double BW;
  double RateLim;
  double PosLim;
  double NegLim;
};

struct d_struct_T
{
  double PosLim;
  double NegLim;
};

struct e_struct_T
{
  c_struct_T elevator;
  c_struct_T l_aileron;
  c_struct_T r_aileron;
  c_struct_T rudder;
  c_struct_T l_flap;
  c_struct_T r_flap;
  d_struct_T throttle;
};

struct f_struct_T
{
  double p_noise;
  double q_noise;
  double r_noise;
  double hx_noise;
  double hy_noise;
  double hz_noise;
  double ax_noise;
  double ay_noise;
  double az_noise;
  double p_bias;
  double q_bias;
  double r_bias;
  double hx_bias;
  double hy_bias;
  double hz_bias;
  double ax_bias;
  double ay_bias;
  double az_bias;
  double p_scf;
  double q_scf;
  double r_scf;
  double hx_scf;
  double hy_scf;
  double hz_scf;
  double ax_scf;
  double ay_scf;
  double az_scf;
};

struct g_struct_T
{
  double ias_noise;
  double h_noise;
  double alpha_noise;
  double beta_noise;
  double Pd_noise;
  double Ps_noise;
  double ias_bias;
  double h_bias;
  double alpha_bias;
  double beta_bias;
  double Pd_bias;
  double Ps_bias;
  double ias_scf;
  double h_scf;
  double alpha_scf;
  double beta_scf;
  double Pd_scf;
  double Ps_scf;
};

struct h_struct_T
{
  double NoiseOn;
  f_struct_T IMU;
  g_struct_T AirData;
};

struct i_struct_T
{
  double useSmoothing;
  double binRatio;
  double binSize;
  double dt;
  double psdTaper;
};

struct cell_wrap_1
{
  double f1[392];
};

struct cell_wrap_2
{
  double f1[28];
};

struct j_struct_T
{
  double pitch_gain;
  double roll_gain;
  double yaw_gain;
  double short_period[2];
  double rollmode;
  double dutchroll[2];
};

struct k_struct_T
{
  double rCG[3];
  double AeroCenter[3];
  double c;
  double b;
  double S;
  double rProp[3];
};

struct l_struct_T
{
  double Ixx;
  double Iyy;
  double Izz;
  double Ixz;
  double Matrix[9];
};

struct m_struct_T
{
  double data[336];
  double alpha[16];
  double phat[7];
  char notes[64];
};

struct n_struct_T
{
  double data[210];
  double alpha[14];
  double rhat[5];
  char notes[64];
};

struct o_struct_T
{
  double data[1248];
  double alpha[16];
  double beta[13];
  char dim[49];
};

struct p_struct_T
{
  double data[624];
  double alpha[16];
  double beta[13];
  double ail[3];
  char dim[66];
};

struct q_struct_T
{
  double data[3744];
  double alpha[16];
  double beta[13];
  double J[3];
  char dim[64];
};

struct r_struct_T
{
  double data[288];
  double alpha[16];
  double flap[2];
  double J[3];
  char dim[55];
};

struct s_struct_T
{
  double data[3744];
  double alpha[16];
  double beta[13];
  double rud[3];
  double J[3];
  char dim[63];
};

struct t_struct_T
{
  double data[432];
  double alpha[16];
  double ele[3];
  double J[3];
  char dim[59];
};

struct u_struct_T
{
  double data[1248];
  double alpha[16];
  double beta[13];
  char dim[55];
};

struct v_struct_T
{
  m_struct_T dC3_p;
  n_struct_T dC3_r;
  struct_T dampder;
  o_struct_T C6_bas;
  p_struct_T dC1_ail;
  q_struct_T dC6_pow;
  r_struct_T dC3_flap;
  s_struct_T dC2_rud;
  t_struct_T dC3_ele;
  double sym_data[1248];
  u_struct_T dC6_asym;
  double AsymmetryOn;
};

struct w_struct_T
{
  double Angles[3];
  double CT[5];
  double CP[5];
  double Power[3];
  double Jmp;
  double Radius;
  b_struct_T ThrottleOutputLimit;
  b_struct_T OmegaSaturation;
};

struct cell_0
{
  char f1[9];
  char f2[11];
  char f3[10];
};

struct x_struct_T
{
  double elevator[3];
  double aileron[3];
  double rudder[3];
  cell_0 descrip;
};

struct y_struct_T
{
  char aircraft[13];
  double Mass;
  k_struct_T Geometry;
  l_struct_T Inertia;
  v_struct_T Aero;
  w_struct_T Prop;
  e_struct_T Actuator;
  h_struct_T Sensors;
  double nctrls;
  x_struct_T actuators;
};

struct ab_struct_T
{
  double A[729];
  double B[351];
  double C[378];
  double D[182];
};

struct bb_struct_T
{
  cell_wrap_1 num[13];
  cell_wrap_2 den[13];
};

struct cb_struct_T
{
  double wmax;
  double dt;
  double winSize;
  double startTime;
  boolean_T pseudoRealTime;
  boolean_T useCalcTime;
  double stepSkip;
  boolean_T printDevOutput;
  boolean_T detrendData;
  i_struct_T fredaSettings;
  double gustFunction;
  double gustFunctionInputs[3];
};

struct db_struct_T
{
  double w[150];
  creal_T freqResp[27300];
};

struct eb_struct_T
{
  double airspeed;
  double cg_offset;
  boolean_T activate_flaps;
  char airframe[13];
  boolean_T doBareAirframeUpdate;
  boolean_T doActuatorUpdate;
};

struct fb_struct_T
{
  eb_struct_T flags;
  j_struct_T bareairframe;
  y_struct_T AC;
};

struct gb_struct_T
{
  fb_struct_T info;
  double outputIndices[4];
  double ctrlSurfIndices[7];
  double gustIndices;
  ab_struct_T sysSS;
  bb_struct_T sysTF;
  db_struct_T freqRespLUT;
};

struct b_steponceStackData
{
  struct {
    double a_data[5001];
    double y_data[5001];
  } f0;

  struct {
    double hcostab_data[5001];
  } f1;

  struct {
    double costable_data[5001];
    double sintable_data[5001];
    double unusedU0_data[5001];
    double hsintab_data[5001];
    double hcostabinv_data[5001];
    double hsintabinv_data[5001];
    creal_T ytmp_data[2500];
  } f2;

  struct {
    double taperIndex_data[7500];
    double tmp_data[7500];
    double x_data[7500];
  } f3;

  struct {
    double costab1q_data[5001];
  } f4;

  struct {
    double wDblSided_data[5001];
    double b_wDblSided_data[5001];
    double psd_data[5001];
  } f5;
};

class streamClass
{
 public:
  streamClass();
  ~streamClass();
  void steponce(double sigma[3]);
  b_steponceStackData *getStackData();
  gb_struct_T dynamMdlParams;
  cb_struct_T algSettings;
  coder::array<double, 2U> uMeas;
  coder::array<double, 2U> yMeas;
 private:
  b_steponceStackData SD_;
};

#endif

// End of code generation (streamClass.h)
