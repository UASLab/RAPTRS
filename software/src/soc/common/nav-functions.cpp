/*
Copyright (c) 2011 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan
Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang
*/

#include "nav-functions.h"

// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3d L2D_Rate(Vector3d v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRadiusUpdate(pRef_D(0), &Rew, &Rns);

    Vector3d v_D;
    v_D(0) = v_L(0) / (Rns + pRef_D(2));
    v_D(1) = v_L(1) / ((Rew + pRef_D(2)) * cos(pRef_D(0)));
    v_D(2) = -v_L(2);

    return v_D;
}
Vector3f L2D_Rate(Vector3f v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRadiusUpdate(pRef_D(0), &Rew, &Rns);

    Vector3f v_D;
    v_D(0) = v_L(0) / (Rns + pRef_D(2));
    v_D(1) = v_L(1) / ((Rew + pRef_D(2)) * cos(pRef_D(0)));
    v_D(2) = -v_L(2);

    return v_D;
}

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Vector3d NavRate(Vector3d v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRadiusUpdate(pRef_D(0), &Rew, &Rns);

    Vector3d nr;
    nr(0) = v_L(1) / (Rew + pRef_D(2));
    nr(1) = -v_L(0) / (Rns + pRef_D(2));
    nr(2) = -v_L(1) * tan(pRef_D(0)) / (Rew + pRef_D(2));

    return nr;
}
Vector3f NavRate(Vector3f v_L, Vector3d pRef_D) {
    double Rew, Rns;
    EarthRadiusUpdate(pRef_D(0), &Rew, &Rns);

    Vector3f nr;
    nr(0) = v_L(1) / (Rew + pRef_D(2));
    nr(1) = -v_L(0) / (Rns + pRef_D(2));
    nr(2) = -v_L(1) * tan(pRef_D(0)) / (Rew + pRef_D(2));

    return nr;
}

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Vector3d D2E(Vector3d p_D) {
    double sinlat = sin(p_D(0));
    double coslat = cos(p_D(0));
    double coslon = cos(p_D(1));
    double sinlon = sin(p_D(1));
    double alt = p_D(2);

    double denom = fabs(1.0 - (ECC2 * sinlat * sinlat));

    double Rew = EarthRadius / sqrt(denom);

    Vector3d p_E;
    p_E(0) = (Rew + alt) * coslat * coslon;
    p_E(1) = (Rew + alt) * coslat * sinlon;
    p_E(2) = (Rew * (1.0 - ECC2) + alt) * sinlat;

    return p_E;
}

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d E2D( Vector3d p_E ) {
    const double Squash = 0.9966471893352525192801545;
    const double ra2 = 1.0 / (EarthRadius * EarthRadius);
    const double e2 = fabs(1 - Squash * Squash);
    const double e4 = e2 * e2;

    // according to
    // H. Vermeille,
    // Direct transformation from geocentric to geodetic ccordinates,
    // Journal of Geodesy (2002) 76:451-454
    Vector3d p_D;
    double X = p_E(0);
    double Y = p_E(1);
    double Z = p_E(2);

    double XXpYY = X*X+Y*Y;
    if( XXpYY + Z*Z < 25 ) {
    	// This function fails near the geocenter region, so catch
    	// that special case here.  Define the innermost sphere of
    	// small radius as earth center and return the coordinates
    	// 0/0/-EQURAD. It may be any other place on geoide's surface,
    	// the Northpole, Hawaii or Wentorf. This one was easy to code
    	// ;-)
    	p_D(0) = 0.0;
    	p_D(1) = 0.0;
    	p_D(2) = -EarthRadius;
    	return p_D;
    }

    double sqrtXXpYY = sqrt(XXpYY);
    double p = XXpYY*ra2;
    double q = Z*Z*(1-e2)*ra2;
    double r = 1/6.0*(p+q-e4);
    double s = e4*p*q/(4*r*r*r);

    /*
       s*(2+s) is negative for s = [-2..0]
       slightly negative values for s due to floating point rounding errors
       cause nan for sqrt(s*(2+s))
       We can probably clamp the resulting parable to positive numbers
    */
    if( s >= -2.0 && s <= 0.0 ) {
      s = 0.0;
    }

    double t = pow(1+s+sqrt(s*(2+s)), 1/3.0);
    double u = r*(1+t+1/t);
    double v = sqrt(u*u+e4*q);
    double w = e2*(u+v-q)/(2*v);
    double k = sqrt(u+v+w*w)-w;
    double D = k*sqrtXXpYY/(k+e2);
    double sqrtDDpZZ = sqrt(D*D+Z*Z);

    p_D(1) = 2*atan2(Y, X+sqrtXXpYY);
    p_D(0) = 2*atan2(Z, D+sqrtDDpZZ);
    p_D(2) = (k+e2-1)*sqrtDDpZZ/k;

    return p_D;
}

// This function converts a vector in ECEF to NED coordinate centered at pRef.
// FIXIT - CDR - This doesn't appear to work as intended.
// Vector3d E2L(Vector3d p_E, Vector3d pRef_D) {
//   Vector3d p_NED;
//   p_NED(2) = -cos(pRef_D(0)) * (cos(pRef_D(1)) * p_E(0) + sin(pRef_D(1)) * p_E(1)) - sin(pRef_D(0)) * p_E(2);
//   p_NED(1) = -sin(pRef_D(1)) * p_E(0) + cos(pRef_D(1)) * p_E(1);
//   p_NED(0) = -sin(pRef_D(0)) * (cos(pRef_D(1)) * p_E(0) + sin(pRef_D(1)) * p_E(1)) + cos(pRef_D(0)) * p_E(2);
//   return p_NED;
// }

Vector3d E2L(Vector3d p_E, Vector3d pRef_D) {
    Matrix3d T_E2L = E2L(pRef_D);
    Vector3d p_NED = T_E2L * (p_E - D2E(pRef_D));
    return p_NED;
}

Matrix3d E2L(Vector3d pRef_D) {
    double sin_lat = sin(pRef_D(0));
    double sin_lon = sin(pRef_D(1));
    double cos_lat = cos(pRef_D(0));
    double cos_lon = cos(pRef_D(1));

    Matrix3d T_E2L;
    T_E2L(0,0) = -sin_lat*cos_lon; T_E2L(1,0) = -sin_lat*sin_lon; T_E2L(2,0) = cos_lat;
    T_E2L(0,1) = -sin_lon;         T_E2L(1,1) = cos_lon;          T_E2L(2,1) = 0.0f;
    T_E2L(0,2) = -cos_lat*cos_lat; T_E2L(1,2) = -cos_lat*sin_lon; T_E2L(2,2) = -sin_lat;

    return T_E2L;
}

Vector4d E2L_Quat(Vector3d pRef_D) {
  double zd2 = 0.5 * pRef_D(1);
  double yd2 = -0.25 * M_PI - 0.5 * pRef_D(0);

  Vector4d quat;
  quat(0) = cos(zd2) * cos(yd2);
  quat(1) = -sin(zd2) * sin(yd2);
  quat(2) = cos(zd2) * sin(yd2);
  quat(3) = sin(zd2) * cos(yd2);

  return quat;
}

// This function gives a skew symmetric matrix from a given vector w
Matrix3d Skew(Vector3d w) {
  Matrix3d C;

  C(0,0) =  0.0;	C(0,1) = -w(2);	C(0,2) =  w(1);
  C(1,0) =  w(2);	C(1,1) =  0.0;	C(1,2) = -w(0);
  C(2,0) = -w(1);	C(2,1) =  w(0);	C(2,2) =  0.0;

  return C;
}
Matrix3f Skew(Vector3f w) {
  Matrix3f C;

  C(0,0) =  0.0;	C(0,1) = -w(2);	C(0,2) =  w(1);
  C(1,0) =  w(2);	C(1,1) =  0.0;	C(1,2) = -w(0);
  C(2,0) = -w(1);	C(2,1) =  w(0);	C(2,2) =  0.0;

  return C;
}

// Quaternion to Euler angles (3-2-1)
Vector3d Quat2Euler(Vector4d quat) {
  double m11 = 2*(quat(0)*quat(0) + quat(1)*quat(1)) - 1;
  double m12 = 2*(quat(1)*quat(2) + quat(0)*quat(3));
  double m13 = 2*(quat(1)*quat(3) - quat(0)*quat(2));
  double m23 = 2*(quat(2)*quat(3) + quat(0)*quat(1));
  double m33 = 2*(quat(0)*quat(0) + quat(3)*quat(3)) - 1;

  Vector3d euler;
  euler(2) = atan2(m12, m11);
  euler(1) = asin(-m13);
  euler(0) = atan2(m23, m33);

  return euler;
}
Vector3f Quat2Euler(Vector4f quat) {
  float m11 = 2*(quat(0)*quat(0) + quat(1)*quat(1)) - 1;
  float m12 = 2*(quat(1)*quat(2) + quat(0)*quat(3));
  float m13 = 2*(quat(1)*quat(3) - quat(0)*quat(2));
  float m23 = 2*(quat(2)*quat(3) + quat(0)*quat(1));
  float m33 = 2*(quat(0)*quat(0) + quat(3)*quat(3)) - 1;

  Vector3f euler;
  euler(2) = atan2(m12, m11);
  euler(1) = asin(-m13);
  euler(0) = atan2(m23, m33);

  return euler;
}

// Computes a quaternion from the given euler angles
Vector4d Euler2Quat(Vector3d euler) {
  double sin_psi = sin(0.5f * euler(2));
  double cos_psi = cos(0.5f * euler(2));
  double sin_the = sin(0.5f * euler(1));
  double cos_the = cos(0.5f * euler(1));
  double sin_phi = sin(0.5f * euler(0));
  double cos_phi = cos(0.5f * euler(0));

  Vector4d quat;
  quat(0) = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi;
  quat(1) = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi;
  quat(2) = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi;
  quat(3) = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi;

  return quat;
}
Vector4f Euler2Quat(Vector3f euler) {
  float sin_psi = sin(0.5f * euler(2));
  float cos_psi = cos(0.5f * euler(2));
  float sin_the = sin(0.5f * euler(1));
  float cos_the = cos(0.5f * euler(1));
  float sin_phi = sin(0.5f * euler(0));
  float cos_phi = cos(0.5f * euler(0));

  Vector4f quat;
  quat(0) = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi;
  quat(1) = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi;
  quat(2) = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi;
  quat(3) = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi;

  return quat;
}

// Quaternion to DCM
Matrix3d Quat2DCM(Vector4d quat) {
  Matrix3d T;

  T(0,0) = 2*(quat(0)*quat(0) + quat(1)*quat(1)) - 1;
  T(1,1) = 2*(quat(0)*quat(0) + quat(2)*quat(2)) - 1;
  T(2,2) = 2*(quat(0)*quat(0) + quat(3)*quat(3)) - 1;

  T(0,1) = 2*(quat(1)*quat(2) + quat(0)*quat(3));
  T(0,2) = 2*(quat(1)*quat(3) - quat(0)*quat(2));

  T(1,0) = 2*(quat(1)*quat(2) - quat(0)*quat(3));
  T(1,2) = 2*(quat(2)*quat(3) + quat(0)*quat(1));

  T(2,0) = 2*(quat(1)*quat(3) + quat(0)*quat(2));
  T(2,1) = 2*(quat(2)*quat(3) - quat(0)*quat(1));

  return T;
}
Matrix3f Quat2DCM(Vector4f quat) {
  Matrix3f T;

  T(0,0) = 2*(quat(0)*quat(0) + quat(1)*quat(1)) - 1;
  T(1,1) = 2*(quat(0)*quat(0) + quat(2)*quat(2)) - 1;
  T(2,2) = 2*(quat(0)*quat(0) + quat(3)*quat(3)) - 1;

  T(0,1) = 2*(quat(1)*quat(2) + quat(0)*quat(3));
  T(0,2) = 2*(quat(1)*quat(3) - quat(0)*quat(2));

  T(1,0) = 2*(quat(1)*quat(2) - quat(0)*quat(3));
  T(1,2) = 2*(quat(2)*quat(3) + quat(0)*quat(1));

  T(2,0) = 2*(quat(1)*quat(3) + quat(0)*quat(2));
  T(2,1) = 2*(quat(2)*quat(3) - quat(0)*quat(1));

  return T;
}

// euler angles (3-2-1) to quaternion
Vector4f euler2quat(Vector3f euler) {
  Vector4f quat;

  quat(0) = cosf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) + sinf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);
  quat(1) = cosf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f) - sinf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f);
  quat(2) = cosf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) + sinf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);
  quat(3) = sinf(euler(2) / 2.0f) * cosf(euler(1) / 2.0f) * cosf(euler(0) / 2.0f) - cosf(euler(2) / 2.0f) * sinf(euler(1) / 2.0f) * sinf(euler(0) / 2.0f);

  return quat;
}

// Quaternion mulitplicaton
Vector4d QuatMult(Vector4d quatA, Vector4d quatB) {
  Vector4d quat;

  quat(0) = quatA(0)*quatB(0) - (quatA(1)*quatB(1) + quatA(2)*quatB(2) + quatA(3)*quatB(3));
  quat(1) = quatA(0)*quatB(1) + quatB(0)*quatA(1) + quatA(2)*quatB(3) - quatA(3)*quatB(2);
  quat(2) = quatA(0)*quatB(2) + quatB(0)*quatA(2) + quatA(3)*quatB(1) - quatA(1)*quatB(3);
  quat(3) = quatA(0)*quatB(3) + quatB(0)*quatA(3) + quatA(1)*quatB(2) - quatA(2)*quatB(1);

  return quat;
}
Vector4f QuatMult(Vector4f quatA, Vector4f quatB) {
  Vector4f quat;

  quat(0) = quatA(0)*quatB(0) - (quatA(1)*quatB(1) + quatA(2)*quatB(2) + quatA(3)*quatB(3));
  quat(1) = quatA(0)*quatB(1) + quatB(0)*quatA(1) + quatA(2)*quatB(3) - quatA(3)*quatB(2);
  quat(2) = quatA(0)*quatB(2) + quatB(0)*quatA(2) + quatA(3)*quatB(1) - quatA(1)*quatB(3);
  quat(3) = quatA(0)*quatB(3) + quatB(0)*quatA(3) + quatA(1)*quatB(2) - quatA(2)*quatB(1);

  return quat;
}

// Earth Radius Updates
void EarthRadiusUpdate(double lat, double *Rew, double *Rns) {
  double denom = fabs(1.0 - (ECC2 * sin(lat) * sin(lat)));
  double sqrt_denom = sqrt(denom);

  (*Rew) = EarthRadius / sqrt_denom;
  (*Rns) = EarthRadius * (1 - ECC2) / (denom * sqrt_denom);
}


// bound yaw angle between -180 and 180
double ConstrainAngle180(double dta) {
  if(dta >  M_PI) dta -= (M_PI * 2.0f);
  if(dta < -M_PI) dta += (M_PI * 2.0f);
  return dta;
}
float ConstrainAngle180(float dta) {
  if(dta >  M_PI) dta -= (M_PI * 2.0f);
  if(dta < -M_PI) dta += (M_PI * 2.0f);
  return dta;
}

// bound heading angle between 0 and 360
double ConstrainAngle360(double dta){
  dta = fmod(dta, 2.0f * M_PI);
  if (dta < 0)
    dta += 2.0f * M_PI;
  return dta;
}
float ConstrainAngle360(float dta){
  dta = fmod(dta, 2.0f * M_PI);
  if (dta < 0)
    dta += 2.0f * M_PI;
  return dta;
}
