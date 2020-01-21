/*
Copyright (c) 2011 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan
Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang
*/

// Reference Frames and Coordinates
// I - ECI (Earch Center Inertial): origin at Earth center
// E - ECEF (Earch Center Earth Fixed): origin at Earth center
// D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
// G - Geocentric: origin at Earth center, Uses spheroid definition
// L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
//
// All units meters and radians

#pragma once

#include <math.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

// Constants that are no longer used
// const double EarthRate = 0.00007292115;      // rotation rate of earth (rad/sec)
// const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid
// const double Flattening = 0.0033528106650;   // flattening of the ellipsoid
// const double Gravity0 = 9.7803730;           // zeroth coefficient for gravity model
// const double Gravity1 = 0.0052891;           // first coefficient for the gravity model
// const double Gravity2 = 0.0000059;           // second coefficient for the gravity model
// const double GravityNom = 9.81;              // nominal gravity
// const double Schuler2 = 1.533421593170545E-06; // Schuler Frequency (rad/sec) Squared

// This function calculates the rate of change of latitude, longitude, and altitude using WGS-84.
Vector3d L2D_Rate(Vector3d v_L, Vector3d pRef_D);
Vector3f L2D_Rate(Vector3f v_L, Vector3d pRef_D);

// This function calculates the angular velocity of the NED frame, also known as the navigation rate using WGS-84.
Vector3d NavRate(Vector3d v_L, Vector3d pRef_D);
Vector3f NavRate(Vector3f v_L, Vector3d pRef_D);

// This function calculates the ECEF Coordinate given the  Latitude, Longitude and Altitude.
Vector3d D2E(Vector3d p_D);

// This function calculates the Latitude, Longitude and Altitude given the ECEF Coordinates.
Vector3d E2D(Vector3d p_E);

// This function converts a vector in ecef to ned coordinate centered at pRef.
Vector3d E2L(Vector3d p_E, Vector3d pRef_D);
Matrix3d E2L(Vector3d pRef_D); // Return the T_E2L DCM from the Reference position in Geodetic
Vector4d E2L_Quat(Vector3d pRef_D); // Returns the E2L transformation as a quaternion.

// This function gives a skew symmetric matrix from a given vector w
Matrix3d Skew(Vector3d w);
Matrix3f Skew(Vector3f w);

// Quaternion to Euler angles (3-2-1)
Vector3d Quat2Euler(Vector4d quat);
Vector3f Quat2Euler(Vector4f quat);

// Quaternion to Euler
Vector4d Euler2Quat(Vector3d euler);
Vector4f Euler2Quat(Vector3f euler);

// Quaternion to T_N2B
Matrix3d Quat2DCM(Vector4d quat);
Matrix3f Quat2DCM(Vector4f quat);

// Quaternion Multiplication
Vector4d QuatMult(Vector4d quatA, Vector4d quatB);
Vector4f QuatMult(Vector4f quatA, Vector4f quatB);

// Update the Earth Dimensions
void EarthRadiusUpdate(double lat, double *Rew, double *Rns);

// Angle Limits
double ConstrainAngle180(double dta); // maps angle to +/- 180
float ConstrainAngle180(float dta); // maps angle to +/- 180
double ConstrainAngle360(double dta); // maps angle to 0-360
float ConstrainAngle360(float dta); // maps angle to 0-360
