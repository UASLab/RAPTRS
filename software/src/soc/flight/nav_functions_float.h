/*
Copyright (c) 2011 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor
Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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

// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3f llarate(Vector3f V, Vector3d lla);

// This function calculates the angular velocity of the NED frame,
// also known as the navigation rate using WGS-84.
Vector3d navrate(Vector3d V, Vector3d lla);

// This function calculates the ECEF Coordinate given the
// Latitude, Longitude and Altitude.
Vector3d lla2ecef(Vector3d lla);

// This function calculates the Latitude, Longitude and Altitude given
// the ECEF Coordinates.
Vector3d ecef2lla( Vector3d ecef_pos );

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3f ecef2ned(Vector3d ecef, Vector3d pos_ref);

// Return a quaternion rotation from the earth centered to the
// simulation usual horizontal local frame from given longitude and
// latitude.  The horizontal local frame used in simulations is the
// frame with x-axis pointing north, the y-axis pointing eastwards and
// the z axis pointing downwards.  (Returns the ecef2ned
// transformation as a quaternion.)
Quaterniond lla2quat(double lon_rad, double lat_rad);

// This function gives a skew symmetric matrix from a given vector w
Matrix3f sk(Vector3f w);

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3f quat2eul(Quaternionf q);

// Computes a quaternion from the given euler angles
Quaternionf eul2quat(float phi, float the, float psi);

// Quaternion to C_N2B
Matrix3f quat2dcm(Quaternionf q);
