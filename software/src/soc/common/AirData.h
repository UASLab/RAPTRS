/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor and Chris Regan
*/

#pragma once

// compile using Arduino IDE or standard C/C++
#if defined(ARDUINO)
	#include "Arduino.h"
#else
	#include <stdint.h>
	#include <math.h>
#endif

class AirData{
	public:
		float getIAS(float qc);
		float getEAS(float qc, float p);
		float getTAS(float AS, float T);
		float getPressureAltitude(float p);
    float getAGL(float p, float c);
		float getMSL(float H, float h);
		float getDensityAltitude(float p, float T);
    float getApproxTemp(float T, float h);
    float getDensity(float p, float T);
    float getAngle1(float pTip, float pAngle1, float pAngle2, float pSide1, float pSide2, float kCal);
		float getAngle2(float pTip, float pAngle, float kCal);

  private:
    const float A0 = 340.29f;   // standard sea level speed of sound, m/s
    const float P0 = 101325.0f; // standard sea level pressure, Pa
    const float T0 = 288.15f;   // standard sea level temperature, K
    const float L = 0.0065f;    // standard lapse rate, K/m
    const float R = 8.314f;     // gas constant, J/kg-mol
    const float M = 0.02895f;   // molecular mass dry air, kg/mol
    const float g = 9.807f;     // acceleration due to gravity, m/s/s
};
