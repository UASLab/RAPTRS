/*
AirData.h
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

#ifndef AIRDATA_h
#define AIRDATA_h

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
    float getAngle(float pTip, float pAngle1, float pAngle2, float pSide1, float pSide2, float kCal);
		float getAngleMeth2(float pTip, float pAngle, float kCal);

  private:
    const float A0 = 340.29f;   // standard sea level speed of sound, m/s
    const float P0 = 101325.0f; // standard sea level pressure, Pa
    const float T0 = 288.15f;   // standard sea level temperature, K
    const float L = 0.0065f;    // standard lapse rate, K/m
    const float R = 8.314f;     // gas constant, J/kg-mol
    const float M = 0.02895f;   // molecular mass dry air, kg/mol
    const float g = 9.807f;     // acceleration due to gravity, m/s/s
};

#endif
