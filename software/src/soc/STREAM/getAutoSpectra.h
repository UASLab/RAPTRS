//
//  getAutoSpectra.h
//
//  Code generation for function 'getAutoSpectra'
//


#ifndef GETAUTOSPECTRA_H
#define GETAUTOSPECTRA_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void getAutoSpectra(const coder::array<creal32_T, 2U> &Fy, float
                    fredaSettings_useSmoothing, float fredaSettings_binRatio,
                    float fredaSettings_binSize, float fredaSettings_dt, coder::
                    array<float, 1U> &w, coder::array<float, 2U> &psd);

#endif

// End of code generation (getAutoSpectra.h)
