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
void getAutoSpectra(const coder::array<creal_T, 2U> &Fy, double
                    fredaSettings_useSmoothing, double fredaSettings_binRatio,
                    double fredaSettings_binSize, double fredaSettings_dt,
                    double w_data[], int w_size[1], coder::array<double, 2U>
                    &psd);

#endif

// End of code generation (getAutoSpectra.h)
