//
//  integrateMdl.h
//
//  Code generation for function 'integrateMdl'
//


#ifndef INTEGRATEMDL_H
#define INTEGRATEMDL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void integrateMdl(const coder::array<double, 3U> &mdlFreqResp, const coder::
                  array<double, 3U> &PHIbar, const double freq_data[], const int
                  freq_size[1], double freqUpperInd, double RHSbar[8]);

#endif

// End of code generation (integrateMdl.h)
