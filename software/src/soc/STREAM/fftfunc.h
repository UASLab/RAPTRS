//
//  fftfunc.h
//
//  Code generation for function 'fftfunc'
//


#ifndef FFTFUNC_H
#define FFTFUNC_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class streamClass;

// Function Declarations
void fftfunc(streamClass *aInstancePtr, coder::array<float, 2U> &y, float tfr,
             float dt, coder::array<creal32_T, 2U> &Fy, float w_data[], int
             w_size[1]);

#endif

// End of code generation (fftfunc.h)
