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
void fftfunc(streamClass *aInstancePtr, coder::array<double, 2U> &y, double tfr,
             double dt, coder::array<creal_T, 2U> &Fy, double w_data[], int
             w_size[1]);

#endif

// End of code generation (fftfunc.h)
