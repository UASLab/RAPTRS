//
//  removeInputsFromMeasFft.h
//
//  Code generation for function 'removeInputsFromMeasFft'
//


#ifndef REMOVEINPUTSFROMMEASFFT_H
#define REMOVEINPUTSFROMMEASFFT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_1;
struct cell_wrap_2;

// Function Declarations
void removeInputsFromMeasFft(const coder::array<creal_T, 2U> &Fy, const coder::
  array<creal_T, 2U> &Fu, const double w_data[], const cell_wrap_1 sysTF_num[13],
  const cell_wrap_2 sysTF_den[13], const double ctrlSurfInd[7], const double
  outInd[4], coder::array<creal_T, 2U> &Fyuse);

#endif

// End of code generation (removeInputsFromMeasFft.h)
