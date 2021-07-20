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

// Function Declarations
void removeInputsFromMeasFft(const coder::array<creal_T, 2U> &Fy, const coder::
  array<creal_T, 2U> &Fu, const double w_data[], const double ctrlSurfInd[7],
  const double outInd[4], const double freqRespLUT_w[150], const creal_T
  freqRespLUT_freqResp[27300], coder::array<creal_T, 2U> &Fyuse);

#endif

// End of code generation (removeInputsFromMeasFft.h)
