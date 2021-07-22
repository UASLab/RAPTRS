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
void removeInputsFromMeasFft(const coder::array<creal32_T, 2U> &Fy, const coder::
  array<creal32_T, 2U> &Fu, const float w_data[], const float ctrlSurfInd[7],
  const float outInd[4], const float freqRespLUT_w[150], const creal32_T
  freqRespLUT_freqResp[27300], coder::array<creal32_T, 2U> &Fyuse);

#endif

// End of code generation (removeInputsFromMeasFft.h)
