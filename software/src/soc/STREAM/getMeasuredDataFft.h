//
//  getMeasuredDataFft.h
//
//  Code generation for function 'getMeasuredDataFft'
//


#ifndef GETMEASUREDDATAFFT_H
#define GETMEASUREDDATAFFT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class streamClass;

// Function Declarations
void getMeasuredDataFft(streamClass *aInstancePtr, const coder::array<float, 2U>
  &u, const coder::array<float, 2U> &y, float psdTaper, float dt, coder::array<
  creal32_T, 2U> &Fy, coder::array<creal32_T, 2U> &Fu, float w_data[], int
  w_size[1]);

#endif

// End of code generation (getMeasuredDataFft.h)
