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
void getMeasuredDataFft(streamClass *aInstancePtr, const coder::array<double, 2U>
  &u, const coder::array<double, 2U> &y, double psdTaper, double dt, coder::
  array<creal_T, 2U> &Fy, coder::array<creal_T, 2U> &Fu, double w_data[], int
  w_size[1]);

#endif

// End of code generation (getMeasuredDataFft.h)
