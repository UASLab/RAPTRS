//
//  psdft.h
//
//  Code generation for function 'psdft'
//


#ifndef PSDFT_H
#define PSDFT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void logsmooth(coder::array<double, 2U> &yin, const double binWidths_data[],
               const int binWidths_size[1], double dt, double w_data[], int
               w_size[1], coder::array<double, 2U> &smy);
void logsmooth(coder::array<creal_T, 2U> &yin, const double binWidths_data[],
               const int binWidths_size[1], double dt, double w_data[], int
               w_size[1], coder::array<creal_T, 2U> &smy);

#endif

// End of code generation (psdft.h)
