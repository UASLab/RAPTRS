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
void logsmooth(coder::array<float, 2U> &yin, const float binWidths_data[], const
               int binWidths_size[1], float dt, float w_data[], int w_size[1],
               coder::array<float, 2U> &smy);
void logsmooth(coder::array<creal32_T, 2U> &yin, const float binWidths_data[],
               const int binWidths_size[1], float dt, float w_data[], int
               w_size[1], coder::array<creal32_T, 2U> &smy);

#endif

// End of code generation (psdft.h)
