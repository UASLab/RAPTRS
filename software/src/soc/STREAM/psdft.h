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
void logsmooth(const coder::array<float, 2U> &yin, const coder::array<float, 1U>
               &binWidths, float dt, coder::array<float, 1U> &w, coder::array<
               float, 2U> &smy);

#endif

// End of code generation (psdft.h)
