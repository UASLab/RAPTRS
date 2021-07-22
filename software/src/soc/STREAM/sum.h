//
//  sum.h
//
//  Code generation for function 'sum'
//


#ifndef SUM_H
#define SUM_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  void sum(const ::coder::array<float, 2U> &x, float y_data[], int y_size[2]);
  float sum(const float x_data[], const int x_size[1]);
  void sum(const ::coder::array<creal32_T, 2U> &x, creal32_T y_data[], int
           y_size[2]);
}

#endif

// End of code generation (sum.h)
