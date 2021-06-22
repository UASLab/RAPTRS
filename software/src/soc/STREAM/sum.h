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
  void sum(const ::coder::array<double, 2U> &x, double y_data[], int y_size[2]);
  void sum(const ::coder::array<creal_T, 2U> &x, creal_T y_data[], int y_size[2]);
}

#endif

// End of code generation (sum.h)
