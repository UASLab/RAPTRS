//
//  xdotc.h
//
//  Code generation for function 'xdotc'
//


#ifndef XDOTC_H
#define XDOTC_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      float xdotc(int n, const float x[12], int ix0, const float y[12], int iy0);
      float xdotc(const float x[9], const float y[9], int iy0);
    }
  }
}

#endif

// End of code generation (xdotc.h)
