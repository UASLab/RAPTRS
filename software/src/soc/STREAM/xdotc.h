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
      double xdotc(int n, const double x[12], int ix0, const double y[12], int
                   iy0);
      double xdotc(const double x[9], const double y[9], int iy0);
    }
  }
}

#endif

// End of code generation (xdotc.h)
