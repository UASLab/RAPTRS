//
//  xaxpy.h
//
//  Code generation for function 'xaxpy'
//


#ifndef XAXPY_H
#define XAXPY_H

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
      void xaxpy(int n, double a, int ix0, double y[8], int iy0);
    }
  }
}

#endif

// End of code generation (xaxpy.h)
