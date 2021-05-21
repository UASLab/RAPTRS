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
      void b_xaxpy(int n, double a, const double x[4], int ix0, double y[12],
                   int iy0);
      void xaxpy(int n, double a, const double x[12], int ix0, double y[4], int
                 iy0);
      void xaxpy(int n, double a, int ix0, double y[12], int iy0);
      void xaxpy(double a, double y[9], int iy0);
    }
  }
}

#endif

// End of code generation (xaxpy.h)
