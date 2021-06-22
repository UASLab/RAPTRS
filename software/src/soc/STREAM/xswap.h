//
//  xswap.h
//
//  Code generation for function 'xswap'
//


#ifndef XSWAP_H
#define XSWAP_H

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
      void b_xswap(double x[12], int ix0, int iy0);
      void xswap(double x[9], int ix0, int iy0);
    }
  }
}

#endif

// End of code generation (xswap.h)
