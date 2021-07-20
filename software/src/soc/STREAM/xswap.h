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
      void b_xswap(double x[8]);
      void xswap(double x[4]);
    }
  }
}

#endif

// End of code generation (xswap.h)
