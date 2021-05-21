//
//  xnrm2.h
//
//  Code generation for function 'xnrm2'
//


#ifndef XNRM2_H
#define XNRM2_H

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
      double xnrm2(int n, const double x[12], int ix0);
      double xnrm2(const double x[3], int ix0);
    }
  }
}

#endif

// End of code generation (xnrm2.h)
