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
      float xnrm2(int n, const float x[12], int ix0);
      float xnrm2(const float x[3], int ix0);
    }
  }
}

#endif

// End of code generation (xnrm2.h)
