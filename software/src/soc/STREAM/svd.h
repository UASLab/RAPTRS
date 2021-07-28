//
//  svd.h
//
//  Code generation for function 'svd'
//


#ifndef SVD_H
#define SVD_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    void svd(const float A[12], float U[12], float s[3], float V[9]);
  }
}

#endif

// End of code generation (svd.h)
