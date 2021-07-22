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
    void svd(const float A[8], float U[8], float s[2], float V[4]);
  }
}

#endif

// End of code generation (svd.h)
