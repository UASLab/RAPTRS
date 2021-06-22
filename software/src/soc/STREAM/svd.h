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
    void svd(const double A[12], double U[12], double s[3], double V[9]);
  }
}

#endif

// End of code generation (svd.h)
