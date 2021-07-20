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
    void svd(const double A[8], double U[8], double s[2], double V[4]);
  }
}

#endif

// End of code generation (svd.h)
