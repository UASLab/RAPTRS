//
//  xrot.h
//
//  Code generation for function 'xrot'
//


#ifndef XROT_H
#define XROT_H

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
      void b_xrot(float x[12], int ix0, int iy0, float c, float s);
      void xrot(float x[9], int ix0, int iy0, float c, float s);
    }
  }
}

#endif

// End of code generation (xrot.h)
