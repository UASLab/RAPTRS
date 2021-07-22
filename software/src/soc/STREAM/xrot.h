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
      void xrot(float x[4], int iy0, float c, float s);
      void xrot(float x[8], int ix0, int iy0, float c, float s);
    }
  }
}

#endif

// End of code generation (xrot.h)
