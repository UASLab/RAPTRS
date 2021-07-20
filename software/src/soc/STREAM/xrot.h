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
      void xrot(double x[4], int iy0, double c, double s);
      void xrot(double x[8], int ix0, int iy0, double c, double s);
    }
  }
}

#endif

// End of code generation (xrot.h)
