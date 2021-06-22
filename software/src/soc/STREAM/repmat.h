//
//  repmat.h
//
//  Code generation for function 'repmat'
//


#ifndef REPMAT_H
#define REPMAT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  void repmat(const creal_T a_data[], const int a_size[1], double varargin_2, ::
              coder::array<creal_T, 2U> &b);
}

#endif

// End of code generation (repmat.h)
