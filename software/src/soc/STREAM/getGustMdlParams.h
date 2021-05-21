//
//  getGustMdlParams.h
//
//  Code generation for function 'getGustMdlParams'
//


#ifndef GETGUSTMDLPARAMS_H
#define GETGUSTMDLPARAMS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class streamClass;

// Function Declarations
void getGustMdlParams(streamClass *aInstancePtr, const double gustFunInputs[3],
                      const double w_data[], const int w_size[1], double phiHat
                      [6], coder::array<double, 3U> &PHIbar);

#endif

// End of code generation (getGustMdlParams.h)
