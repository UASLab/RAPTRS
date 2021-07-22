//
//  trapz.h
//
//  Code generation for function 'trapz'
//


#ifndef TRAPZ_H
#define TRAPZ_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  float trapz(const float x_data[], const int x_size[1], const float y_data[],
              const int y_size[1]);
}

#endif

// End of code generation (trapz.h)
