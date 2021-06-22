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
  double trapz(const double x_data[], const int x_size[1], const double y_data[],
               const int y_size[1]);
}

#endif

// End of code generation (trapz.h)
