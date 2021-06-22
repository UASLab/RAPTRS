//
//  tffreqresp_sti.h
//
//  Code generation for function 'tffreqresp_sti'
//


#ifndef TFFREQRESP_STI_H
#define TFFREQRESP_STI_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_1;
struct cell_wrap_2;

// Function Declarations
void tffreqresp_sti(const cell_wrap_1 num[13], const cell_wrap_2 den[13], const
                    double w_data[], const int w_size[1], const double yi[4],
                    double ui, coder::array<creal_T, 3U> &resp);

#endif

// End of code generation (tffreqresp_sti.h)
