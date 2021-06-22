//
//  getSysGustMdlFreqResp.h
//
//  Code generation for function 'getSysGustMdlFreqResp'
//


#ifndef GETSYSGUSTMDLFREQRESP_H
#define GETSYSGUSTMDLFREQRESP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct cell_wrap_1;
struct cell_wrap_2;

// Function Declarations
void getSysGustMdlFreqResp(const cell_wrap_1 sysTF_num[13], const cell_wrap_2
  sysTF_den[13], const double outInd[4], const double freq_data[], const int
  freq_size[1], coder::array<double, 3U> &freqResp2);

#endif

// End of code generation (getSysGustMdlFreqResp.h)
