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

// Function Declarations
void getSysGustMdlFreqResp(const double outInd[4], const double w_data[], const
  int w_size[1], const double freqRespLUT_w[150], const creal_T
  freqRespLUT_freqResp[27300], coder::array<double, 3U> &freqResp2);

#endif

// End of code generation (getSysGustMdlFreqResp.h)
