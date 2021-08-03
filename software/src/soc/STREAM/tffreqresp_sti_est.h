//
//  tffreqresp_sti_est.h
//
//  Code generation for function 'tffreqresp_sti_est'
//


#ifndef TFFREQRESP_STI_EST_H
#define TFFREQRESP_STI_EST_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void tffreqresp_sti_est(const float freqRespLUT_w[150], const creal32_T
  freqRespLUT_freqResp[27300], const coder::array<float, 2U> &w, const float yi
  [4], float ui, coder::array<creal32_T, 3U> &resp);

#endif

// End of code generation (tffreqresp_sti_est.h)
