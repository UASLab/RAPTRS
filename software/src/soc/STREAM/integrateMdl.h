//
//  integrateMdl.h
//
//  Code generation for function 'integrateMdl'
//


#ifndef INTEGRATEMDL_H
#define INTEGRATEMDL_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void integrateMdl(const float mdlFreqResp_data[], const float PHIbar_data[],
                  const float freq_data[], const int freq_size[1], unsigned char
                  freqUpperInd, float RHSbar[8]);

#endif

// End of code generation (integrateMdl.h)
