//
//  stepSTREAMThrice.h
//
//  Code generation for function 'stepSTREAMThrice'
//


#ifndef STEPSTREAMTHRICE_H
#define STEPSTREAMTHRICE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class streamClass;

// Function Declarations
void b_stepSTREAMThrice(streamClass *aInstancePtr, const coder::array<float, 2U>
  &u, const coder::array<float, 2U> &y, float algSettings_fredaSettings_dt,
  float algSettings_fredaSettings_psdTaper);
void stepSTREAMThrice(streamClass *aInstancePtr, const coder::array<float, 2U>
                      &u, const coder::array<float, 2U> &y, float
                      algSettings_fredaSettings_dt, float
                      algSettings_fredaSettings_psdTaper);
void stepSTREAMThrice(streamClass *aInstancePtr, const coder::array<float, 2U>
                      &u, const coder::array<float, 2U> &y, const float
                      dynamicMdlParams_outputIndices[4], const float
                      dynamicMdlParams_ctrlSurfIndices[7], const float
                      dynamicMdlParams_freqRespLUT_w[150], const creal32_T
                      dynamicMdlParams_freqRespLUT_freqResp[27300], float
                      algSettings_wmax, float
                      algSettings_fredaSettings_useSmoothing, float
                      algSettings_fredaSettings_binRatio, float
                      algSettings_fredaSettings_binSize, float
                      algSettings_fredaSettings_dt, const float
                      algSettings_gustFunctionInputs[3], float sigout[3]);

#endif

// End of code generation (stepSTREAMThrice.h)
