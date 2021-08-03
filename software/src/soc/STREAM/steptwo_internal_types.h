//
//  steptwo_internal_types.h
//
//  Code generation for function 'steptwo'
//


#ifndef STEPTWO_INTERNAL_TYPES_H
#define STEPTWO_INTERNAL_TYPES_H

// Include files
#include "rtwtypes.h"
#include "steptwo_types.h"
#include "streamClass.h"
#include "coder_array.h"

// Type Definitions
struct steptwoPersistentData
{
  gb_struct_T dynamMdlParams;
  cb_struct_T algSettings;
  coder::array<creal32_T, 2U> Fyhat;
  boolean_T Fyhat_not_empty;
  coder::array<float, 2U> wDblSided;
  coder::array<creal32_T, 2U> Fu;
};

#endif

// End of code generation (steptwo_internal_types.h)
