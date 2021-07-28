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

// Type Definitions
struct steptwoPersistentData
{
  gb_struct_T dynamMdlParams;
  cb_struct_T algSettings;
  float RHSbar[12];
  float sigmaYHat[4];
  float phiHat[6];
};

#endif

// End of code generation (steptwo_internal_types.h)
