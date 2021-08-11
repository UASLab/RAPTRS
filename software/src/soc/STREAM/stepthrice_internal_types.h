//
//  stepthrice_internal_types.h
//
//  Code generation for function 'stepthrice'
//


#ifndef STEPTHRICE_INTERNAL_TYPES_H
#define STEPTHRICE_INTERNAL_TYPES_H

// Include files
#include "rtwtypes.h"
#include "stepthrice_types.h"
#include "streamClass.h"
#include "coder_array.h"

// Type Definitions
struct stepthricePersistentData
{
  gb_struct_T dynamMdlParams;
  cb_struct_T algSettings;
  coder::bounded_array<float, 5001U, 1U> wDblSided;
  boolean_T wDblSided_not_empty;
  coder::array<creal32_T, 2U> Fu;
  boolean_T Fu_not_empty;
  coder::array<creal32_T, 2U> Fy;
  boolean_T Fy_not_empty;
};

#endif

// End of code generation (stepthrice_internal_types.h)
