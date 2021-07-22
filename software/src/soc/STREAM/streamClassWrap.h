//
//  streamClass.h
//
//  Code generation for function 'streamClass'
//


#ifndef STREAMCLASSWRAP_H
#define STREAMCLASSWRAP_H

// Include files
#include "streamClass.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>



// Type Definitions
class streamClassWrap : public streamClass
{
 public:
  void set_uMeas(coder::array<float, 2U> UIN, const int u_ind[], const int u_ind_size);

  void set_yMeas(coder::array<float, 2U> YIN, const int y_ind[], const int y_ind_size);

};

#endif

// End of code generation (streamClass.h)
