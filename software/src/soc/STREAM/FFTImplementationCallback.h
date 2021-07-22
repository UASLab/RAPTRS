//
//  FFTImplementationCallback.h
//
//  Code generation for function 'FFTImplementationCallback'
//


#ifndef FFTIMPLEMENTATIONCALLBACK_H
#define FFTIMPLEMENTATIONCALLBACK_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class streamClass;

// Type Definitions
namespace coder
{
  namespace internal
  {
    namespace fft
    {
      class FFTImplementationCallback
      {
       public:
        static void dobluesteinfft(streamClass *aInstancePtr, const ::coder::
          array<float, 2U> &x, int n2blue, int nfft, const float costab_data[],
          const int costab_size[2], const float sintab_data[], const float
          sintabinv_data[], ::coder::array<creal32_T, 2U> &y);
        static void doHalfLengthRadix2(const ::coder::array<float, 2U> &x, ::
          coder::array<creal32_T, 2U> &y, int nChan, int unsigned_nRows, int
          nrowsx, const float costab_data[], const int costab_size[2], const
          float sintab_data[]);
       protected:
        static void r2br_r2dit_trig(const ::coder::array<creal32_T, 1U> &x, int
          n1_unsigned, const float costab_data[], const float sintab_data[], ::
          coder::array<creal32_T, 1U> &y);
        static void b_r2br_r2dit_trig(const ::coder::array<creal32_T, 1U> &x,
          int n1_unsigned, const float costab_data[], const float sintab_data[],
          ::coder::array<creal32_T, 1U> &y);
        static void doHalfLengthBluestein(streamClass *aInstancePtr, const ::
          coder::array<float, 2U> &x, ::coder::array<creal32_T, 2U> &y, int
          nChan, int nrowsx, int nRows, int nfft, const ::coder::array<creal32_T,
          1U> &wwc, const float costab_data[], const int costab_size[2], const
          float sintab_data[], const float costabinv_data[], const float
          sintabinv_data[], int nRowsIdx);
      };
    }
  }
}

#endif

// End of code generation (FFTImplementationCallback.h)
