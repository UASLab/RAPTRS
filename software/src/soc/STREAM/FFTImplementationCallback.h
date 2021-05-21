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
          array<double, 2U> &x, int n2blue, int nfft, const ::coder::array<
          double, 2U> &costab, const ::coder::array<double, 2U> &sintab, const ::
          coder::array<double, 2U> &sintabinv, ::coder::array<creal_T, 2U> &y);
        static void doHalfLengthRadix2(streamClass *aInstancePtr, const ::coder::
          array<double, 2U> &x, ::coder::array<creal_T, 2U> &y, int nChan, int
          unsigned_nRows, int nrowsx, const ::coder::array<double, 2U> &costab,
          const ::coder::array<double, 2U> &sintab);
       protected:
        static void generate_twiddle_tables(int nRows, double costab_data[], int
          costab_size[2], double sintab_data[], int sintab_size[2], double
          sintabinv_data[], int sintabinv_size[2]);
        static void r2br_r2dit_trig(const ::coder::array<creal_T, 1U> &x, int
          n1_unsigned, const ::coder::array<double, 2U> &costab, const ::coder::
          array<double, 2U> &sintab, ::coder::array<creal_T, 1U> &y);
        static void b_r2br_r2dit_trig(const ::coder::array<creal_T, 1U> &x, int
          n1_unsigned, const ::coder::array<double, 2U> &costab, const ::coder::
          array<double, 2U> &sintab, ::coder::array<creal_T, 1U> &y);
        static void doHalfLengthBluestein(streamClass *aInstancePtr, const ::
          coder::array<double, 2U> &x, ::coder::array<creal_T, 2U> &y, int nChan,
          int nrowsx, int nRows, int nfft, const ::coder::array<creal_T, 1U>
          &wwc, const ::coder::array<double, 2U> &costab, const ::coder::array<
          double, 2U> &sintab, const ::coder::array<double, 2U> &costabinv,
          const ::coder::array<double, 2U> &sintabinv, int nRowsIdx);
      };
    }
  }
}

#endif

// End of code generation (FFTImplementationCallback.h)
