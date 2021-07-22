//
//  fifo.cpp
//
//  Code generation for function 'fifo'
//


// Include files
#include "fifo.h"
#include "coder_array.h"

// Function Definitions
void fifo(coder::array<float, 2U> &A, const float Arow_data[], const int
          Arow_size[2])
{
  coder::array<float, 2U> c_A;
  int b_A;
  int i;
  int i1;
  int loop_ub;
  int result;
  short input_sizes_idx_0;
  signed char sizes_idx_0;
  bool empty_non_axis_sizes;
  if (2 > A.size(0)) {
    i = -1;
    i1 = -1;
  } else {
    i = 0;
    i1 = A.size(0) - 1;
  }

  loop_ub = i1 - i;
  if ((loop_ub != 0) && (A.size(1) != 0)) {
    result = A.size(1);
  } else if ((Arow_size[0] != 0) && (Arow_size[1] != 0)) {
    result = Arow_size[1];
  } else {
    if (A.size(1) > 0) {
      result = A.size(1);
    } else {
      result = 0;
    }

    if (Arow_size[1] > result) {
      result = Arow_size[1];
    }
  }

  empty_non_axis_sizes = (result == 0);
  if (empty_non_axis_sizes || ((loop_ub != 0) && (A.size(1) != 0))) {
    input_sizes_idx_0 = static_cast<short>(loop_ub);
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((Arow_size[0] != 0) && (Arow_size[1] != 0))) {
    sizes_idx_0 = static_cast<signed char>(Arow_size[0]);
  } else {
    sizes_idx_0 = 0;
  }

  b_A = A.size(1) - 1;
  c_A.set_size(loop_ub, (b_A + 1));
  for (i1 = 0; i1 <= b_A; i1++) {
    for (int i2 = 0; i2 < loop_ub; i2++) {
      c_A[i2 + c_A.size(0) * i1] = A[((i + i2) + A.size(0) * i1) + 1];
    }
  }

  A.set_size((input_sizes_idx_0 + sizes_idx_0), result);
  for (i = 0; i < result; i++) {
    loop_ub = input_sizes_idx_0;
    for (i1 = 0; i1 < loop_ub; i1++) {
      A[i1 + A.size(0) * i] = c_A[i1 + input_sizes_idx_0 * i];
    }
  }

  for (i = 0; i < result; i++) {
    loop_ub = sizes_idx_0;
    for (i1 = 0; i1 < loop_ub; i1++) {
      A[(i1 + input_sizes_idx_0) + A.size(0) * i] = Arow_data[i1 + Arow_size[0] *
        i];
    }
  }
}

// End of code generation (fifo.cpp)
